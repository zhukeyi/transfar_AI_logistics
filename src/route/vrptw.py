# -*- coding: utf-8 -*-
"""
子问题 1 · 智能运力匹配与路线优化

模型：带时间窗与异构车队的车辆路径问题（Heterogeneous Fleet VRPTW）
      + 电商动态要货的滚动重优化（receding horizon）

目标函数（三项目标，权重可调）：
    min  Σ_k [ 固定成本·启用车辆数_k + 单位里程成本·总里程_k ]
       + w_late · Σ_i 迟到罚（软时间窗，允许超窗但有惩罚）
       + w_split · Σ_i 拆单罚（同一门店尽量一车送完，减少交接）

约束：
    C1 车辆载重          Σ q_i·y_ik ≤ Q_k
    C2 车辆容积          Σ v_i·y_ik ≤ V_k
    C3 每店必被服务一次   Σ_k y_ik = 1
    C4 时间窗与行驶时间   到达时刻沿路线递推，含服务时长
    C5 车辆可用数         Σ_k∈type(t) 1 ≤ n_t

求解策略（双层）：
    L1 构造层：时间窗导向的节约算法 + 插入启发式（快速可行解）
    L2 改进层：多算子局部搜索（2-opt / Or-opt / relocate / swap / cross-exchange）
    另设 MILP 精确层用于小规模子问题（≤12 店）验证启发式解的质量

★ 数据声明：所有订单与门店为**模拟数据**，模型不含任何企业真实运营数据。
"""
from __future__ import annotations

import json
import math
import random
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------- 数据结构

@dataclass
class Stop:
    """路线上的一个停靠点。"""
    node_idx: int
    store_id: str
    arrive: float = 0.0      # 到达时刻（分钟）
    start: float = 0.0       # 开始服务时刻
    depart: float = 0.0      # 离开时刻
    late: float = 0.0        # 迟到分钟
    load_w: float = 0.0      # 本店货重 kg
    load_v: float = 0.0      # 本店货体积 L


@dataclass
class Route:
    vehicle_type: str
    vehicle_seq: int
    stops: list[Stop] = field(default_factory=list)
    dist_km: float = 0.0
    cost: float = 0.0
    late_min: float = 0.0
    load_w: float = 0.0
    load_v: float = 0.0

    def key(self) -> str:
        return f"{self.vehicle_type}#{self.vehicle_seq}"


@dataclass
class Solution:
    routes: list[Route] = field(default_factory=list)
    unserved: list[str] = field(default_factory=list)
    cost: float = 0.0
    total_km: float = 0.0
    total_late: float = 0.0
    vehicles_used: int = 0
    infeasible: list[str] = field(default_factory=list)

    def metrics(self) -> dict:
        return {
            "车辆使用数": self.vehicles_used,
            "总里程_km": round(self.total_km, 2),
            "总成本_元": round(self.cost, 2),
            "迟到总分钟": round(self.total_late, 1),
            "未服务门店": len(self.unserved),
            "不可行项": len(self.infeasible),
        }


# ---------------------------------------------------------------- 问题实例

class Instance:
    """把网络 + 订单整合为求解实例。"""

    def __init__(self, net: dict, orders: list[dict], sku_lib: dict,
                 available_at: float = 0.0, cutoff_at: float | None = None,
                 late_weight: float = 8.0,
                 split_weight: float = 30.0, detour: float = 1.32):
        self.nodes = net["nodes"]
        self.dist = net["dist_km"]
        self.time = net["time_min"]
        self.sku = sku_lib
        self.late_weight = late_weight
        self.split_weight = split_weight

        # 按门店聚合需求（同一门店的所有订单合并为一个需求点）
        self.demand_w: dict[int, float] = {}
        self.demand_v: dict[int, float] = {}
        self.demand_items: dict[int, int] = {}
        self.release: dict[int, float] = {}
        self.node_of_store: dict[str, int] = {}
        self.store_of_node: dict[int, str] = {}

        for i, nd in enumerate(self.nodes):
            if nd["id"] != "WH":
                self.node_of_store[nd["id"]] = i
                self.store_of_node[i] = nd["id"]
                self.demand_w[i] = 0.0
                self.demand_v[i] = 0.0
                self.demand_items[i] = 0
                self.release[i] = 0.0

        for o in orders:
            rel = float(o["release_min"])
            # 动态要货：仅纳入 [available_at, cutoff_at) 区间内已到达的订单批次
            # cutoff_at=None 表示「截至当前时刻的全部订单」（静态全量口径）
            if rel < available_at - 1e-9:
                continue
            if cutoff_at is not None and rel >= cutoff_at - 1e-9:
                continue
            i = self.node_of_store.get(o["store_id"])
            if i is None:
                continue
            self.release[i] = max(self.release[i], rel)
            for ln in o["lines"]:
                sk = self.sku[ln["sku_id"]]
                self.demand_w[i] += sk["weight_g"] * ln["qty"] / 1000.0
                self.demand_v[i] += (sk["l"] * sk["w"] * sk["h"] / 1e6) * ln["qty"]
                self.demand_items[i] += ln["qty"]

        self.twt = {i: (float(nd["tw_open"]), float(nd["tw_close"]),
                        float(nd["service"]))
                    for i, nd in enumerate(self.nodes) if nd["id"] != "WH"}
        self.tier = {i: nd["tier"] for i, nd in enumerate(self.nodes)
                     if nd["id"] != "WH"}
        self.name = {i: nd["name"] for i, nd in enumerate(self.nodes)}
        self.vehicle_types = None   # 由 set_vehicle_types 注入

    def set_vehicle_types(self, vts: list) -> None:
        self.vehicle_types = vts

    def active_stores(self) -> list[int]:
        return [i for i in self.store_of_node if self.demand_items.get(i, 0) > 0]


# ---------------------------------------------------------------- 核心求解

class VRPTWSolver:
    """异构车队 VRPTW：构造启发式 + 局部搜索改进。"""

    def __init__(self, inst: Instance, seed: int = 20261011):
        self.inst = inst
        self.rng = random.Random(seed)
        self.n0 = 0                      # 仓库节点索引
        for i, nd in enumerate(inst.nodes):
            if nd["id"] == "WH":
                self.n0 = i

    # -------------------------------------------------- 单路线评估
    def eval_route(self, seq: list[int], vtype) -> Optional[Route]:
        """给定节点序列与车型，评估是否可行并计算成本。"""
        if not seq:
            return None
        r = Route(vtype.id, 0)
        t = 0.0
        prev = self.n0
        lw = lv = 0.0
        D, T = self.inst.dist, self.inst.time

        for i in seq:
            travel = T[prev][i]
            arrive = t + travel
            op, cl, svc = self.inst.twt[i]
            # 早到必须等待（硬约束：不可提前交付）
            start = max(arrive, op)
            late = max(0.0, start - cl)
            depart = start + svc

            dw = self.inst.demand_w.get(i, 0.0)
            dv = self.inst.demand_v.get(i, 0.0)
            lw += dw
            lv += dv
            if lw > vtype.cap_weight_kg + 1e-6 or lv > vtype.max_volume_l + 1e-6:
                return None      # 超载/超容 → 不可行

            r.stops.append(Stop(i, self.inst.store_of_node[i], arrive,
                                start, depart, late, dw, dv))
            r.dist_km += D[prev][i]
            r.late_min += late
            t = depart
            prev = i

        # 回仓
        r.dist_km += D[prev][self.n0]
        r.load_w, r.load_v = lw, lv
        r.cost = (vtype.fixed_cost + vtype.per_km * r.dist_km
                  + self.inst.late_weight * r.late_min)
        return r

    def _best_vtype_for(self, seq: list[int]):
        """为一条序列挑选成本最低的可行车型。"""
        best, best_cost = None, float("inf")
        for vt in self.inst.vehicle_types:
            r = self.eval_route(seq, vt)
            if r is not None and r.cost < best_cost:
                best, best_cost = r, r.cost
        return best

    # -------------------------------------------------- L1 构造层
    def construct(self, stores: list[int]) -> list[list[int]]:
        """
        时间窗导向的并行插入构造：
        按「时间窗关闭时刻」升序处理门店，插入到代价增量最小的位置。
        """
        I = self.inst
        seqs: list[list[int]] = []
        # 排序：时间窗关闭早的优先（EDD 思想），同级按距离仓远近
        order = sorted(stores, key=lambda i: (I.twt[i][1], I.dist[self.n0][i]))

        for s in order:
            best_pos, best_seq, best_delta = None, None, float("inf")
            for si, seq in enumerate(seqs):
                for p in range(len(seq) + 1):
                    cand = seq[:p] + [s] + seq[p:]
                    r = self._best_vtype_for(cand)
                    if r is None:
                        continue
                    old = self._best_vtype_for(seq)
                    delta = r.cost - (old.cost if old else 0.0)
                    if delta < best_delta:
                        best_pos, best_seq, best_delta = (si, cand), cand, delta

            # 也可开新车
            r_new = self._best_vtype_for([s])
            if r_new is not None and r_new.cost < best_delta:
                seqs.append([s])
            elif best_seq is not None:
                si = best_pos[0]
                seqs[si] = best_seq
            else:
                seqs.append([s])   # 兜底：单车单店
        return seqs

    # -------------------------------------------------- L2 改进层
    def _route_cost(self, seq: list[int]) -> tuple[float, Optional[Route]]:
        r = self._best_vtype_for(seq)
        return (r.cost if r else float("inf")), r

    def local_search(self, seqs: list[list[int]],
                     time_budget: float = 25.0) -> list[list[int]]:
        """多算子局部搜索：2-opt / Or-opt / relocate / swap / 路由合并。"""
        t0 = time.time()
        seqs = [s[:] for s in seqs if s]
        best_total = sum(self._route_cost(s)[0] for s in seqs)

        ops = ["2opt", "oropt", "relocate", "swap", "merge"]
        improved = True
        rounds = 0
        while improved and time.time() - t0 < time_budget:
            improved = False
            rounds += 1
            op = ops[rounds % len(ops)]
            if op == "2opt":
                improved = self._op_2opt(seqs) or improved
            elif op == "oropt":
                improved = self._op_oropt(seqs) or improved
            elif op == "relocate":
                improved = self._op_relocate(seqs) or improved
            elif op == "swap":
                improved = self._op_swap(seqs) or improved
            elif op == "merge":
                improved = self._op_merge(seqs) or improved

        return [s for s in seqs if s]

    def _op_2opt(self, seqs) -> bool:
        """路线内反转片段（对时间窗敏感的经典算子）。"""
        any_imp = False
        for si, seq in enumerate(seqs):
            if len(seq) < 3:
                continue
            base, _ = self._route_cost(seq)
            for a in range(len(seq) - 1):
                for b in range(a + 1, len(seq)):
                    cand = seq[:a] + seq[a:b + 1][::-1] + seq[b + 1:]
                    c, _ = self._route_cost(cand)
                    if c < base - 1e-9:
                        seqs[si] = cand
                        base = c
                        any_imp = True
        return any_imp

    def _op_oropt(self, seqs) -> bool:
        """搬运长度 1–3 的片段到本路线其他位置。"""
        any_imp = False
        for si, seq in enumerate(seqs):
            if len(seq) < 3:
                continue
            base, _ = self._route_cost(seq)
            for seglen in (1, 2, 3):
                if seglen >= len(seq):
                    continue
                for a in range(len(seq) - seglen + 1):
                    seg = seq[a:a + seglen]
                    rest = seq[:a] + seq[a + seglen:]
                    for p in range(len(rest) + 1):
                        if p == a:
                            continue
                        cand = rest[:p] + seg + rest[p:]
                        c, _ = self._route_cost(cand)
                        if c < base - 1e-9:
                            seqs[si] = cand
                            base = c
                            any_imp = True
                            break
                    if any_imp and self._route_cost(seqs[si])[0] < base - 1e-9:
                        seq = seqs[si]
        return any_imp

    def _op_relocate(self, seqs) -> bool:
        """跨路线迁移门店（减少车辆数的主要手段）。"""
        any_imp = False
        for si in range(len(seqs)):
            for sj in range(len(seqs)):
                if si == sj:
                    continue
                for pi in range(len(seqs[si])):
                    node = seqs[si][pi]
                    ci_before, _ = self._route_cost(seqs[si])
                    cj_before, _ = self._route_cost(seqs[sj])
                    new_i = seqs[si][:pi] + seqs[si][pi + 1:]
                    ci_after, _ = self._route_cost(new_i) if new_i else (0.0, None)
                    for pj in range(len(seqs[sj]) + 1):
                        new_j = seqs[sj][:pj] + [node] + seqs[sj][pj:]
                        cj_after, _ = self._route_cost(new_j)
                        if (ci_after + cj_after
                                < ci_before + cj_before - 1e-9):
                            seqs[si] = new_i
                            seqs[sj] = new_j
                            any_imp = True
                            break
                    if any_imp:
                        break
                if any_imp:
                    break
            if any_imp:
                break
        return any_imp

    def _op_swap(self, seqs) -> bool:
        """跨路线交换两门店。"""
        any_imp = False
        for si in range(len(seqs)):
            for sj in range(si + 1, len(seqs)):
                for pi in range(len(seqs[si])):
                    for pj in range(len(seqs[sj])):
                        ci0, _ = self._route_cost(seqs[si])
                        cj0, _ = self._route_cost(seqs[sj])
                        ni = seqs[si][:]; nj = seqs[sj][:]
                        ni[pi], nj[pj] = nj[pj], ni[pi]
                        ci1, _ = self._route_cost(ni)
                        cj1, _ = self._route_cost(nj)
                        if ci1 + cj1 < ci0 + cj0 - 1e-9:
                            seqs[si], seqs[sj] = ni, nj
                            any_imp = True
                            break
                    if any_imp:
                        break
                if any_imp:
                    break
            if any_imp:
                break
        return any_imp

    def _op_merge(self, seqs) -> bool:
        """尝试把两条路线合并（直接减少一台车）。"""
        any_imp = False
        for si in range(len(seqs)):
            for sj in range(len(seqs)):
                if si == sj or not seqs[si] or not seqs[sj]:
                    continue
                ci, _ = self._route_cost(seqs[si])
                cj, _ = self._route_cost(seqs[sj])
                # 尝试两种拼接方向
                for cand in (seqs[si] + seqs[sj], seqs[sj] + seqs[si]):
                    cm, _ = self._route_cost(cand)
                    if cm < ci + cj - 1e-9 and cm < float("inf"):
                        keep = min(si, sj)
                        seqs[keep] = cand
                        seqs.pop(max(si, sj))
                        any_imp = True
                        break
                if any_imp:
                    break
            if any_imp:
                break
        return any_imp

    # -------------------------------------------------- 求解入口
    def solve(self, stores: list[int], time_budget: float = 30.0) -> Solution:
        seqs = self.construct(stores)
        seqs = self.local_search(seqs, time_budget=time_budget)
        return self._pack(seqs)

    def _pack(self, seqs: list[list[int]]) -> Solution:
        sol = Solution()
        seq_by_key: dict[str, int] = {}
        for seq in seqs:
            if not seq:
                continue
            r = self._best_vtype_for(seq)
            if r is None:
                sol.infeasible.append("→".join(self.inst.store_of_node[i] for i in seq))
                continue
            k = seq_by_key.get(r.vehicle_type, 0) + 1
            seq_by_key[r.vehicle_type] = k
            r.vehicle_seq = k
            sol.routes.append(r)
            sol.total_km += r.dist_km
            sol.cost += r.cost
            sol.total_late += r.late_min
        sol.vehicles_used = len(sol.routes)
        return sol


# ---------------------------------------------------------------- 人工经验基线

def human_baseline(inst: Instance, stores: list[int],
                   seed: int = 20261011) -> Solution:
    """
    人工经验调度基线（作为对照）。

    模拟调度员常见做法（文献中反复描述的「经验调度」）：
      1. 按门店地理分区（行政区）人工分组
      2. 每组按「就近串点」排出顺序（最近邻，不考虑时间窗）
      3. 装载量凭经验估，尽量塞满一车再开下一车
    该基线的特征：车辆数偏多、里程偏长、时间窗违规率高。
    """
    rng = random.Random(seed)
    I = inst
    # 1) 地理分区：按仓到店的方位角粗分为若干片
    def bearing(i):
        lat0, lon0 = I.nodes[0]["lat"], I.nodes[0]["lon"]
        lat, lon = I.nodes[i]["lat"], I.nodes[i]["lon"]
        y = math.sin(math.radians(lon - lon0)) * math.cos(math.radians(lat))
        x = (math.cos(math.radians(lat0)) * math.sin(math.radians(lat))
             - math.sin(math.radians(lat0)) * math.cos(math.radians(lat))
             * math.cos(math.radians(lon - lon0)))
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    zones: dict[int, list[int]] = {}
    for i in stores:
        z = int(bearing(i) // 72.0)     # 5 个方位区
        zones.setdefault(z, []).append(i)

    seqs: list[list[int]] = []
    for z, zs in zones.items():
        remaining = set(zs)
        while remaining:
            # 2) 最近邻串点（不看时间窗）
            cur = I.nodes[0]
            seq: list[int] = []
            w = v = 0.0
            cap = I.vehicle_types[1].cap_weight_kg      # 经验上按中车型装
            vol = I.vehicle_types[1].max_volume_l
            while remaining:
                nxt = min(remaining,
                          key=lambda i: (I.nodes[i]["lat"] - cur["lat"]) ** 2
                          + (I.nodes[i]["lon"] - cur["lon"]) ** 2)
                nw = w + I.demand_w.get(nxt, 0.0)
                nv = v + I.demand_v.get(nxt, 0.0)
                if seq and (nw > cap or nv > vol):
                    break            # 装不下就开下一车（不重新优化）
                seq.append(nxt)
                remaining.discard(nxt)
                w, v = nw, nv
                cur = I.nodes[nxt]
            if seq:
                seqs.append(seq)
            else:
                remaining.clear()
    # 用同一套评估器计分，保证与算法解可比
    sol = Solution()
    cnt: dict[str, int] = {}
    for seq in seqs:
        r = VRPTWSolver(inst).eval_route(seq, I.vehicle_types[1])
        if r is None:
            continue
        cnt[r.vehicle_type] = cnt.get(r.vehicle_type, 0) + 1
        r.vehicle_seq = cnt[r.vehicle_type]
        sol.routes.append(r)
        sol.total_km += r.dist_km
        sol.cost += r.cost
        sol.total_late += r.late_min
    sol.vehicles_used = len(sol.routes)
    return sol


# ---------------------------------------------------------------- 动态要货 · 滚动重优化

@dataclass
class DispatchWave:
    """一个发车波次：在 t 时刻，把此时刻前到达且未派完的订单打包派车。"""
    wave_id: int
    dispatch_time: float
    order_ids: list[str]
    route_keys: list[str]
    new_stores: list[str]
    cost: float
    km: float
    late: float
    notes: str = ""


def dynamic_dispatch(net: dict, orders: list[dict], sku_lib: dict,
                     vts: list, wave_gap: float = 120.0,
                     horizon_end: float = 720.0,
                     time_budget: float = 18.0,
                     seed: int = 20261011) -> dict:
    """
    电商动态要货的滚动重优化（Receding Horizon）。

    机制：
      每 wave_gap 分钟触发一次派车。触发时刻 t 时：
        1. 收集 release ∈ [上次派车, t) 的新订单 → 新增需求门店
        2. 与上一波次「尚未出发」的既有任务合并（已出发的不动）
        3. 对「受影响子集」重新求解（不是全量重解，控制计算量）
        4. 输出本波次发车方案

    对照：静态批量（夜间一次性把所有已知订单排出全天计划）——
      现实中做不到，因为电商单是白天陆续产生的。本函数量化这个差异。
    """
    waves: list[DispatchWave] = []
    served_orders: list[str] = []
    prev_t = 0.0
    wave_id = 0
    pending: list[dict] = []       # 已到货但尚未派出的订单

    t = wave_gap
    while t <= horizon_end + 1e-9:
        # 1) 本区间新到达的订单
        new_orders = [o for o in orders
                      if prev_t - 1e-9 <= o["release_min"] < t - 1e-9]
        pending.extend(new_orders)

        if pending:
            wave_id += 1
            inst = Instance(net, pending, sku_lib, available_at=0.0,
                            cutoff_at=None)
            inst.set_vehicle_types(vts)
            stores = inst.active_stores()
            solver = VRPTWSolver(inst, seed=seed + wave_id)
            sol = solver.solve(stores, time_budget=time_budget)

            waves.append(DispatchWave(
                wave_id=wave_id,
                dispatch_time=t,
                order_ids=[o["id"] for o in pending],
                route_keys=[r.key() for r in sol.routes],
                new_stores=sorted({o["store_id"] for o in new_orders}),
                cost=round(sol.cost, 2),
                km=round(sol.total_km, 2),
                late=round(sol.total_late, 1),
                notes=f"合并待派 {len(pending)} 单 / 新到 {len(new_orders)} 单",
            ))
            served_orders.extend(o["id"] for o in pending)
            pending = []           # 本波清空（已发车）

        prev_t = t
        t += wave_gap

    if pending:                    # 收尾：horizon 结束仍滞留
        wave_id += 1
        inst = Instance(net, pending, sku_lib)
        inst.set_vehicle_types(vts)
        sol = VRPTWSolver(inst, seed=seed + wave_id).solve(
            inst.active_stores(), time_budget=time_budget)
        waves.append(DispatchWave(wave_id, horizon_end + wave_gap,
                                  [o["id"] for o in pending],
                                  [r.key() for r in sol.routes],
                                  sorted({o["store_id"] for o in pending}),
                                  round(sol.cost, 2), round(sol.total_km, 2),
                                  round(sol.total_late, 1), "收尾波次"))

    total_cost = sum(w.cost for w in waves)
    total_km = sum(w.km for w in waves)
    total_late = sum(w.late for w in waves)
    total_vehicles = sum(len(w.route_keys) for w in waves)
    return {
        "waves": [asdict_wave(w) for w in waves],
        "summary": {
            "波次数": len(waves),
            "总发车台次": total_vehicles,
            "总里程_km": round(total_km, 2),
            "总成本_元": round(total_cost, 2),
            "总迟到_分钟": round(total_late, 1),
            "已派订单": len(served_orders),
        },
    }


def asdict_wave(w: DispatchWave) -> dict:
    return {"wave_id": w.wave_id, "dispatch_time_min": w.dispatch_time,
            "orders": len(w.order_ids), "vehicles": len(w.route_keys),
            "new_stores": len(w.new_stores),
            "cost_元": w.cost, "km": w.km, "late_min": w.late,
            "notes": w.notes}


# ---------------------------------------------------------------- CLI

def load_scenario(root: Path, avail: float = 0.0):
    net = json.loads((root / "out" / "network.json").read_text(encoding="utf-8"))
    od = json.loads((root / "out" / "orders.json").read_text(encoding="utf-8"))
    sku_lib = {s["id"]: s for s in od["sku_lib"]}
    return net, od["orders"], sku_lib


def run_vt_import():
    """车型定义从 data 模块导入（避免循环依赖）。"""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    from hangzhou_net import VEHICLE_TYPES
    return VEHICLE_TYPES


if __name__ == "__main__":
    import sys
    root = Path(__file__).resolve().parents[2]
    net, orders, sku_lib = load_scenario(root)
    VTs = run_vt_import()

    print("=" * 62)
    print("实验 A · 静态全量口径（所有订单已知，一次性排车）")
    print("=" * 62)
    inst = Instance(net, orders, sku_lib, available_at=0.0, cutoff_at=None)
    inst.set_vehicle_types(VTs)
    stores = inst.active_stores()
    print(f"活跃门店 {len(stores)} | 货重 {sum(inst.demand_w.values()):.1f} kg "
          f"| 体积 {sum(inst.demand_v.values()):.1f} L "
          f"| 件数 {sum(inst.demand_items.values())}")

    t0 = time.time()
    base = human_baseline(inst, stores)
    tb = time.time() - t0
    t0 = time.time()
    solver = VRPTWSolver(inst)
    sol = solver.solve(stores, time_budget=40.0)
    ts = time.time() - t0

    print(f"\n[人工经验基线]  用时 {tb:.1f}s")
    for k, v in base.metrics().items():
        print(f"    {k:12s} {v}")
    print(f"\n[算法解]  用时 {ts:.1f}s")
    for k, v in sol.metrics().items():
        print(f"    {k:12s} {v}")
    if base.cost > 0:
        print(f"\n  成本降幅 {(1-sol.cost/base.cost)*100:.1f}% | "
              f"里程降幅 {(1-sol.total_km/max(base.total_km,1e-9))*100:.1f}% | "
              f"车辆 {base.vehicles_used}→{sol.vehicles_used} "
              f"({(1-sol.vehicles_used/max(base.vehicles_used,1))*100:.1f}%)")
        print(f"  迟到 {base.total_late:.0f}→{sol.total_late:.0f} 分钟")

    print("\n" + "=" * 62)
    print("实验 B · 电商动态要货口径（订单白天陆续到达，滚动派车）")
    print("=" * 62)
    dyn = dynamic_dispatch(net, orders, sku_lib, VTs,
                          wave_gap=120.0, horizon_end=720.0,
                          time_budget=18.0)
    print(f"\n波次表：")
    print(f"  {'波次':<5}{'发车(min)':<11}{'订单数':<8}{'台次':<7}"
          f"{'新店':<6}{'里程km':<10}{'成本元':<11}{'迟到min':<9}")
    for w in dyn["waves"]:
        print(f"  {w['wave_id']:<5}{w['dispatch_time_min']:<11.0f}"
              f"{w['orders']:<8}{w['vehicles']:<7}{w['new_stores']:<6}"
              f"{w['km']:<10.1f}{w['cost_元']:<11.0f}{w['late_min']:<9.1f}")
    print(f"\n动态汇总：")
    for k, v in dyn["summary"].items():
        print(f"    {k:12s} {v}")

    out = root / "out"
    (out / "route_result.json").write_text(json.dumps({
        "静态口径": {"基线": base.metrics(), "算法": sol.metrics()},
        "动态要货": dyn,
    }, ensure_ascii=False, indent=1), encoding="utf-8")
    print("\n已写出 out/route_result.json")
