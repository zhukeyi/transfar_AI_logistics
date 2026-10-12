# -*- coding: utf-8 -*-
"""
MILP 精确层 · 用于验证启发式解的质量

两个用途：
  A. 小规模 VRPTW 精确解（≤12 门店）：用集合划分模型或弧流模型精确求解，
     与启发式解对比，量化启发式的次优性（gap）。
  B. 装载问题的精确解（见 loading 模块）。

求解器：scipy.optimize.milp（内置 HiGHS，BSD 许可，无需额外安装）

模型选择说明：
  - 弧流模型 (arc flow) 变量数 O(n²)，适合 n ≤ 12
  - 这里实现「两阶段」：先生成所有可行路线（列生成的思想），
    再解集合覆盖/划分问题（SPP），对小规模实例可精确求解
"""
from __future__ import annotations

import itertools
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import milp, LinearConstraint, Bounds
from scipy.sparse import lil_matrix


# ---------------------------------------------------------------- 路线枚举

def enumerate_feasible_routes(inst, vtype, max_routes: int = 60000,
                              max_len: int | None = None):
    """
    枚举所有「可行」路线（子集 + 顺序）。

    可行性：载重/容积不超限，且存在满足时间窗的到达序列。
    这里只枚举子集并检查是否存在可行顺序（用最近时间窗优先排序后校验）。
    返回 (frozenset(节点), 成本, 里程) 列表。

    max_len: 单条路线允许的最大门店数。None = 不限制（枚举到全部门店数）。
             ★ 早期版本固定为 8，导致 9 店实例无法形成单车路线，
               精确解反而劣于启发式（gap 为负）。已修正为动态上限。
    """
    stores = inst.active_stores()
    n = len(stores)
    routes = []
    upper = n if max_len is None else min(max_len, n)
    # 按子集大小枚举（限制 max_len 控制规模）
    for k in range(1, upper + 1):
        for combo in itertools.combinations(stores, k):
            w = sum(inst.demand_w[i] for i in combo)
            v = sum(inst.demand_v[i] for i in combo)
            if w > vtype.cap_weight_kg + 1e-6 or v > vtype.max_volume_l + 1e-6:
                continue
            # 求该子集的最优可行顺序（k ≤ 8 时全排列可行）
            best_cost, best_seq = _best_order(inst, combo, vtype)
            if best_seq is None:
                continue
            routes.append((frozenset(combo), best_cost, best_seq))
            if len(routes) > max_routes:
                return routes
    return routes


def _best_order(inst, combo, vtype, tw_aware: bool = True):
    """
    在子集上求最优顺序（Held-Karp DP）。

    ★★ 正确性要点（两次踩坑后固化，勿轻易改）：
      状态 = (已访问子集 mask, 最后节点 j)，标签 = (累计成本 cost, 当前时刻 t)。
      支配规则必须同时满足 cost' ≤ cost 且 t' ≤ t 才可剪枝，因为：
        - cost 已含「已发生的迟到罚」（沉没成本），并非只含里程
        - 若 t' 更晚，则后续每个时间窗的迟到风险都不同，不能仅凭 cost 剪枝
      早期版本只按 cost 单维比较、并用随机排列截断，导致
        ① DP 结果劣于启发式（gap 为负）
        ② DP 自报成本与其回溯出的路径不一致（236.60 vs 实际 303.01）
      现改为严格双维 Pareto 剪枝 + 确定性回溯，并以暴力全排列交叉验证
      （见 tests/verify_exact_dp.py）。

    返回 (含固定成本的总成本, 节点序列)。若子集不可行返回 (None, None)。
    """
    n0 = 0
    for i, nd in enumerate(inst.nodes):
        if nd["id"] == "WH":
            n0 = i
    nodes = list(combo)
    n = len(nodes)

    LATE_W = inst.late_weight
    D, T = inst.dist, inst.time
    INF = float("inf")

    # dp[(mask, last)] = [Label, ...]（append-only，索引稳定）
    # Label = (cost, t, parent_key, parent_index, alive)
    #   ★ 索引稳定性是硬要求：子标签记录的是「父标签在该列表中的下标」，
    #     若中途重排/过滤列表，既有下标会失效，回溯将指向错误标签。
    #     因此这里只追加、只用 alive 标记失效标签，绝不重排。
    dp: dict[tuple[int, int], list] = {}

    def add_label(mask, last, cost, t, parent_key, parent_index):
        lst = dp.setdefault((mask, last), [])
        for lab in lst:
            if lab[4] and lab[0] <= cost + 1e-9 and lab[1] <= t + 1e-9:
                return False                      # 被支配，丢弃新标签
        for k, lab in enumerate(lst):
            if lab[4] and cost <= lab[0] + 1e-9 and t <= lab[1] + 1e-9:
                lst[k] = (lab[0], lab[1], lab[2], lab[3], False)  # 标记失效
        lst.append((cost, t, parent_key, parent_index, True))
        return True

    # 初始化：仓 → i（只计里程与迟到，固定成本最后统一加）
    for k, i in enumerate(nodes):
        arrive = T[n0][i]
        op, cl, svc = inst.twt[i]
        start = max(arrive, op)
        late = max(0.0, start - cl)
        c = vtype.per_km * D[n0][i] + LATE_W * late
        add_label(1 << k, k, c, start + svc, None, -1)

    full = (1 << n) - 1
    # 按 mask 递增推进（父状态必先算完）
    for mask in range(1, full + 1):
        for last in range(n):
            if not (mask >> last) & 1:
                continue
            for pidx, lab in enumerate(dp.get((mask, last), [])):
                if not lab[4]:
                    continue
                c, t = lab[0], lab[1]
                g_last = nodes[last]          # ★ 全局节点索引
                for nxt in range(n):
                    if (mask >> nxt) & 1:
                        continue
                    g_nxt = nodes[nxt]        # ★ 必须转成全局索引再查矩阵
                    # ★★ 历史 bug：此处曾直接使用局部索引 nxt 查询
                    #    inst.dist / inst.time / inst.twt，导致距离与时间窗
                    #    取错（恒取第 0 号门店），DP 成本累加停滞在初始值，
                    #    出现「自报 236.60、实际 281.39」的不一致。
                    arrive = t + T[g_last][g_nxt]
                    op, cl, svc = inst.twt[g_nxt]
                    start = max(arrive, op)
                    late = max(0.0, start - cl)
                    nc = c + vtype.per_km * D[g_last][g_nxt] + LATE_W * late
                    add_label(mask | (1 << nxt), nxt, nc, start + svc,
                              (mask, last), pidx)

    # 收口：选成本最低的 (full, last)，再加回程与固定成本
    best_cost, best_key, best_index = INF, None, None
    for last in range(n):
        for pidx, lab in enumerate(dp.get((full, last), [])):
            if not lab[4]:
                continue
            total = lab[0] + vtype.per_km * D[nodes[last]][n0]
            if total < best_cost:
                best_cost, best_key, best_index = total, (full, last), pidx
    if best_key is None:
        return None, None

    seq = _backtrack_labels(dp, best_key, best_index, nodes)
    return vtype.fixed_cost + best_cost, seq


def _backtrack_labels(dp, key, index, nodes):
    """
    沿「标签下标链」回溯，得到与最优标签严格同源的节点顺序。

    ★ 修正要点（两次踩坑记录）：
      早期版本在每一级用 min(父标签) 重选，导致回溯序列与最优标签不同源
      （DP 报 236.60，实际序列 281.39）。现直接存储父标签的 (key, index)，
      并要求 dp 列表 append-only、索引稳定，逐级精确定位。
    """
    order = []
    cur_key, cur_idx = key, index
    guard = 0
    while cur_key is not None:
        guard += 1
        if guard > 1000:
            break
        mask, last = cur_key
        lst = dp.get(cur_key)
        if not lst or not (0 <= cur_idx < len(lst)):
            break
        order.append(last)
        _c, _t, pk, pi, _alive = lst[cur_idx]
        cur_key, cur_idx = pk, pi
    order.reverse()
    return [nodes[i] for i in order]


# ---------------------------------------------------------------- 集合划分 MILP

def solve_spp_exact(inst, vtype, routes, time_limit=60.0):
    """
    集合划分问题（Set Partitioning）：
        min  Σ_r c_r · x_r
        s.t. Σ_{r ∋ i} x_r = 1   ∀ 门店 i
             x_r ∈ {0,1}
    这是 VRPTW 的标准精确建模方式（列 = 路线）。
    """
    stores = inst.active_stores()
    idx = {s: k for k, s in enumerate(stores)}
    m, R = len(stores), len(routes)
    if R == 0:
        return None

    c = np.array([r[1] for r in routes], dtype=float)
    # 约束矩阵：每列一条路线，行 = 门店覆盖
    A = lil_matrix((m, R), dtype=float)
    for j, (cov, _c, _s) in enumerate(routes):
        for s in cov:
            if s in idx:
                A[idx[s], j] = 1.0
    A = A.tocsc()

    constraints = LinearConstraint(A, lb=np.ones(m), ub=np.ones(m))
    integrality = np.ones(R)
    bounds = Bounds(lb=0, ub=1)
    options = {"time_limit": time_limit, "mip_rel_gap": 0.0,
               "disp": False, "presolve": True}
    res = milp(c=c, constraints=constraints, integrality=integrality,
               bounds=bounds, options=options)
    if res.status not in (0, 1):
        return {"status": "no_solution", "code": res.status,
                "message": res.message}
    chosen = [routes[j] for j in range(R) if res.x[j] > 0.5]
    return {
        "status": "optimal" if res.status == 0 else "feasible",
        "objective": float(res.fun),
        "n_routes": len(chosen),
        "routes": [r[2] for r in chosen],
        "runtime_s": round(float(getattr(res, "mip_node_count", 0)), 1),
        "raw": chosen,
    }


def run_verification(size: int = 9, seed: int = 20261011):
    """
    在小规模子实例上跑精确解，对比启发式，输出 gap。
    size = 参与精确求解的门店数（≤10 以保证枚举可行）
    """
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "route"))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    from vrptw import load_scenario, Instance, VRPTWSolver, run_vt_import

    root = Path(__file__).resolve().parents[2]
    net, orders, sku_lib = load_scenario(root)
    VTs = run_vt_import()

    inst = Instance(net, orders, sku_lib)
    inst.set_vehicle_types(VTs)
    all_stores = inst.active_stores()

    # 取距离仓最近的 size 家门店（地理紧凑，便于枚举）
    stores = sorted(all_stores, key=lambda i: inst.dist[0][i])[:size]
    sub = Instance(net, orders, sku_lib)
    sub.set_vehicle_types(VTs)
    sub.active_stores = lambda: stores
    for i in list(sub.demand_w):
        if i not in stores:
            sub.demand_w[i] = sub.demand_v[i] = 0.0
            sub.demand_items[i] = 0

    print(f"精确验证子实例：{size} 家门店（距仓最近）")
    print(f"  总货重 {sum(sub.demand_w[i] for i in stores):.1f} kg"
          f" | 总体积 {sum(sub.demand_v[i] for i in stores):.1f} L")

    results = {}
    for vt in VTs:
        t0 = time.time()
        routes = enumerate_feasible_routes(sub, vt)
        t_enum = time.time() - t0
        if not routes:
            print(f"  [{vt.id}] 无可行路线（该车型装不下任何门店组合）")
            continue
        t0 = time.time()
        exact = solve_spp_exact(sub, vt, routes, time_limit=45.0)
        t_milp = time.time() - t0
        # 启发式（同车型单车队）
        sub_inst = Instance(net, orders, sku_lib)
        sub_inst.set_vehicle_types([vt])
        sub_inst.active_stores = lambda: stores
        for i in list(sub_inst.demand_w):
            if i not in stores:
                sub_inst.demand_w[i] = sub_inst.demand_v[i] = 0.0
                sub_inst.demand_items[i] = 0
        t0 = time.time()
        heur = VRPTWSolver(sub_inst, seed=seed).solve(stores, time_budget=8.0)
        t_heur = time.time() - t0
        print(f"  [{vt.id} {vt.name}] 枚举路线 {len(routes)} 条 "
              f"({t_enum:.1f}s) | MILP {t_milp:.1f}s | 启发式 {t_heur:.1f}s")
        if exact and exact.get("objective"):
            eo, ho = exact["objective"], heur.cost
            gap = (ho - eo) / eo * 100 if eo > 0 else 0.0
            print(f"       精确解 {eo:.2f} 元 ({exact['n_routes']} 车) | "
                  f"启发式 {ho:.2f} 元 ({heur.vehicles_used} 车) | "
                  f"gap {gap:+.2f}%")
            results[vt.id] = {"exact": round(eo, 2),
                              "heuristic": round(ho, 2),
                              "gap_pct": round(gap, 3),
                              "exact_vehicles": exact["n_routes"],
                              "heur_vehicles": heur.vehicles_used,
                              "routes_enumerated": len(routes)}
        else:
            print(f"       MILP 未得解：{exact}")
    return results


if __name__ == "__main__":
    out = run_verification(size=9)
    root = Path(__file__).resolve().parents[2]
    (root / "out" / "milp_verify.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("\n已写出 out/milp_verify.json")
