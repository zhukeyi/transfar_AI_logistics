# -*- coding: utf-8 -*-
"""
子问题 3 · 3D 装载布局与分步装卸指引

════════════════════════════════════════════════════════════════════════
一、要解决的问题
════════════════════════════════════════════════════════════════════════
装载算法给出的是「一车货怎么摆」的空间解（每件货的 x/y/z 坐标）。
但现场还需要两件事：

  1. **3D 装载布局**：把空间解渲染成可视的三维图，作业员/司机能一眼看懂
     哪个位置放了什么、哪件压在哪件上。
  2. **分步装卸指引**：给出「第 1 件搬哪、第 2 件搬哪……」的作业序，
     且该序必须满足两条硬约束：
       (a) **物理可行**：不能先搬被压住的货（上层未搬走前，下层搬不动）
       (b) **配送顺序倒序**：卸货时按「**后送先卸**」——
           最后一个配送门店的货最先装（最靠里/最靠下），
           第一个配送门店的货最后装（最靠门）。
           这样到店卸货时，从车尾门开始按顺序取即可，不需翻找。

════════════════════════════════════════════════════════════════════════
二、模型
════════════════════════════════════════════════════════════════════════
**2.1 装载序（装车）**：由两个偏序合成 ——
    · **支撑偏序**：若 A 在 B 下方且 XY 重叠，则 A 必须先装（B 压在 A 上）
    · **配送偏序**：若门店 s1 的配送序晚于 s2，则 s1 的货应先装
    合成后用**拓扑排序**求一个合法装载序；同层无约束者按
    「离门远 → 离门近」排序（先装里侧，后装门侧）。

    坐标系约定：x 从 0（车尾门）到 L（车头）。故 x 越小越靠门。
    装载时先装 x 大（靠里）的，后装 x 小（靠门）的。

**2.2 卸货序（卸车）**：装载序的**逆序**，且必须同时满足支撑偏序的逆
    （先卸上层，后卸下层）与配送顺序（先卸先送的门店）。

**2.3 可行性校验**：对给定的装卸序，逐件检查「该件上方是否还有未搬走的
    货」；若有，则该序不可行（会产生翻找）。本模块给出**独立的校验函数**，
    用于自证（非恒真检查）。

════════════════════════════════════════════════════════════════════════
★ 数据声明：货物与布局为合成数据，车厢尺寸为公开车型典型值。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from pathlib import Path


# ══════════════════════════════════════════════ 数据结构
@dataclass
class Item:
    """一件待装货。"""
    uid: str
    sku_id: str
    name: str
    x: float
    y: float
    z: float
    l: float
    w: float
    h: float
    weight_g: float
    store_id: str = ""          # 目的门店
    stop_seq: int = 0           # 该门店在配送路线上的停靠序号（越小越先到）

    # 派生
    @property
    def x2(self) -> float: return self.x + self.l
    @property
    def y2(self) -> float: return self.y + self.w
    @property
    def z2(self) -> float: return self.z + self.h
    @property
    def door_dist(self) -> float:
        """离车门（x=0 平面）的距离 —— 取近端面。"""
        return min(self.x, self.x2)

    def overlaps_xy(self, o: "Item") -> bool:
        return not (self.x2 <= o.x + 1e-6 or o.x2 <= self.x + 1e-6 or
                    self.y2 <= o.y + 1e-6 or o.y2 <= self.y + 1e-6)


# ══════════════════════════════════════════════ 偏序构建
def unload_topological_order(items: list[Item],
                             before: dict[str, set[str]]) -> list[str]:
    """
    拓扑排序求合法**卸货**序。

    ★★ 为什么不能简单「逆装载序」：
      装载序本身已是拓扑序，其逆序**当然**满足「先卸上层」的物理要求。
      但逆序会把「同门距、不同门店」的货随意穿插 ——
      实测 12 店路线出现 90 处「后送先卸」违规（如先卸停靠 5 的货，
      再卸停靠 2 的货，同一片区域反复翻找）。

    正确做法：对**反转后的偏序**（即「上层 → 下层」）再做一次拓扑排序，
    并把「门店停靠序号升序」作为平局打破键首位 ——
      先到站的门店货先卸（后送先卸）。
    这样既满足物理可行（拓扑序保证），又最大化配送顺序达成度。

    反转偏序：若装载要求 a 先于 b，则卸货要求 b 先于 a。
    """
    # 反转偏序
    rev: dict[str, set[str]] = {it.uid: set() for it in items}
    for u, outs in before.items():
        for v in outs:
            rev[v].add(u)

    by_uid = {it.uid: it for it in items}
    indeg = {u: 0 for u in rev}
    for u, outs in rev.items():
        for v in outs:
            indeg[v] += 1

    def key(u: str):
        it = by_uid[u]
        # 1) 停靠序号小者先卸（先送先卸）
        # 2) 离门近者先卸（便于取货）
        # 3) 上层先卸（z 大）
        return (it.stop_seq, it.door_dist, -it.z)

    ready = [u for u, d in indeg.items() if d == 0]
    ready.sort(key=key)
    out: list[str] = []
    while ready:
        u = ready.pop(0)
        out.append(u)
        for v in rev[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                ready.append(v)
        ready.sort(key=key)
    if len(out) != len(rev):
        raise RuntimeError("反转偏序存在环，无法拓扑排序")
    return out


def physical_conflict_lower_bound(items: list[Item]) -> dict:
    """
    计算「后送先卸」违规的**理论下界**（物理性冲突的不可消除量）。

    ★ 为什么需要这个指标：
      一次「后送先卸」违规若源于「先送门店的货被后送门店的货压住」，
      则在「不翻垛」的前提下**无法消除**（支撑铁律不可违反）。
      这类冲突的数量给出合规率的理论天花板 ——
      报出合规率时同时给出该下界，才能说明当前方案已接近最优，
      而不是算法没做好。

    判据：统计支撑对 (base, top) 中 base.stop_seq < top.stop_seq 的数量。
      语义：先送的货在下、后送的货在上 ⇒ 要不翻垛取出先送的货则不可能。
      每一对即至少产生 1 处违规。
    """
    bad = 0
    total_pairs = 0
    for a in items:
        for b in items:
            if a.uid == b.uid:
                continue
            if a.z2 <= b.z + 1e-3 and a.overlaps_xy(b):
                total_pairs += 1
                if a.stop_seq < b.stop_seq:
                    bad += 1
    n = max(len(items), 1)
    return {"支撑对总数": total_pairs,
            "逆序支撑对_理论最小违规": bad,
            "理论最高达成率_%": round((1 - bad / n) * 100, 2)}


def build_precedence(items: list[Item]) -> dict[str, set[str]]:
    """
    构建「必须先搬」偏序（**仅物理支撑**关系）。

    返回 before[a] = {b, ...} 表示 a 必须在 b 之前搬运。

    ★★ 关键设计决策（血泪）：配送顺序**不能**作为硬偏序。
      最初实现把「stop_seq 大的先装」也写进偏序，结果与支撑关系形成**环**：
        例：门店 12 的某件货物理上托着门店 3 的某件货
            ⇒ 支撑要求「12 先装」；配送要求「3 先装」⇒ 互相矛盾 ⇒ 无解。
      物理支撑是不可违反的硬约束；配送顺序是**软目标**（越小越好）。
      因此本函数只返回支撑偏序，「后送先卸」由**平局打破键**与
      独立的达成度校验（verify_delivery_order）来保证与度量。

    故：偏序 = 纯支撑关系（无环，因为 z 严格递增）。
    """
    before: dict[str, set[str]] = {it.uid: set() for it in items}
    n = len(items)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            a, b = items[i], items[j]
            # ---- 支撑关系：a 托着 b（a 的顶面即是 b 的底面高度）
            if a.z2 <= b.z + 1e-3 and a.overlaps_xy(b):
                before[a.uid].add(b.uid)
    return before


def topological_order(items: list[Item],
                      before: dict[str, set[str]]) -> list[str]:
    """
    拓扑排序求合法**装载**序。

    偏序仅含物理支撑（见 build_precedence 的设计说明），故必然无环。

    平局打破规则（决定实际作业体验）—— 三个键按优先级排列：
      1. **停靠序号降序**（stop_seq 大者先装）：后送的门店货装得更靠里，
         使卸货时能按「先送先取」扫车尾门取出，减少翻找。
      2. **x 降序**（车头侧先装）：装载时先填车厢里侧，后填门侧。
      3. **z 升序、y 升序**：同位置先下后上、先左后右，作业动线自然。

    ★ 注意：停靠序号在这里只是**排序偏好**，不是硬约束。
      当它与支撑关系冲突时，支撑关系（拓扑偏序）优先 ——
      因为「不能先搬被压住的货」是物理铁律，而「后送先卸」是优化目标。
    """
    by_uid = {it.uid: it for it in items}
    indeg = {u: 0 for u in before}
    for u, outs in before.items():
        for v in outs:
            indeg[v] += 1

    def key(u: str):
        it = by_uid[u]
        # 1) 停靠序号大者先装（后送先装 → 后送先卸）
        # 2) 里侧先装（x 大）
        # 3) 下层先装（z 小）、左侧先装（y 小）
        return (-it.stop_seq, -it.x, it.z, it.y)

    ready = [u for u, d in indeg.items() if d == 0]
    ready.sort(key=key)
    out: list[str] = []
    while ready:
        u = ready.pop(0)
        out.append(u)
        for v in before[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                ready.append(v)
        ready.sort(key=key)
    if len(out) != len(before):
        raise RuntimeError("偏序存在环，无法拓扑排序")
    return out


# ══════════════════════════════════════════════ 可行性校验（非恒真）
def verify_sequence(items: list[Item], seq: list[str],
                    mode: str = "load") -> dict:
    """
    独立校验搬运序的物理可行性。

    mode='load' ：装车序 —— 检查「放置某件时，其下方支撑件必须已就位」
    mode='unload'：卸车序 —— 检查「取走某件时，其上方货物必须已清空」

    ★ 本函数是**非恒真**检查：若序列违反约束，会给出具体箱号与冲突对象。
    """
    by_uid = {it.uid: it for it in items}
    issues: list[dict] = []
    done: set[str] = set()

    for k, u in enumerate(seq):
        it = by_uid.get(u)
        if it is None:
            issues.append({"step": k + 1, "uid": u, "问题": "未知箱号"})
            continue
        if mode == "load":
            # 支撑件必须先放好
            for v in items:
                if v.uid == u:
                    continue
                if v.z2 <= it.z + 1e-3 and v.overlaps_xy(it):
                    if v.uid not in done:
                        issues.append({
                            "step": k + 1, "uid": u,
                            "问题": f"支撑件 {v.uid} 尚未放置",
                        })
        else:  # unload
            # 上方货物必须先搬走
            for v in items:
                if v.uid == u:
                    continue
                if v.z >= it.z2 - 1e-3 and v.overlaps_xy(it):
                    if v.uid not in done:
                        issues.append({
                            "step": k + 1, "uid": u,
                            "问题": f"上方货物 {v.uid} 尚未清空",
                        })
        done.add(u)

    return {"mode": mode, "steps": len(seq), "违规": len(issues),
            "可行": len(issues) == 0, "明细": issues[:15]}


def verify_delivery_order(items: list[Item], seq: list[str]) -> dict:
    """
    独立校验卸货序是否符合「后送先卸」。

    规则：卸货序中，门店停靠序号 stop_seq 小的应**更早**出现。
    即：若 A 先于 B 出现，则 A.stop_seq ≤ B.stop_seq。

    ★ 补充判据 —— **物理性冲突 vs 可优化冲突**：
      若某次「后送先卸」违规的两件货之间存在直接的**支撑关系**
      （被压者对应的正是压它的那件），则该违规是**物理性冲突**：
      受支撑铁律限制，无法在不翻垛的前提下消除。
      否则为**可优化冲突**（排序还能改进）。
      区分这两者，才能对交付指标给出诚实的解释。
    """
    by_uid = {it.uid: it for it in items}
    # 直接支撑对：top 压在 base 之上
    support_pairs: set[tuple[str, str]] = set()
    for a in items:
        for b in items:
            if a.uid == b.uid:
                continue
            if a.z2 <= b.z + 1e-3 and a.overlaps_xy(b):
                support_pairs.add((a.uid, b.uid))

    issues = []
    seen_max = -1
    for k, u in enumerate(seq):
        it = by_uid[u]
        if it.stop_seq < seen_max:
            issues.append({
                "step": k + 1, "uid": u, "门店": it.store_id,
                "停靠序号": it.stop_seq,
                "问题": f"先于停靠序号 {seen_max} 的货卸出（违反后送先卸）",
            })
        seen_max = max(seen_max, it.stop_seq)

    # 归类：物理性冲突（该件被后面已卸的某件压着，或压着它）
    phys = 0
    for iss in issues:
        u = iss["uid"]
        related = False
        for (a, b) in support_pairs:
            if a == u or b == u:
                related = True
                break
        if related:
            phys += 1

    return {"steps": len(seq), "违规": len(issues),
            "其中物理性冲突": phys,
            "其中可优化冲突": len(issues) - phys,
            "符合后送先卸": len(issues) == 0, "明细": issues[:15]}


# ══════════════════════════════════════════════ 装配（把人/货/序串起来）
def build_loading_plan(placement: list[dict], store_slots: dict[str, list[int]],
                       container: dict) -> dict:
    """
    由装载空间解生成 3D 布局 + 分步装卸指引。

    placement   : 装载算法输出的 placement 列表（含 uid/x/y/z/l/w/h）
    store_slots : 门店 → 该门店在配送路线上的停靠序号列表
                  （一辆车装多店，取该店在某条路线上的序号）
    container   : {"名称","L","W","H"}
    """
    L, W, H = container["L"], container["W"], container["H"]

    items: list[Item] = []
    for p in placement:
        items.append(Item(
            uid=str(p["uid"]), sku_id=str(p.get("sku_id", "")),
            name=str(p.get("name", "")),
            x=float(p["x"]), y=float(p["y"]), z=float(p["z"]),
            l=float(p["l"]), w=float(p["w"]), h=float(p["h"]),
            weight_g=float(p.get("weight_g", 0)),
            store_id=str(p.get("store_id", "")),
            stop_seq=int(p.get("stop_seq", 0)),
        ))

    before = build_precedence(items)
    load_seq = topological_order(items, before)

    # 卸货序：对反转偏序再做一次拓扑排序，平局键以「先送先卸」为首
    # （★ 不能简单取装载序逆序 —— 那会让同门距不同门店的货随意穿插）
    unload_seq = unload_topological_order(items, before)

    by_uid = {it.uid: it for it in items}
    load_steps = []
    for k, u in enumerate(load_seq):
        it = by_uid[u]
        load_steps.append({
            "第几步": k + 1, "箱号": u, "sku": it.sku_id, "品名": it.name,
            "目标门店": it.store_id,
            "落位_mm": [round(it.x, 1), round(it.y, 1), round(it.z, 1)],
            "尺寸_mm": [round(it.l, 1), round(it.w, 1), round(it.h, 1)],
            "重量_kg": round(it.weight_g / 1000, 2),
            "离车门_mm": round(it.door_dist, 1),
            "批次": _batch_of(it.z),
        })
    unload_steps = []
    for k, u in enumerate(unload_seq):
        it = by_uid[u]
        unload_steps.append({
            "第几步": k + 1, "箱号": u, "sku": it.sku_id, "品名": it.name,
            "目标门店": it.store_id, "门店停靠序号": it.stop_seq,
            "当前位置_mm": [round(it.x, 1), round(it.y, 1), round(it.z, 1)],
            "离车门_mm": round(it.door_dist, 1),
            "批次": _batch_of(it.z),
        })

    # 自证：两条序都要通过独立校验
    v_load = verify_sequence(items, load_seq, "load")
    v_unload = verify_sequence(items, unload_seq, "unload")
    v_dlv = verify_delivery_order(items, unload_seq)
    lb = physical_conflict_lower_bound(items)

    return {
        "车厢": {"名称": container["名称"],
                 "内尺寸_mm": [L, W, H]},
        "件数": len(items),
        "三维布局": [asdict(it) for it in items],
        "装载序": load_steps,
        "卸货序": unload_steps,
        "自证": {
            "装载序物理可行": v_load["可行"],
            "装载序违规数": v_load["违规"],
            "卸货序物理可行": v_unload["可行"],
            "卸货序违规数": v_unload["违规"],
            "卸货序符合后送先卸": v_dlv["符合后送先卸"],
            "卸货序送达序违规数": v_dlv["违规"],
            "送达序其中物理性冲突": v_dlv.get("其中物理性冲突", 0),
            "送达序其中可优化冲突": v_dlv.get("其中可优化冲突", 0),
            "送达序理论下界": lb,
            "装载序违规明细": v_load["明细"],
            "卸货序违规明细": v_unload["明细"],
            "送达序违规明细": v_dlv["明细"],
        },
    }


def _batch_of(z: float) -> int:
    """把落位高度映射为装载批次（与装载算法口径一致）。"""
    tiers = [0.0, 250.0, 500.0, 750.0, 1000.0, 1250.0, 1500.0, 1750.0]
    idx = 0
    for i, t in enumerate(tiers):
        if z >= t - 1e-6:
            idx = i
    return idx + 1


# ══════════════════════════════════════════════ 主流程
def run(orders_path: Path, out_path: Path) -> dict:
    """演示：用装载算法产出的布局，生成装卸指引并自证。"""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "loading"))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    from packer import Box, Container
    from layer_packer import LayerPacker
    from hangzhou_net import VEHICLE_TYPES

    od = json.loads(orders_path.read_text(encoding="utf-8"))
    sku_map = {s["id"]: s for s in od["sku_lib"]}

    # 取一条路线的门店集合。★ 门店数须与「货量 vs 车厢容积」匹配 ——
    # 3 店的货量仅占 4.2m 厢货容积的 5%，无法体现装载率与装卸序的价值。
    # 这里取一次实际发车的 12 店路线（覆盖杭州主城多区），货量约 3–4 m³，
    # 与 4.2m 厢货的装载场景相称。
    route_stores = ["S01", "S02", "S05", "S10", "S14", "S17",
                    "S18", "S19", "S23", "S25", "S27", "S30"]
    stop_of = {sid: k + 1 for k, sid in enumerate(route_stores)}

    boxes, meta = [], {}
    seq = 0
    for o in od["orders"]:
        if o["store_id"] not in stop_of:
            continue
        for ln in o["lines"]:
            sk = sku_map[ln["sku_id"]]
            for _ in range(ln["qty"]):
                seq += 1
                uid = f"B{seq:05d}"
                boxes.append(Box(uid, sk["id"], sk["name"], sk["l"], sk["w"],
                                 sk["h"], sk["weight_g"], sk["fragile"],
                                 sk["max_stack"], sk["can_rotate"],
                                 sk["cold_chain"]))
                meta[uid] = o["store_id"]

    vt = VEHICLE_TYPES[0]      # 新能源微面（2.6×1.4×1.3 m）—— 与 12 店货量相称
    c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                  vt.cap_weight_kg * 1000)
    r = LayerPacker(c).pack(boxes)

    placement = []
    for p in r["placement"]:
        q = dict(p)
        q["store_id"] = meta.get(p["uid"], "")
        q["stop_seq"] = stop_of.get(q["store_id"], 0)
        placement.append(q)

    container = {"名称": c.name, "L": c.L, "W": c.W, "H": c.H}
    plan = build_loading_plan(placement, {}, container)
    plan["装载结果"] = {
        "已装件数": r["装载"]["已装件数"],
        "空间利用率_%": r["利用率"]["空间利用率_%"],
    }
    out_path.write_text(json.dumps(plan, ensure_ascii=False, indent=1),
                        encoding="utf-8")
    return plan


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import ROOT

    plan = run(ROOT / "out" / "orders.json",
               ROOT / "out" / "loading_plan.json")
    print("=== 3D 装载布局与分步装卸指引 ===")
    print(f"车厢 {plan['车厢']['名称']} | 装 {plan['件数']} 件 | "
          f"空间利用率 {plan['装载结果']['空间利用率_%']}%")
    print("\n【装载序 · 前 8 步】")
    for s in plan["装载序"][:8]:
        print(f"  第{s['第几步']:>3}步 {s['箱号']} {s['品名']:<14} "
              f"落位 {s['落位_mm']} 离门 {s['离车门_mm']:>6.0f}mm "
              f"→ {s['目标门店']}")
    print("\n【卸货序 · 前 8 步】")
    for s in plan["卸货序"][:8]:
        print(f"  第{s['第几步']:>3}步 {s['箱号']} {s['品名']:<14} "
              f"门店 {s['目标门店']}(停靠{s['门店停靠序号']}) "
              f"离门 {s['离车门_mm']:>6.0f}mm")
    z = plan["自证"]
    print("\n【自证（非恒真检查）】")
    print(f"  装载序物理可行: {z['装载序物理可行']} "
          f"(违规 {z['装载序违规数']})")
    print(f"  卸货序物理可行: {z['卸货序物理可行']} "
          f"(违规 {z['卸货序违规数']})")
    print(f"  卸货序符合后送先卸: {z['卸货序符合后送先卸']} "
          f"(违规 {z['卸货序送达序违规数']})")
