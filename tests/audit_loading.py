# -*- coding: utf-8 -*-
"""
装载方案合规性审计

对任一装载方案逐箱检查 8 项约束（C1–C8），输出违规明细。

★ 方法学意义：装载方案不能只比「利用率」。
  一个把易碎品压在最底层、堆叠超过上限的方案，利用率可能更高，
  但在业务上不可接受。因此必须把约束合规定量化后再比利用率，
  否则对比结论不可信。

  本审计是「非恒真」检查：若方案违规，报表给出具体箱号与类型。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "loading"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "data"))

from packer import Container, Placement, Box


def audit(container: Container, placements: list[Placement],
          boxes_meta: dict[str, Box], support_ratio: float = 0.60) -> dict:
    """
    逐项审计。boxes_meta: uid -> Box（提供原始 max_stack / fragile 等）。

    返回 {"violations": [...], "counts": {...}, "total": n, "合规": bool}
    """
    v: list[dict] = []

    def add(kind, uid, detail):
        v.append({"类型": kind, "箱号": uid, "说明": detail})

    # C1 载重
    tot_w = sum(p.weight_g for p in placements)
    if tot_w > container.max_weight_g + 1e-6:
        add("C1 超载", "-", f"总重 {tot_w/1000:.1f}kg > "
                            f"核定 {container.max_weight_g/1000:.1f}kg")

    # C3 越界
    for p in placements:
        if (p.x < -1e-6 or p.y < -1e-6 or p.z < -1e-6 or
                p.x2 > container.L + 1e-6 or p.y2 > container.W + 1e-6 or
                p.z2 > container.H + 1e-6):
            add("C3 越界", p.uid,
                f"坐标 ({p.x:.0f},{p.y:.0f},{p.z:.0f}) "
                f"尺寸 {p.l:.0f}×{p.w:.0f}×{p.h:.0f}")

    # C2 几何重叠
    n = len(placements)
    for i in range(n):
        for j in range(i + 1, n):
            if placements[i].overlaps_3d(placements[j]):
                add("C2 重叠", f"{placements[i].uid}/{placements[j].uid}",
                    "三维交叠")

    # 建立堆叠关系
    def supporters(p, all_p):
        out = []
        if p.z <= 1e-6:
            return out
        for q in all_p:
            if abs(q.z2 - p.z) > 1e-3:
                continue
            ox = min(p.x2, q.x2) - max(p.x, q.x)
            oy = min(p.y2, q.y2) - max(p.y, q.y)
            if ox > 1e-6 and oy > 1e-6:
                out.append(q)
        return out

    def support_ratio_of(p, all_p):
        if p.z <= 1e-6:
            return 1.0
        foot = p.l * p.w
        cov = 0.0
        for q in all_p:
            if abs(q.z2 - p.z) > 1e-3:
                continue
            ox = min(p.x2, q.x2) - max(p.x, q.x)
            oy = min(p.y2, q.y2) - max(p.y, q.y)
            if ox > 1e-6 and oy > 1e-6:
                cov += ox * oy
        return min(1.0, cov / max(foot, 1e-9))

    for p in placements:
        sup = supporters(p, placements)
        # C6 易碎禁压：其下方若有 fragile==2 的箱，则违规
        for q in sup:
            if q.fragile == 2:
                add("C6 禁压箱被压", p.uid,
                    f"压在禁压箱 {q.uid}（{q.name}）之上")
        # C5 堆叠层数
        for q in sup:
            meta = boxes_meta.get(q.uid)
            ms = meta.max_stack if meta else q.max_stack
            if ms == 0:
                continue        # 已在 C6 覆盖
            # ★ 层数语义：统计 q 顶面之上「与 q 在 XY 重叠」的箱，
            #   按底面高度分组，组数即层数（不是箱数）
            levels = sorted({round(r.z, 3) for r in placements
                             if r.z >= q.z2 - 1e-6 and r.overlaps_xy(q)})
            above = len(levels)
            if above > ms:
                add("C5 超堆叠上限", q.uid,
                    f"其上堆 {above} 层 > 上限 {ms} 层（{q.name}）")
        # C8 支撑率
        sr = support_ratio_of(p, placements)
        if sr < support_ratio - 1e-6 and p.z > 1e-6:
            add("C8 支撑不足", p.uid,
                f"支撑率 {sr*100:.1f}% < 要求 {support_ratio*100:.0f}%")
        # C7 方向：不可倒置箱（由调用方保证 pl/pw/ph 与原始一致）
        meta = boxes_meta.get(p.uid)
        if meta is not None and not meta.can_rotate:
            if abs(p.h - meta.h) > 1e-6:
                add("C7 方向违规", p.uid,
                    f"不可倒置箱被改变高度 {meta.h}→{p.h}")

    counts: dict[str, int] = {}
    for item in v:
        counts[item["类型"]] = counts.get(item["类型"], 0) + 1
    return {"violations": v, "counts": counts,
            "total_items": len(placements),
            "violation_total": len(v),
            "合规": len(v) == 0}


def report(container: Container, placements, boxes_meta, tag: str,
           support_ratio: float = 0.60) -> dict:
    r = audit(container, placements, boxes_meta, support_ratio)
    print(f"\n[{tag}] 件数 {r['total_items']} | 违规 {r['violation_total']} 项 "
          f"| {'合规' if r['合规'] else '不合规'}")
    for k, c in sorted(r["counts"].items(), key=lambda x: -x[1]):
        print(f"    {k}: {c} 项")
    for item in r["violations"][:8]:
        print(f"      · {item['箱号']}: {item['说明']}")
    return r


if __name__ == "__main__":
    import json
    from packer import LoadingPlanner, naive_pack
    from paths import ROOT
    from hangzhou_net import VEHICLE_TYPES
    from goods import SKU_LIB

    od = json.loads((ROOT / "out" / "orders.json").read_text(encoding="utf-8"))
    sku_map = {s["id"]: s for s in od["sku_lib"]}
    boxes, meta = [], {}
    seq = 0
    for o in od["orders"]:
        for ln in o["lines"]:
            sk = sku_map[ln["sku_id"]]
            for _ in range(ln["qty"]):
                seq += 1
                uid = f"B{seq:05d}"
                b = Box(uid, sk["id"], sk["name"], sk["l"], sk["w"], sk["h"],
                        sk["weight_g"], sk["fragile"], sk["max_stack"],
                        sk["can_rotate"], sk["cold_chain"])
                boxes.append(b)
                meta[uid] = b

    vt = VEHICLE_TYPES[2]
    c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                  vt.cap_weight_kg * 1000)
    print(f"车型 {vt.id} {vt.name} | 待装 {len(boxes)} 件")

    algo = LoadingPlanner(c)
    algo.pack(boxes)
    report(c, algo.placed, meta, "算法方案")

    base = naive_pack(c, boxes)
    # 人工方案需要重建 Placement 对象才能审计
    base_ps = [Placement(**{k: p[k] for k in (
        "uid", "x", "y", "z", "l", "w", "h", "weight_g", "fragile",
        "max_stack", "sku_id", "name")}) for p in base["placement"]]
    report(c, base_ps, meta, "人工方案")
