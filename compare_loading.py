# -*- coding: utf-8 -*-
"""
三维装载 · 算法 vs 人工基线**同口径对比**

为什么单独出一个对比脚本：
  如果只报「算法 73.56%」，无法说明它好在哪 —— 因为人工方案数字看着更高。
  必须把两者放在**同一个审计器**下过一遍，才能看出差别在**合规性**上。

★ 对比纪律：
  · 两个方案都过同一个 `tests/audit_loading.py`（C1–C8）；
  · 都统计**已装件数 / 空间利用率 / 违规总数与分项**；
  · 人工基线**故意不实现**易碎保护与堆叠上限检查（模拟现场"凭经验"），
    这不是给算法作弊，而是把「人工经验的实际代价」量化出来。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE / "src"))
sys.path.insert(0, str(_HERE / "src" / "loading"))
sys.path.insert(0, str(_HERE / "src" / "data"))
sys.path.insert(0, str(_HERE / "tests"))

from paths import ROOT                                     # noqa: E402
from hangzhou_net import VEHICLE_TYPES                     # noqa: E402
from loading.packer import Container, Box, Placement, naive_pack   # noqa: E402
from loading.layer_packer import LayerPacker               # noqa: E402
import audit_loading as AL                                 # noqa: E402


def build_boxes() -> list[Box]:
    od = json.loads((ROOT / "out" / "orders.json").read_text(encoding="utf-8"))
    sku_map = {s["id"]: s for s in od["sku_lib"]}
    boxes, seq = [], 0
    for o in od["orders"]:
        for ln in o["lines"]:
            sk = sku_map[ln["sku_id"]]
            for _ in range(ln["qty"]):
                seq += 1
                boxes.append(Box(f"B{seq:05d}", sk["id"], sk["name"],
                                 sk["l"], sk["w"], sk["h"], sk["weight_g"],
                                 sk["fragile"], sk["max_stack"],
                                 sk["can_rotate"], sk["cold_chain"]))
    return boxes


def to_placements(recs: list[dict], meta: dict[str, Box]) -> list[Placement]:
    out = []
    for d in recs:
        b = meta[d["uid"]]
        out.append(Placement(d["uid"], d["x"], d["y"], d["z"],
                             d["l"], d["w"], d["h"], d["weight_g"],
                             b.fragile, b.max_stack, b.sku_id, b.name))
    return out


def main() -> int:
    boxes = build_boxes()
    meta = {b.uid: b for b in boxes}
    print("═" * 74)
    print(f"三维装载对比 · 待装 {len(boxes)} 件 · 车型 3 种 · 同一审计器 (C1–C8)")
    print("═" * 74)

    rows = []
    for vt in VEHICLE_TYPES:
        c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                      vt.cap_weight_kg * 1000)

        # ── 算法
        ra = LayerPacker(c).pack(boxes)
        pa = to_placements(ra["placement"], meta)
        aa = AL.audit(c, pa, meta, support_ratio=0.6)

        # ── 人工基线
        rb = naive_pack(c, boxes)
        pb = to_placements(rb["placement"], meta)
        ab = AL.audit(c, pb, meta, support_ratio=0.6)

        rows.append({
            "车型": f"{vt.id} {vt.name}",
            "算法_件数": len(pa),
            "算法_利用率_%": ra["利用率"]["空间利用率_%"],
            "算法_违规": aa["violation_total"],
            "算法_违规分项": aa["counts"],
            "基线_件数": len(pb),
            "基线_利用率_%": rb["利用率"]["空间利用率_%"],
            "基线_违规": ab["violation_total"],
            "基线_违规分项": ab["counts"],
        })

        print(f"\n[{vt.id} {vt.name}]  内尺寸 "
              f"{vt.box_l}×{vt.box_w}×{vt.box_h} mm")
        print(f"  {'方案':<12}{'装载件数':>9}{'空间利用率':>11}{'约束违规':>10}"
              f"{'  分项'}")
        print(f"  {'算法·分层货架':<12}{len(pa):>9}{ra['利用率']['空间利用率_%']:>10.2f}%"
              f"{aa['violation_total']:>10}   {aa['counts'] or '—'}")
        print(f"  {'基线·人工经验':<12}{len(pb):>9}{rb['利用率']['空间利用率_%']:>10.2f}%"
              f"{ab['violation_total']:>10}   {ab['counts'] or '—'}")

    print("\n" + "═" * 74)
    tot_a = sum(r["算法_违规"] for r in rows)
    tot_b = sum(r["基线_违规"] for r in rows)
    print(f"合计违规：算法 {tot_a} 项 vs 人工基线 {tot_b} 项 "
          f"（降幅 {100*(tot_b-tot_a)/max(tot_b,1):.1f}%）")
    print("═" * 74)

    outp = ROOT / "out" / "loading_compare.json"
    outp.write_text(json.dumps({"逐车型": rows,
                                "合计_算法违规": tot_a,
                                "合计_基线违规": tot_b},
                               ensure_ascii=False, indent=1), encoding="utf-8")
    print("写出", outp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
