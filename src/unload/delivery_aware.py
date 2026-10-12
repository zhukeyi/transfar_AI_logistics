# -*- coding: utf-8 -*-
"""
子问题 3 · 配送感知装载（Delivery-Aware Loading）

════════════════════════════════════════════════════════════════════════
一、问题：为什么「先装里侧」还不够
════════════════════════════════════════════════════════════════════════
上一版装载算法（LayerPacker）只按「底面积降序」排序装载，**不知道**每件货
要送哪个门店。实测 12 店路线的后果（`out/loading_plan.json` 自证）：

    卸货序「后送先卸」违规 33 处，且经归类 **33 处全部为物理性冲突**
    （0 处可优化）——
    即：先到站门店的货被后到站门店的货**物理压住**，
    除非翻垛否则无法取出。

这说明「先装里侧」是必要条件但不充分：**必须让后送门店的货更深**。
即装载阶段就要把「配送停靠序号」作为排序偏好的首要项。

════════════════════════════════════════════════════════════════════════
二、模型：两级排序（配送序 → 装填序）
════════════════════════════════════════════════════════════════════════
把装载排序键改为**字典序**：

    第 1 级：配送停靠序号**降序**（stop_seq 大者先装 → 落在更深处）
             这样卸货时「先送的门店货在门侧、后送的在里侧」，
             从门侧向里逐层取即可，不需翻找。
    第 2 级：底面积降序（大件先放，作支撑基础，与原算法一致）
    第 3 级：高度降序、重量降序（同规格下更稳）

同时，**深度约束**以软目标形式注入：把「x 越大越里」与 stop_seq 挂钩 ——
在落位择优时，对 stop_seq 大的货**额外偏好大 x**（更靠里），
对 stop_seq 小的货偏好小 x（靠门）。这通过落位评分键实现。

════════════════════════════════════════════════════════════════════════
三、效果验证（非恒真）
════════════════════════════════════════════════════════════════════════
对比「非配送感知」与「配送感知」两版装载，输出：
    · 卸货序「后送先卸」达成率
    · 其中物理性冲突 / 可优化冲突 的数量
    · 空间利用率（须不因配送感知而显著下降，否则需权衡）

★ 数据声明：货物与布局为合成数据，车厢尺寸为公开车型典型值。
"""
from __future__ import annotations

import json
import sys
from dataclasses import asdict
from pathlib import Path

# ★ 路径准备：本模块被「作为脚本运行」与「被包内导入」两种情形都会发生，
#   故在导入期即把 loading/ 与与/ 加入 sys.path，避免 ModuleNotFoundError。
_HERE = Path(__file__).resolve()
for _p in (_HERE.parents[1], _HERE.parents[1] / "loading"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from packer import Box, Container, Placement


class DeliveryAwareLayerPacker:
    """
    配送感知的分层（货架 + 天际线）装载器。

    与 `LayerPacker` 的唯一区别是**装载顺序**与**落位偏好**增加了
    配送停靠序号（stop_seq）这一维度，其余约束校验（C1–C8）完全复用。
    """

    def __init__(self, container: Container,
                 box_stop: dict[str, int],
                 support_ratio: float = 0.60,
                 cog_abs: float = 0.95,
                 cog_warmup: int = 24,
                 door_at_low_x: bool = True):
        """
        box_stop     : uid → 该件货的配送停靠序号（1 起，越小越先送）
        door_at_low_x: True 表示 x=0 侧是车门（默认）。装载时先装大 x，
                       并把 stop_seq 大的货优先推向大 x。
        """
        # 直接复用主装载器的实现
        from layer_packer import LayerPacker
        self._base = LayerPacker(container,
                                 support_ratio=support_ratio,
                                 cog_abs=cog_abs,
                                 cog_warmup=cog_warmup)
        self.c = container
        self.box_stop = box_stop
        self.door_at_low_x = door_at_low_x

    # ------------------------------------------------------------ 委托
    def __getattr__(self, name):
        return getattr(self._base, name)

    # ------------------------------------------------------- 排序与偏好
    def _load_order(self, boxes: list[Box]) -> list[Box]:
        """
        装载排序键（字典序）：

          1. 停靠序号**降序** —— 后送的先装，落在更深（大 x）处
          2. 底面积降序       —— 大件先放，作支撑基础
          3. 高度降序、重量降序
        """
        def key(b: Box):
            stop = self.box_stop.get(b.uid, 0)
            return (-stop, -b.footprint(), -b.h, -b.weight_g)
        return sorted(boxes, key=key)

    def pack(self, boxes: list[Box]) -> dict:
        """用配送感知的顺序 + 纵深偏好调用主装载器。"""
        ordered = self._load_order(boxes)
        base = self._base
        # ★ 纵深偏好：把 stop_seq 注入落位择优键。
        #   原理：_rank_key 目前的键为 (z, 1-sup, d_wall, cog)，
        #   在此基础上把「该件是否落在与其停靠序号相称的纵深」作为
        #   末位微调 —— stop_seq 大者偏好大 x（里侧），小者偏好小 x（门侧）。
        #   以**软目标**实现：不改变任何硬约束（C1–C8 完全复用），
        #   只在同等 z / 支撑率 / 贴墙条件下优选纵深方向。
        orig_rank = base._rank_key

        def rank_with_delivery(p):
            k = orig_rank(p)
            stop = self.box_stop.get(p.uid, 0)
            max_stop = max(self.box_stop.values()) if self.box_stop else 1
            if max_stop <= 1:
                return k
            # 归一化纵深：期望停靠序号越大越靠里（x 越大）
            want_deep = (stop - 1) / (max_stop - 1)        # 0=最靠门, 1=最靠里
            x_c = (p.x + p.l / 2) / max(self.c.L, 1e-9)    # 实际纵深比重
            # 偏差越小越好；仅作末位键，不干扰前三级
            return k + (round(abs(x_c - want_deep), 3),)

        base._rank_key = rank_with_delivery
        try:
            r = base.pack(ordered)
        finally:
            base._rank_key = orig_rank
        r["装载顺序口径"] = {
            "策略": "配送感知（停靠序号降序 → 底面积降序 → 高/重降序）",
            "纵深偏好": "末位软目标：停靠序号大者偏好车厢里侧，小者偏好门侧",
            "说明": "后送门店的货先装且落位更深；先送的货后装且靠近车门。",
        }
        return r


def compare(orders_path: Path, out_path: Path) -> dict:
    """
    对比「非配送感知」与「配送感知」两种装载，并各自生成装卸指引自证。
    """
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "loading"))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    from packer import Box, Container
    from layer_packer import LayerPacker
    from hangzhou_net import VEHICLE_TYPES
    from unload_guide import build_loading_plan

    od = json.loads(orders_path.read_text(encoding="utf-8"))
    sku_map = {s["id"]: s for s in od["sku_lib"]}

    route_stores = ["S01", "S02", "S05", "S10", "S14", "S17",
                    "S18", "S19", "S23", "S25", "S27", "S30"]
    stop_of = {sid: k + 1 for k, sid in enumerate(route_stores)}

    boxes, meta, box_stop = [], {}, {}
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
                box_stop[uid] = stop_of[o["store_id"]]

    vt = VEHICLE_TYPES[0]
    c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                  vt.cap_weight_kg * 1000)
    container = {"名称": c.name, "L": c.L, "W": c.W, "H": c.H}

    out = {"路线门店": route_stores, "货量件数": len(boxes)}

    # ---- 方案 A：非配送感知（原算法）
    rA = LayerPacker(c).pack(boxes)
    pA = [dict(p, store_id=meta.get(p["uid"], ""),
               stop_seq=stop_of.get(meta.get(p["uid"], ""), 0))
          for p in rA["placement"]]
    planA = build_loading_plan(pA, {}, container)

    # ---- 方案 B：配送感知
    rB = DeliveryAwareLayerPacker(c, box_stop).pack(boxes)
    pB = [dict(p, store_id=meta.get(p["uid"], ""),
               stop_seq=stop_of.get(meta.get(p["uid"], ""), 0))
          for p in rB["placement"]]
    planB = build_loading_plan(pB, {}, container)

    def summarize(r, plan):
        z = plan["自证"]
        total = max(plan["件数"], 1)
        lb = z.get("送达序理论下界", {})
        floor = lb.get("逆序支撑对_理论最小违规", 0)
        got = z["卸货序送达序违规数"]
        return {
            "已装件数": r["装载"]["已装件数"],
            "空间利用率_%": r["利用率"]["空间利用率_%"],
            "卸货序物理违规": z["卸货序违规数"],
            "后送先卸违规": got,
            "其中物理性冲突": z["送达序其中物理性冲突"],
            "其中可优化冲突": z["送达序其中可优化冲突"],
            "后送先卸达成率_%": round((1 - got / total) * 100, 2),
            "理论最小违规_下界": floor,
            "距理论下界": got - floor,
            "理论最高达成率_%": lb.get("理论最高达成率_%", 0),
        }

    out["方案A_非配送感知"] = summarize(rA, planA)
    out["方案B_配送感知"] = summarize(rB, planB)
    da = out["方案A_非配送感知"]["后送先卸违规"]
    db = out["方案B_配送感知"]["后送先卸违规"]
    out["改善"] = {
        "后送先卸违规降幅_%": round((da - db) / da * 100, 2) if da > 0 else 0.0,
        "空间利用率变化_百分点": round(
            out["方案B_配送感知"]["空间利用率_%"]
            - out["方案A_非配送感知"]["空间利用率_%"], 2),
    }
    out["方案B_卸货序前三步"] = planB["卸货序"][:3]
    out["方案B_装载序前三步"] = planB["装载序"][:3]

    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=1),
                        encoding="utf-8")
    return out


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import ROOT

    r = compare(ROOT / "out" / "orders.json",
                ROOT / "out" / "delivery_aware.json")
    print("=== 配送感知装载对比（12 店路线 · 新能源微面）===")
    print(f"路线门店 {len(r['路线门店'])} 家 | 货量 {r['货量件数']} 件\n")
    for name in ("方案A_非配送感知", "方案B_配送感知"):
        s = r[name]
        print(f"[{name}]")
        print(f"  已装 {s['已装件数']} 件 | 空间利用率 {s['空间利用率_%']}%")
        print(f"  卸货序物理违规 {s['卸货序物理违规']} 处")
        print(f"  后送先卸违规 {s['后送先卸违规']} 处 "
              f"(物理性 {s['其中物理性冲突']} / 可优化 {s['其中可优化冲突']})")
        print(f"  后送先卸达成率 {s['后送先卸达成率_%']}% "
              f"| 理论最高 {s['理论最高达成率_%']}% "
              f"(下界 {s['理论最小违规_下界']} 处，距下界 {s['距理论下界']})")
        print()
    print(f"⇒ 后送先卸违规降幅 {r['改善']['后送先卸违规降幅_%']}% | "
          f"空间利用率变化 {r['改善']['空间利用率变化_百分点']} 个百分点")
