# -*- coding: utf-8 -*-
"""
货物 (SKU) 模型与电商动态要货订单生成

药品/连锁药店城市配送的货物特征：
  - 规格极碎：从 60mm 小药盒到 600mm 整箱医疗器械
  - 重量跨度大：50g 到 12kg
  - 易碎性：玻璃瓶装糖浆、试剂、玻璃体温计等需禁压
  - 堆叠规则：药品有明确的正置/禁倒/层数上限要求（GSP 规范约束）

★ 口径声明：SKU 规格为**合成的典型值**，量级参照零售药品包装常见范围，
  不代表任何具体商品的真实包装参数。订单为模拟数据。
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass
class SKU:
    id: str
    name: str
    # 包装尺寸 mm
    l: int
    w: int
    h: int
    weight_g: float
    # 易碎等级: 0=普通 1=轻压可 2=禁压(易碎)
    fragile: int
    # 允许最大堆叠层数（自身之上可压几层）
    max_stack: int
    # 是否可倒置
    can_rotate: bool
    # 是否需温控（冷链药品）
    cold_chain: bool
    category: str

    @property
    def volume_l(self) -> float:
        return self.l * self.w * self.h / 1_000_000.0


# SKU 库：按药品零售典型品类构造（合成规格）
SKU_LIB: list[SKU] = []


def _mk(sid, name, l, w, h, g, fragile, max_stack, rot, cold, cat):
    SKU_LIB.append(SKU(sid, name, l, w, h, g, fragile, max_stack, rot, cold, cat))


# —— 小规格口服药（盒装）
_mk("K001", "感冒灵颗粒(盒)",   120,  80,  60,  65, 0, 8, True,  False, "口服药")
_mk("K002", "维生素C片(瓶)",     60,  60, 110, 120, 0, 6, True,  False, "保健品")
_mk("K003", "阿莫西林胶囊(盒)",  110,  75,  55,  55, 0, 8, True,  False, "处方药")
_mk("K004", "布洛芬缓释(盒)",    130,  85,  60,  70, 0, 8, True,  False, "口服药")
_mk("K005", "降压药(盒)",        100,  70,  50,  48, 0, 8, True,  False, "慢病药")
_mk("K006", "降糖药(盒)",        100,  70,  50,  52, 0, 8, True,  False, "慢病药")
# —— 瓶装液体（中规格，禁倒置）
_mk("K007", "止咳糖浆(瓶)",      80,  80, 180, 340, 1, 4, False, False, "液体制剂")
_mk("K008", "碘伏消毒液(瓶)",    75,  75, 150, 260, 1, 4, False, False, "外用")
_mk("K009", "生理盐水(瓶)",      90,  90, 220, 560, 1, 3, False, False, "输液")
# —— 玻璃容器（易碎，禁压）
_mk("K010", "玻璃体温计(盒)",    180,  40,  25,  45, 2, 0, True,  False, "器械")
_mk("K011", "试剂瓶(盒)",        150, 100, 120, 480, 2, 0, False, False, "检验")
_mk("K012", "口服液玻璃瓶(箱)",  200, 150, 160, 1250, 2, 1, False, False, "液体制剂")
# —— 医疗器械（大件，较重）
_mk("K013", "制氧机(台)",        420, 260, 520, 9800, 1, 2, False, False, "器械")
_mk("K014", "轮椅(台)",          980, 300, 880, 12500, 0, 1, False, False, "器械")
_mk("K015", "电子血压计(盒)",    180, 140, 100, 480, 1, 4, True,  False, "器械")
_mk("K016", "雾化器(台)",        220, 180, 260, 1600, 1, 3, False, False, "器械")
# —— 冷链药品
_mk("K017", "胰岛素笔(盒)",      150, 100,  60, 180, 1, 5, True,  True,  "冷链")
_mk("K018", "疫苗冷藏盒",        260, 180, 160, 1450, 1, 3, False, True,  "冷链")
_mk("K019", "益生菌(盒)",        120,  90,  80, 210, 1, 5, True,  True,  "冷链")
# —— 耗材（可压，规整）
_mk("K020", "医用口罩(箱)",      400, 300, 220, 2400, 0, 6, True,  False, "耗材")
_mk("K021", "纱布绷带(箱)",      350, 260, 200, 1800, 0, 6, True,  False, "耗材")
_mk("K022", "注射器(箱)",        380, 280, 180, 2100, 0, 5, True,  False, "耗材")
_mk("K023", "创可贴(盒)",        140, 100,  70, 150, 0, 8, True,  False, "耗材")
_mk("K024", "消毒棉签(盒)",      160, 110,  90, 190, 0, 8, True,  False, "耗材")


@dataclass
class OrderLine:
    sku_id: str
    qty: int
    store_id: str


@dataclass
class Order:
    """动态要货订单：电商场景下订单在时间轴上陆续到达。"""
    id: str
    store_id: str
    lines: list[OrderLine]
    release_min: int      # 下单时刻（分钟，自 06:00 起算）
    tag: str              # 订单类型：常规补货 / 紧急要货 / 电商即时单


def gen_orders(net: dict, seed: int = 20261011,
               horizon_min: int = 720) -> list[Order]:
    """
    生成电商动态要货订单流。

    场景设定：连锁药店总部中心仓，门店通过线上系统动态补货
      - 常规补货单：夜间/凌晨批量下单（release 集中于 0–120 min）
      - 电商即时单：白天陆续到达（release 分散于 0–600 min），时效要求紧
      - 紧急要货单：随时可能到达（release 均匀分布），需插单
    """
    rng = random.Random(seed)
    stores = [n["id"] for n in net["nodes"] if n["id"] != "WH"]
    orders: list[Order] = []
    oid = 0

    def sku_by_category(prefer: list[str] | None = None) -> SKU:
        pool = SKU_LIB
        if prefer:
            sub = [s for s in SKU_LIB if s.category in prefer]
            if sub:
                pool = sub
        return rng.choice(pool)

    # 1) 常规补货单：每店 1–2 张，凌晨到达
    for sid in stores:
        for _ in range(rng.randint(1, 2)):
            oid += 1
            nl = rng.randint(3, 9)
            lines = []
            used = set()
            for _ in range(nl):
                sk = sku_by_category()
                if sk.id in used:
                    continue
                used.add(sk.id)
                # 药品按小批量补货：1–8 个单位
                lines.append(OrderLine(sk.id, rng.randint(1, 8), sid))
            if lines:
                orders.append(Order(
                    f"O{oid:04d}", sid, lines,
                    release_min=rng.randint(0, 120), tag="常规补货"))

    # 2) 电商即时单：白天陆续到达，时效紧
    for _ in range(rng.randint(14, 20)):
        oid += 1
        sid = rng.choice(stores)
        nl = rng.randint(1, 4)
        lines = [OrderLine(sku_by_category(["口服药", "保健品", "耗材"]).id,
                           rng.randint(1, 4), sid) for _ in range(nl)]
        orders.append(Order(
            f"O{oid:04d}", sid, lines,
            release_min=rng.randint(60, 600), tag="电商即时"))

    # 3) 紧急要货单：随时到达
    for _ in range(rng.randint(5, 8)):
        oid += 1
        sid = rng.choice(stores)
        lines = [OrderLine(sku_by_category(["冷链", "处方药", "器械"]).id,
                           rng.randint(1, 3), sid) for _ in range(rng.randint(1, 2))]
        orders.append(Order(
            f"O{oid:04d}", sid, lines,
            release_min=rng.randint(0, horizon_min), tag="紧急要货"))

    orders.sort(key=lambda o: o.release_min)
    return orders


def summarize(orders: list[Order]) -> dict:
    by_tag, by_store, qty = {}, {}, 0
    for o in orders:
        by_tag[o.tag] = by_tag.get(o.tag, 0) + 1
        q = sum(l.qty for l in o.lines)
        qty += q
        by_store[o.store_id] = by_store.get(o.store_id, 0) + q
    sku_ids = {l.sku_id for o in orders for l in o.lines}
    return {"订单数": len(orders), "总件数": qty,
            "涉及SKU": len(sku_ids), "按类型": by_tag,
            "覆盖门店": len(by_store)}


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    net = json.loads((root / "out" / "network.json").read_text(encoding="utf-8"))
    orders = gen_orders(net)
    s = summarize(orders)
    print("=== 订单流汇总 ===")
    for k, v in s.items():
        print(f"  {k}: {v}")
    print(f"\nSKU 库: {len(SKU_LIB)} 种")
    tot_v = sum(x.volume_l for x in SKU_LIB)
    tot_w = sum(x.weight_g for x in SKU_LIB)
    print(f"SKU 平均单体体积: {tot_v/len(SKU_LIB):.2f} L")
    print(f"SKU 平均单体重量: {tot_w/len(SKU_LIB):.0f} g")
    fr = [x for x in SKU_LIB if x.fragile == 2]
    cc = [x for x in SKU_LIB if x.cold_chain]
    print(f"禁压易碎 SKU: {len(fr)} 种 | 冷链 SKU: {len(cc)} 种")

    out = root / "out"
    payload = {
        "orders": [{"id": o.id, "store_id": o.store_id, "tag": o.tag,
                    "release_min": o.release_min,
                    "lines": [asdict(l) for l in o.lines]} for o in orders],
        "sku_lib": [asdict(s) for s in SKU_LIB],
    }
    (out / "orders.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    print("\n已写出 out/orders.json")
