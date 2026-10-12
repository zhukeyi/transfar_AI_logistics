# -*- coding: utf-8 -*-
"""
子问题 2a · 仓内拣货路径优化与最小翻垛配货

════════════════════════════════════════════════════════════════════════
一、问题
════════════════════════════════════════════════════════════════════════
中心仓按门店订单生成拣货任务。仓内是**通道-货架**结构：
    · 货架沿通道两侧布置，每个货位 (aisle, side, level, slot)
    · 叉车/拣货车只能沿通道行走，跨通道需走横向通道（cross-aisle）
拣货单要求：
    1. **路径最短**：给定待拣货位集合，规划走行顺序使行走距离最小
    2. **翻垛最少**：同货位深位（rear）的货被前排（front）挡住时，
       需先把前排搬开才能取到后排 —— 每次搬开计 1 次「翻垛」。
       拣货顺序应尽量减少翻垛次数。

════════════════════════════════════════════════════════════════════════
二、模型
════════════════════════════════════════════════════════════════════════
**2.1 行走距离**：仓库为「多通道 + 两端横通道」的经典布局。
    通道内行走距离 = |行内坐标差|
    跨通道行走距离 = 走最近的横通道（y=0 端或 y=Y 端）绕行
    即 d(s1, s2) = |x1-x2| (同通道) 或
       |x1-x_front| + cross + |x_front-x2| (跨通道，取两端较小者)

**2.2 拣货路径**：本质是 TSP 的**度量（metric）变体**（走行距离满足三角
    不等式）。仓库拣货路径的经典结论是「**S 形（蛇形 / S-shape）策略**
    在单块仓库中近似最优」。故本实现采用两段式：
      - 构造层：**S 形通道遍历**（逐个通道蛇形推进，天然近优，作业员易懂）
      - 改进层：**2-opt + Or-opt** 局部搜索（严格保证不劣于构造解）
    并给出「朴素按单序拣货」基线作为对照。

**2.3 最小翻垛**：把货位按 (通道, 侧, 层) 分组，组内按「深位序号」排序：
    必须先取前排（深位序号小）再取后排。若拣货顺序违反该约束，
    则产生翻垛。本实现：
      - 用**拓扑偏序**表达「前 → 后」的先后要求
      - 在局部搜索中，把「翻垛次数」作为**次级目标**（主目标为距离），
        即 lexicographic (距离, 翻垛)：仅当地理距离不变时才接受翻垛更少的解

════════════════════════════════════════════════════════════════════════
三、口径
════════════════════════════════════════════════════════════════════════
仓库货位布局为**合成的典型值**（依据连锁药店中心仓的常见形制构造），
不代表任何具体仓库的真实布局。订单为模拟数据。

★ 数据声明：全部为合成数据，可复现（固定随机种子）。
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, asdict, field
from pathlib import Path


# ══════════════════════════════════════════════ 仓库布局
@dataclass
class Slot:
    """货位。

    坐标语义（仓库平面，单位 m）：
        aisle  : 通道号（0 起）
        side   : 通道内的侧（0=左, 1=右）
        level  : 层号（0=地面层）
        depth  : 深位序号（0=最靠通道外侧，即前排；越大越靠里）
        x      : 沿通道方向的位置（0 = 通道入口处）
    """
    code: str
    aisle: int
    side: int
    level: int
    depth: int
    x: float
    sku_id: str


@dataclass
class Warehouse:
    """仓库平面布局。"""
    name: str
    n_aisles: int            # 通道数
    aisle_len: float         # 通道长度 m
    cross_width: float       # 横通道宽度 m
    slot_pitch: float        # 货位沿通道方向的间距 m
    depth_per_slot: float    # 每个深位的进深 m
    n_levels: int            # 层数
    max_depth: int           # 每货位的深位数

    @property
    def total_y(self) -> float:
        """通道之间的中心距（横向 span）。"""
        return self.cross_width + self.n_aisles * 0.0 + (self.n_aisles - 1) * 4.0

    def slot_pos(self, aisle: int, x: float) -> tuple[float, float]:
        """货位在仓库平面上的近似坐标（m）。

        仓库为「多通道横向排列」：通道 i 的中心线 y = i * aisle_spacing。
        货位在通道 i 的两侧，取其通道中心线即可（对走行距离影响为常数）。
        """
        y = aisle * 4.0            # 通道中心距 4 m（含货架宽度）
        return x, y

    def aisle_entry(self, aisle: int) -> tuple[float, float]:
        """通道入口（x=0 端）。"""
        return 0.0, aisle * 4.0

    def aisle_exit(self, aisle: int) -> tuple[float, float]:
        """通道出口（x=aisle_len 端）。"""
        return self.aisle_len, aisle * 4.0

    def travel(self, a1: int, x1: float, a2: int, x2: float) -> float:
        """
        两点间走行距离（m）。

        同通道：沿通道直行 |x1-x2|
        跨通道：可走 y=0 端横通道或 y=max 端横通道，取较短者：
            min( x1 + x2 ,  (L-x1) + (L-x2) ) + 横向跨通道距离
        横向跨通道距离 = |a1 - a2| × 通道间距
        """
        if a1 == a2:
            return abs(x1 - x2)
        cross = abs(a1 - a2) * 4.0
        via_front = x1 + x2                 # 走 x=0 端
        via_back = (self.aisle_len - x1) + (self.aisle_len - x2)   # 走 x=L 端
        return min(via_front, via_back) + cross


def build_warehouse(sku_ids: list[str], seed: int = 20261011) -> tuple[Warehouse, dict[str, Slot]]:
    """
    构造仓库布局与 SKU→货位 映射。

    形制（连锁药店中心仓典型值）：
        8 条通道 × 通道长 40 m × 每 1.0 m 一个货位 × 每货位 3 深位 × 3 层
        ⇒ 8 × 40 × 3 × 3 = 2880 个货位
    """
    wh = Warehouse(
        name="杭州中心仓 · 整箱拣选区",
        n_aisles=8,
        aisle_len=40.0,
        cross_width=3.0,
        slot_pitch=1.0,
        depth_per_slot=1.2,
        n_levels=3,
        max_depth=3,
    )
    rng = random.Random(seed)
    slots: dict[str, Slot] = {}
    # ══════════════════════════════════════════════════════════════════
    # 货位分配规则（★ 直接决定翻垛是否真实存在）
    #
    # 真实的整箱拣选区里，一个货位组（通道 × 侧 × 层）内通常存放
    # **同品类但不同 SKU** 的整箱货，且深位（进深方向）有 2–3 个箱位。
    # 「后排被前排挡住」正是翻垛的来源。
    #
    # 本实现按「SKU 分品类家族」分配：
    #   - 把 SKU 按 id 顺序每 3 个编成一个「家族」（同组同深位带）；
    #   - 家族内交错占据 depth 0,1,2 ⇒ 组内既有前排也有后排；
    #   - 沿通道方向 x 逐格推进，形成真实的货架排布。
    # ══════════════════════════════════════════════════════════════════
    FAMILY = 3                      # 每个货位组容纳的 SKU 数（= 深位数）
    FAMILY_PER_X = wh.n_levels * 2  # 一个 x 位置可容纳的货位组数（层 × 两侧）
    for i, sku in enumerate(sorted(sku_ids)):
        fam = i // FAMILY           # 家族号 → 决定是哪个货位组
        member = i % FAMILY         # 家族内序号 → 深位
        # 家族号依次分配到 (通道, x 格, 侧, 层)
        gi = fam                            # 组序号
        aisle = gi % wh.n_aisles
        gx = (gi // wh.n_aisles) % 5        # 每通道取 5 个 x 格
        side = (gi // (wh.n_aisles * 5)) % 2
        level = (gi // (wh.n_aisles * 5 * 2)) % wh.n_levels
        x = gx * 7.0 + 1.0
        x = min(x, wh.aisle_len - 1.0)
        depth = member              # ★ 家族成员依次占据深位 0,1,2
        code = f"A{aisle+1:02d}-{'L' if side==0 else 'R'}-{level+1}-{depth+1}"
        slots[sku] = Slot(code=code, aisle=aisle, side=side, level=level,
                          depth=depth, x=x, sku_id=sku)
    return wh, slots


# ══════════════════════════════════════════════ 翻垛
def count_restacks(order: list[Slot]) -> int:
    """
    统计给定拣货顺序下的**翻垛次数**。

    规则：同一 (通道, 侧, 层) 组内，深位越靠前（depth 小）的货越先被取走才
    不需要翻垛。若顺序中出现「先取 depth 大的，后取 depth 小的」，
    则前者需要先搬开后者 —— 每次违反计 1 次翻垛。

    等价表述：对每个组，统计顺序中的**逆序对数**（按 depth 升序为有序）。
    """
    groups: dict[tuple, list[int]] = {}
    for idx, s in enumerate(order):
        groups.setdefault((s.aisle, s.side, s.level), []).append(idx)
    restacks = 0
    for key, idxs in groups.items():
        # 取该组的 depth 序列，统计相对顺序的逆序对
        depths = [order[i].depth for i in idxs]
        n = len(depths)
        for i in range(n):
            for j in range(i + 1, n):
                if depths[i] > depths[j]:
                    restacks += 1
    return restacks


# ══════════════════════════════════════════════ 求解器
@dataclass
class PickRoute:
    order: list[str]           # 货位 code 序列
    distance_m: float          # 走行距离
    restacks: int              # 翻垛次数
    legs: list[dict] = field(default_factory=list)


class PickPlanner:
    """
    拣货路径规划器。

    两段式：
      1. 构造 S 形（蛇形通道遍历）初始解 —— 仓库拣货的经典近优策略
      2. 2-opt + Or-opt 局部搜索，目标函数为 **字典序 (距离, 翻垛)**
    """

    def __init__(self, wh: Warehouse, slots: dict[str, Slot],
                 start_aisle: int = 0, start_x: float = 0.0):
        self.wh = wh
        self.slots = slots
        self.start_aisle = start_aisle
        self.start_x = start_x

    # ---------------------------------------------- 目标函数
    def evaluate(self, seq: list[Slot]) -> tuple[float, int]:
        """返回 (走行距离 m, 翻垛次数)。字典序比较用。"""
        if not seq:
            return 0.0, 0
        dist = 0.0
        a0, x0 = self.start_aisle, self.start_x
        for s in seq:
            dist += self.wh.travel(a0, x0, s.aisle, s.x)
            a0, x0 = s.aisle, s.x
        # 返程
        dist += self.wh.travel(a0, x0, self.start_aisle, self.start_x)
        return dist, count_restacks(seq)

    # ---------------------------------------------- 构造：S 形
    def s_shape(self, slots_todo: list[Slot]) -> list[Slot]:
        """
        S 形（蛇形）构造 + **翻垛前置约束**。

        通道号升序推进；每条通道内，按 x 方向**交替**（奇数通道从 x=0 到
        x=L，偶数通道从 x=L 到 x=0），形成蛇形走位，避免每条通道都返程。

        ★ 翻垛消解：同一 (通道,侧,层) 组内，深位靠前（depth 小）的货必须
          先取。若单纯按 x 排序会破坏该偏序（同组的货可能 x 不同），故在
          排序键中**把 depth 作为首要键之一**：
              键 = (深度桶, x 或 -x)
          即先把同组的前排货排在前面，再按蛇形方向排序。
        """
        by_aisle: dict[int, list[Slot]] = {}
        for s in slots_todo:
            by_aisle.setdefault(s.aisle, []).append(s)
        seq: list[Slot] = []
        for k, aisle in enumerate(sorted(by_aisle)):
            items = by_aisle[aisle]
            reverse = (k % 2 == 1)
            # 首要键 depth：保证组内前排先拣（消解翻垛）
            # 次序键 x：按蛇形方向排列
            items.sort(key=lambda s: (s.depth,
                                      -s.x if reverse else s.x))
            seq.extend(items)
        return seq

    # ---------------------------------------------- 改进：2-opt / Or-opt
    def improve(self, seq: list[Slot], max_iter: int = 200) -> list[Slot]:
        """
        局部搜索。目标为字典序 (距离, 翻垛)。

        邻域：
          · 2-opt：反转一段（对度量 TSP 是标准邻域）
          · Or-opt：把一段长度 ≤3 的片段搬到别处（含前后插入）
        """
        if len(seq) < 4:
            return seq
        cur = list(seq)
        cur_cost = self.evaluate(cur)
        improved = True
        it = 0
        while improved and it < max_iter:
            improved = False
            it += 1
            n = len(cur)
            # ---- 2-opt
            for i in range(n - 1):
                for j in range(i + 1, n):
                    cand = cur[:i] + cur[i:j + 1][::-1] + cur[j + 1:]
                    c = self.evaluate(cand)
                    if c < cur_cost:
                        cur, cur_cost = cand, c
                        improved = True
                        break
                if improved:
                    break
            if improved:
                continue
            # ---- Or-opt（段长 1-3）
            for seg_len in (1, 2, 3):
                for i in range(0, n - seg_len + 1):
                    seg = cur[i:i + seg_len]
                    rest = cur[:i] + cur[i + seg_len:]
                    for j in range(len(rest) + 1):
                        if j == i:
                            continue
                        cand = rest[:j] + seg + rest[j:]
                        c = self.evaluate(cand)
                        if c < cur_cost:
                            cur, cur_cost = cand, c
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break
        return cur

    # ---------------------------------------------- 基线：按单序
    def naive_order(self, slots_todo: list[Slot]) -> list[Slot]:
        """
        人工经验基线：按**拣货单行序**拣货。

        现场最常见的做法 —— 拣货员按订单系统打印出的拣货单自上而下逐行
        拣，不做路径规划，也不看货位深位。拣货单的行序由订单录入顺序决定，
        与货位的物理位置、深位**无关**。

        这里用「按待拣 SKU 在原订单中首次出现的顺序」模拟该行序，
        正是这种「按单序」的做法同时造成**绕路**与**翻垛**。
        """
        return list(slots_todo)

    # ---------------------------------------------- 求解
    def solve(self, slots_todo: list[Slot], do_improve: bool = True) -> PickRoute:
        seq = self.s_shape(slots_todo)
        if do_improve:
            seq = self.improve(seq)
        dist, rk = self.evaluate(seq)
        legs = []
        a0, x0 = self.start_aisle, self.start_x
        for s in seq:
            d = self.wh.travel(a0, x0, s.aisle, s.x)
            legs.append({"to": s.code, "sku": s.sku_id, "aisle": s.aisle + 1,
                         "x_m": round(s.x, 2), "depth": s.depth + 1,
                         "leg_m": round(d, 2)})
            a0, x0 = s.aisle, s.x
        legs.append({"to": "START", "sku": "-", "aisle": self.start_aisle + 1,
                     "x_m": round(self.start_x, 2), "depth": 0,
                     "leg_m": round(self.wh.travel(a0, x0, self.start_aisle,
                                                   self.start_x), 2)})
        return PickRoute(order=[s.code for s in seq], distance_m=round(dist, 2),
                         restacks=rk, legs=legs)


# ══════════════════════════════════════════════ 主流程
def run(net_path: Path, orders_path: Path, out_path: Path) -> dict:
    od = json.loads(orders_path.read_text(encoding="utf-8"))
    sku_ids = [s["id"] for s in od["sku_lib"]]
    wh, slots = build_warehouse(sku_ids)

    # 汇总待拣 SKU。★ 按「订单录入顺序」（orders.json 中订单与行的天然顺序）
    # 收集，使「按单序基线」反映真实的拣货单行序，而非人为打乱。
    need: dict[str, int] = {}
    for o in od["orders"]:
        for ln in o["lines"]:
            need[ln["sku_id"]] = need.get(ln["sku_id"], 0) + ln["qty"]
    todo = [slots[sid] for sid in need if sid in slots]

    planner = PickPlanner(wh, slots)
    naive_seq = planner.naive_order(todo)
    algo = planner.solve(todo, do_improve=True)
    construct_only = planner.solve(todo, do_improve=False)
    nd, nr = planner.evaluate(naive_seq)

    result = {
        "仓库": {"名称": wh.name,
                 "通道数": wh.n_aisles, "通道长_m": wh.aisle_len,
                 "层数": wh.n_levels, "每货位深位": wh.max_depth,
                 "货位总数": wh.n_aisles * int(wh.aisle_len / wh.slot_pitch)
                             * wh.n_levels * wh.max_depth},
        "拣货任务": {"待拣SKU数": len(todo),
                     "覆盖门店数": len({o["store_id"] for o in od["orders"]})},
        "基线_按单序": {"走行距离_m": round(nd, 2), "翻垛次数": nr},
        "构造_S形": {"走行距离_m": construct_only.distance_m,
                     "翻垛次数": construct_only.restacks},
        "算法_S形+局部搜索": {"走行距离_m": algo.distance_m,
                              "翻垛次数": algo.restacks,
                              "拣货序列": algo.order,
                              "分段明细": algo.legs},
    }
    if nd > 0:
        result["改进幅度"] = {
            "距离降幅_%": round((nd - algo.distance_m) / nd * 100, 2),
            "翻垛降幅_%": (round((nr - algo.restacks) / nr * 100, 2)
                          if nr > 0 else 0.0),
        }
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=1),
                        encoding="utf-8")
    return result


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import ROOT

    r = run(ROOT / "out" / "network.json", ROOT / "out" / "orders.json",
            ROOT / "out" / "picking_result.json")
    print("=== 仓内拣货路径优化 ===")
    print(f"仓库: {r['仓库']['名称']} | 通道 {r['仓库']['通道数']} × "
          f"{r['仓库']['通道长_m']}m × {r['仓库']['层数']}层 × "
          f"{r['仓库']['每货位深位']}深位 = {r['仓库']['货位总数']} 货位")
    print(f"待拣 SKU {r['拣货任务']['待拣SKU数']} 个")
    print(f"  基线·按单序      : {r['基线_按单序']['走行距离_m']:.1f} m, "
          f"翻垛 {r['基线_按单序']['翻垛次数']} 次")
    print(f"  构造·S形蛇行     : {r['构造_S形']['走行距离_m']:.1f} m, "
          f"翻垛 {r['构造_S形']['翻垛次数']} 次")
    print(f"  算法·S形+局部搜索: {r['算法_S形+局部搜索']['走行距离_m']:.1f} m, "
          f"翻垛 {r['算法_S形+局部搜索']['翻垛次数']} 次")
    if "改进幅度" in r:
        print(f"  ⇒ 距离降 {r['改进幅度']['距离降幅_%']}% | "
              f"翻垛降 {r['改进幅度']['翻垛降幅_%']}%")
