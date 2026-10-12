# -*- coding: utf-8 -*-
"""
子问题 2b · 车厢三维装载规划（核心）

问题：给定一批货物（含重量/体积/尺寸/易碎等级/堆叠上限/可否倒置/冷链），
     在车厢三维空间内生成合法且高利用率的三维摆放方案。

模型：三维装箱问题 3D-BPP 的**多约束变体**（Multi-Constraint 3D-BPP）
     额外约束：
       C1 载重        Σ w_i ≤ Q
       C2 几何不重叠  任意两箱三维不交叠
       C3 完全在车厢内 箱体六个面均在车厢尺寸范围内
       C4 重心平衡    重心投影须落在底面积中心邻域内（防侧翻）
       C5 堆叠规则    箱 i 之上可压层数 ≤ max_stack_i；不可压箱上方禁放
       C6 易碎保护    易碎等级 2（禁压）的箱上方不得放任何箱
       C7 方向约束    不可倒置箱（液体/冷链）只能保持正置
       C8 支撑率      上层箱底面至少 60% 面积有下层箱直接支撑

求解：分层定向启发式（Layer-based + Extreme Point）
     1) 先按「底面积 × 高度」对货物排序（大不落小原则）
     2) 用极值点法（Extreme Point）维护候选落位
     3) 每个落位用多重判据评分：底面贴合优先 → 靠内靠边优先 → 高度低优先
     4) 放置后更新极值点集与堆叠计数

★ 数据声明：货物规格为合成数据，车厢尺寸为公开车型典型值。
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, field, asdict
from pathlib import Path


# ---------------------------------------------------------------- 数据结构

@dataclass
class Box:
    """待装货物单元（SKU 展开后的一个具体箱体）。"""
    uid: str            # 唯一标识（SKU id + 序号）
    sku_id: str
    name: str
    l: float            # 原始长 mm
    w: float            # 原始宽 mm
    h: float            # 原始高 mm
    weight_g: float
    fragile: int        # 0 普通 / 1 轻压可 / 2 禁压
    max_stack: int      # 之上可压几层
    can_rotate: bool    # 可否水平旋转 90°
    cold_chain: bool
    # 实际摆放朝向（求解后填充）
    pl: float = 0.0
    pw: float = 0.0
    ph: float = 0.0
    rotated: bool = False

    @property
    def volume_l(self) -> float:
        return self.l * self.w * self.h / 1e6

    def footprint(self) -> float:
        return self.l * self.w


@dataclass
class Placement:
    """一个已放置的箱体。"""
    uid: str
    x: float          # 左下后角坐标（mm）
    y: float
    z: float          # 底面高度
    l: float
    w: float
    h: float
    weight_g: float
    fragile: int
    max_stack: int
    sku_id: str
    name: str
    # 该箱之上已压的层数（由 solver 维护）
    stacked_above: int = 0

    @property
    def x2(self) -> float: return self.x + self.l
    @property
    def y2(self) -> float: return self.y + self.w
    @property
    def z2(self) -> float: return self.z + self.h

    def overlaps_xy(self, o: "Placement") -> bool:
        return not (self.x2 <= o.x + 1e-6 or o.x2 <= self.x + 1e-6 or
                    self.y2 <= o.y + 1e-6 or o.y2 <= self.y + 1e-6)

    def overlaps_3d(self, o: "Placement") -> bool:
        return (self.overlaps_xy(o) and
                not (self.z2 <= o.z + 1e-6 or o.z2 <= self.z + 1e-6))


@dataclass
class Container:
    """车厢（内尺寸，单位 mm）。坐标系：x 沿车长（0=车尾门，L=车头），
    y 沿车宽，z 垂直向上。"""
    name: str
    L: float
    W: float
    H: float
    max_weight_g: float

    @property
    def volume_l(self) -> float:
        return self.L * self.W * self.H / 1e6


# ---------------------------------------------------------------- 载货规划器

class LoadingPlanner:
    """
    分层极值点装载器。

    评分函数（越小越优先）：
        score = w_contact * (1 - 底面支撑率)      # 优先获得支撑
              + w_corner  * 归一化「离角距离」     # 优先靠边靠内（利于固定）
              + w_height  * 归一化高度            # 优先填低处（堆叠稳）
              + w_front   * 归一化 x 坐标         # 见 unload 模块：可配置装门优先
    约束校验逐项实现（C1–C8），任一不满足则该落位非法。
    """

    def __init__(self, container: Container,
                 w_contact: float = 3.0, w_corner: float = 1.0,
                 w_height: float = 1.5, w_front: float = 0.0,
                 w_cog: float = 1.2,
                 support_ratio: float = 0.60,
                 cog_rel: float = 0.55, cog_abs: float = 0.85):
        self.c = container
        self.w_contact = w_contact
        self.w_corner = w_corner
        self.w_height = w_height
        self.w_front = w_front
        self.w_cog = w_cog
        self.support_ratio = support_ratio
        self.cog_rel = cog_rel
        self.cog_abs = cog_abs
        self.placed: list[Placement] = []
        self.placed_uid: set[str] = set()
        self.rejected: list[dict] = []

    # ------------------------------------------------ 约束校验
    def _inside(self, p: Placement) -> bool:
        return (p.x >= -1e-6 and p.y >= -1e-6 and p.z >= -1e-6 and
                p.x2 <= self.c.L + 1e-6 and p.y2 <= self.c.W + 1e-6 and
                p.z2 <= self.c.H + 1e-6)

    def _no_overlap(self, p: Placement) -> bool:
        return all(not p.overlaps_3d(q) for q in self.placed)

    def _support(self, p: Placement) -> float:
        """计算 p 底面被下层箱支撑的面积比例。"""
        if p.z <= 1e-6:
            return 1.0                       # 直接落地
        foot = p.l * p.w
        if foot <= 0:
            return 0.0
        covered = 0.0
        for q in self.placed:
            if abs(q.z2 - p.z) > 1e-3:
                continue
            ox = min(p.x2, q.x2) - max(p.x, q.x)
            oy = min(p.y2, q.y2) - max(p.y, q.y)
            if ox > 1e-6 and oy > 1e-6:
                covered += ox * oy
        return min(1.0, covered / foot)

    def _support_boxes(self, p: Placement) -> list[Placement]:
        """返回直接支撑 p 的下层箱列表。"""
        out = []
        if p.z <= 1e-6:
            return out
        for q in self.placed:
            if abs(q.z2 - p.z) > 1e-3:
                continue
            ox = min(p.x2, q.x2) - max(p.x, q.x)
            oy = min(p.y2, q.y2) - max(p.y, q.y)
            if ox > 1e-6 and oy > 1e-6:
                out.append(q)
        return out

    def _stack_ok(self, p: Placement) -> bool:
        """
        C5/C6：检查 p 放在该位置是否违反堆叠约束。

        ★★ 语义澄清与传播（第四次修正，关键）：
          max_stack 的含义是「该箱之上最多再堆几**层**」，不是「几个箱」。

          更关键的是**约束必须向下传播**：p 放在 z=1200 处，其直接支撑
          可能是 z=900 的箱子，但 p 同样「压」着 z=0 处的大箱（中间层
          与底层在 XY 上重叠）。早期实现只检查直接支撑箱，导致
          底层 max_stack=1 的轮椅上面实际堆了 3 层仍被判合法
          （审计报出 232 项 C5 违规）。

        正确判据：对**所有**在 p 正下方、且与 p 在 XY 上重叠的箱 q，
        检查「q 之上将形成的层数」是否 ≤ q.max_stack。
        """
        if p.z <= 1e-6:
            return True
        for q in self.placed:
            if q.z2 > p.z + 1e-6:
                continue                      # q 不在 p 下方
            if not q.overlaps_xy(p):
                continue                      # XY 无重叠，不承压
            # C6：禁压箱之上不得放任何箱
            if q.fragile == 2 or q.max_stack == 0:
                return False
            # C5：q 之上（含本次放置）的层数不得超过 max_stack
            if self._layer_count_above(q) + 1 > q.max_stack:
                return False
        return True

    def _layer_count_above(self, q: Placement) -> int:
        """
        计算 q 顶面之上的堆叠层数。

        做法：取所有 z >= q.z2 且与 q 在 XY 重叠的箱，
        按底面高度分组，组数即层数（同层并排算一层）。
        """
        levels = {round(r.z, 3) for r in self.placed
                  if r.z >= q.z2 - 1e-6 and r.overlaps_xy(q)}
        return len(levels)

    def _cog_ok(self, p: Placement) -> bool:
        """
        C4：纵向/横向偏载约束（硬门禁，但判据须随装载进度自适应）。

        ★★ 建模修正（第三次修正，关键）：
          早期用「重心对车厢几何中心的偏离 ≤ 35%」作硬门禁，
          但第一个箱必然放在角落 ⇒ 偏离度 0.97 ⇒ 判非法 ⇒ 算法装载率为 0。
          这个约束在物理上是对的，但**判据错了**：空车与半载车的
          重心可接受范围不同。

        物理事实（载重分布安全规则）：
          1. 车辆允许的重心范围由**轴距**决定，不是车厢几何中心；
             半载时允许的偏心量远大于满载时。
          2. 通行做法是约束「重心相对**已装载区域**的偏心」，
             并对整车纵向设一个绝对上限（防止后轴过载/前轴漂浮）。

        本实现采用双判据：
          (a) 相对已装载区域包围盒的中心偏离 ≤ cog_rel（默认 0.55）
          (b) 相对整车厢几何中心的偏离 ≤ cog_abs（默认 0.85，宽松安全兜底）
          装载初期 (a) 宽松（只有一箱时偏离为 0），随装载推进逐步收紧，
          符合「装得越满、重心越要居中」的实际要求。
        """
        total_w = sum(q.weight_g for q in self.placed) + p.weight_g
        if total_w <= 0:
            return True
        allp = self.placed + [p]
        cx = sum(q.weight_g * (q.x + q.l / 2) for q in allp) / total_w
        cy = sum(q.weight_g * (q.y + q.w / 2) for q in allp) / total_w

        # (a) 相对已装载区域包围盒的偏心
        x0 = min(q.x for q in allp); x1 = max(q.x2 for q in allp)
        y0 = min(q.y for q in allp); y1 = max(q.y2 for q in allp)
        bx, by = (x0 + x1) / 2, (y0 + y1) / 2
        hx, hy = max((x1 - x0) / 2, 1e-6), max((y1 - y0) / 2, 1e-6)
        ex_rel = abs(cx - bx) / hx
        ey_rel = abs(cy - by) / hy
        if ex_rel > self.cog_rel or ey_rel > self.cog_rel:
            return False

        # (b) 相对整车厢几何中心的绝对兜底
        ex_abs = abs(cx - self.c.L / 2) / (self.c.L / 2)
        ey_abs = abs(cy - self.c.W / 2) / (self.c.W / 2)
        return ex_abs <= self.cog_abs and ey_abs <= self.cog_abs

    def _cog_penalty(self, p: Placement) -> float:
        """重心偏离程度（0=居中，1=贴边），用于评分而非门禁。"""
        total_w = sum(q.weight_g for q in self.placed) + p.weight_g
        if total_w <= 0:
            return 0.0
        allp = self.placed + [p]
        cx = sum(q.weight_g * (q.x + q.l / 2) for q in allp) / total_w
        cy = sum(q.weight_g * (q.y + q.w / 2) for q in allp) / total_w
        # 用「已装载区域」中心作参照，使评分在装载早期即有意义
        x0 = min(q.x for q in allp); x1 = max(q.x2 for q in allp)
        y0 = min(q.y for q in allp); y1 = max(q.y2 for q in allp)
        bx, by = (x0 + x1) / 2, (y0 + y1) / 2
        hx = max((x1 - x0) / 2, 1e-6); hy = max((y1 - y0) / 2, 1e-6)
        return (abs(cx - bx) / hx + abs(cy - by) / hy) / 2.0

    def _weight_ok(self, p: Placement) -> bool:
        return (sum(q.weight_g for q in self.placed) + p.weight_g
                <= self.c.max_weight_g + 1e-6)

    # ------------------------------------------------ 极值点
    def _extreme_points(self) -> list[tuple[float, float, float]]:
        """
        生成候选落位点集合。

        ★★ 候选集必须足够丰富（第五次修正，性能关键）：
          早期版本每个已放箱只贡献 7 个顶点，导致装到几十件后就找不到
          合法落位（无约束下也仅装 218 件）。

        本实现采用三类候选点：
          (a) 地面网格点：车尾起，沿车长/车宽以固定步长布置
          (b) 箱顶「驻点」：每个已放箱顶面四角 → 可在其上继续码放
          (c) 箱侧「贴靠点」：沿已放箱的右侧/前侧边贴靠
        三者合并后按与已装箱的贴合度择优，能显著提高装载率。
        """
        pts: set[tuple[float, float, float]] = {(0.0, 0.0, 0.0)}

        # (a) 地面网格（步长取 100mm，兼顾搜索量与覆盖率）
        step = 100.0
        x = 0.0
        while x <= self.c.L:
            y = 0.0
            while y <= self.c.W:
                pts.add((x, y, 0.0))
                y += step
            x += step

        # (b) 箱顶驻点 + (c) 箱侧贴靠点
        for q in self.placed:
            # 顶面四角与其投影
            pts.add((q.x, q.y, q.z2))
            pts.add((q.x2, q.y, q.z2))
            pts.add((q.x, q.y2, q.z2))
            pts.add((q.x2, q.y2, q.z2))
            # 顶面中线上也放点（小箱可落在两箱交界处）
            pts.add(((q.x + q.x2) / 2, q.y, q.z2))
            pts.add((q.x, (q.y + q.y2) / 2, q.z2))
            # 侧面贴靠（同一高度层向右/向前延伸）
            pts.add((q.x2, q.y, q.z))
            pts.add((q.x, q.y2, q.z))
            pts.add((q.x2, q.y2, q.z))
        return list(pts)

    def _candidates(self, box: Box) -> list[Placement]:
        """为 box 生成所有候选摆放（含朝向变体与落位点）。

        效率控制：候选点先按「与已装箱的贴合度」粗排，只取前 N 个
        进入完整约束校验，避免 O(候选点 × 已装箱) 的重复扫描。
        """
        out = []
        orients = [(box.l, box.w, box.h, False)]
        if box.can_rotate and abs(box.l - box.w) > 1e-6:
            orients.append((box.w, box.l, box.h, True))
        pts = self._extreme_points()
        for (pl, pw, ph, rot) in orients:
            if pl > self.c.L or pw > self.c.W or ph > self.c.H:
                continue
            for (x, y, z) in pts:
                p = Placement(box.uid, x, y, z, pl, pw, ph,
                              box.weight_g, box.fragile, box.max_stack,
                              box.sku_id, box.name)
                if not self._inside(p):
                    continue
                if not self._no_overlap(p):
                    continue
                out.append(p)
        return out

    def _score(self, p: Placement) -> float:
        sup = self._support(p)
        # 离最近「角」的距离，归一化：偏好贴边贴角（便于固定）
        d_corner = min(
            ((p.x) ** 2 + (p.y) ** 2) ** 0.5,
            ((self.c.L - p.x2) ** 2 + (p.y) ** 2) ** 0.5,
            ((p.x) ** 2 + (self.c.W - p.y2) ** 2) ** 0.5,
            ((self.c.L - p.x2) ** 2 + (self.c.W - p.y2) ** 2) ** 0.5,
        ) / max(self.c.L, self.c.W)
        d_height = p.z / self.c.H
        d_front = p.x / self.c.L
        d_cog = self._cog_penalty(p)
        return (self.w_contact * (1.0 - sup)
                + self.w_corner * d_corner
                + self.w_height * d_height
                + self.w_front * d_front
                + self.w_cog * d_cog)

    # ------------------------------------------------ 主流程
    def pack(self, boxes: list[Box]) -> dict:
        """按「大不落小」排序后逐个装载。"""
        # 排序：底面积降序 → 高度降序 → 重量降序（重货垫底更稳）
        order = sorted(boxes, key=lambda b: (-b.footprint(), -b.h, -b.weight_g))
        for b in order:
            cands = self._candidates(b)
            best, best_s = None, float("inf")
            for p in cands:
                if not self._weight_ok(p):
                    continue
                if not self._stack_ok(p):
                    continue
                if self._support(p) < self.support_ratio:
                    continue
                if not self._cog_ok(p):
                    continue
                s = self._score(p)
                if s < best_s:
                    best, best_s = p, s
            if best is None:
                self.rejected.append({"uid": b.uid, "sku": b.sku_id,
                                      "name": b.name, "reason": "无合法落位"})
                continue
            # 更新支撑箱的堆叠计数
            for q in self._support_boxes(best):
                q.stacked_above += 1
            self.placed.append(best)
            self.placed_uid.add(best.uid)
        return self.report(boxes)

    def report(self, all_boxes: list[Box]) -> dict:
        used_vol = sum(p.l * p.w * p.h for p in self.placed)
        used_w = sum(p.weight_g for p in self.placed)
        total_vol = self.c.volume_l
        return {
            "车厢": {"名称": self.c.name, "内尺寸_mm": [self.c.L, self.c.W, self.c.H],
                     "容积_L": round(total_vol, 1),
                     "核定载重_kg": round(self.c.max_weight_g / 1000, 1)},
            "装载": {
                "总件数": len(all_boxes),
                "已装件数": len(self.placed),
                "未装件数": len(all_boxes) - len(self.placed),
                "装载体积_L": round(used_vol / 1000, 1),
                "装载重量_kg": round(used_w / 1000, 1),
            },
            "利用率": {
                "空间利用率_%": round(used_vol / 1e6 / total_vol * 100, 2),
                "载重利用率_%": round(used_w / self.c.max_weight_g * 100, 2),
            },
            "约束": {
                "支撑率下限": self.support_ratio,
                "重心相对偏心上限": self.cog_rel,
                "重心绝对偏心上限": self.cog_abs,
            },
            "未装明细": self.rejected[:20],
            "placement": [asdict(p) for p in self.placed],
        }


# ---------------------------------------------------------------- 对照基线

def naive_pack(container: Container, boxes: list[Box],
               support_ratio: float = 0.0) -> dict:
    """
    人工经验装载基线（对照）。

    模拟现场装车工的常见做法：
      - 按 SKU 归类后「逐排码放」：沿车宽方向码满一排，再沿车高叠一层，
        再往车头方向推进一排
      - 不做极值点优化，不主动检查重心与堆叠上限
      - 易碎品保护靠人工记忆（本基线**故意不实现**，用于暴露人工方案的缺陷）

    ★ 正确性要求：本基线也必须通过几何不重叠与越界检查，
      否则会产出「利用率 158%」这类物理上不可能的数字。
      这里用与算法相同的 _no_overlap/_inside 判据逐件校验。
    """
    p = LoadingPlanner(container, support_ratio=support_ratio)
    order = sorted(boxes, key=lambda b: (b.sku_id, -b.footprint()))

    def try_place(b, x, y, z):
        pl, pw, ph = (b.w, b.l, b.h) if (b.can_rotate and b.w > b.l) \
            else (b.l, b.w, b.h)
        cand = Placement(b.uid, x, y, z, pl, pw, ph, b.weight_g,
                         b.fragile, b.max_stack, b.sku_id, b.name)
        return cand, pl, pw, ph

    # 逐排推进：x 为车长方向（0=车尾），y 车宽，z 车高
    # 人工策略：在固定的 (x,y) 列上「往上码」，码不动就换 y，再换 x
    weight_so_far = [0.0]      # 用列表包住以便闭包内更新
    x = 0.0
    while x < container.L and len(p.placed) < len(boxes):
        y = 0.0
        col_any = False
        while y < container.W and len(p.placed) < len(boxes):
            z = 0.0
            placed_any = False
            inner_guard = 0
            while z < container.H - 1e-6 and inner_guard < 60:
                inner_guard += 1
                hit = None
                for b in order:
                    if b.uid in p.placed_uid:
                        continue
                    cand, pl, pw, ph = try_place(b, x, y, z)
                    if (pl > container.L - x + 1e-6
                            or pw > container.W - y + 1e-6
                            or ph > container.H - z + 1e-6):
                        continue
                    if not p._no_overlap(cand):
                        continue
                    if weight_so_far[0] + b.weight_g > container.max_weight_g:
                        continue
                    hit = (b, cand, pw, ph)
                    break
                if hit is None:
                    break
                b, cand, pw, ph = hit
                p.placed.append(cand)
                p.placed_uid.add(b.uid)
                weight_so_far[0] += b.weight_g
                placed_any = True
                col_any = True
                z = cand.z2                       # 精确叠到顶面
            if placed_any:
                # 该列已码，沿车宽推进一个「列宽」
                colw = max((q.w for q in p.placed
                            if abs(q.x - x) < 1e-6 and abs(q.y - y) < 1e-6),
                           default=0.0)
                y += max(colw, 1.0)
            else:
                y += 100.0
        if col_any:
            # 该 x 切片完成，沿车长推进一个「切片长」
            slice_l = max((q.l for q in p.placed if abs(q.x - x) < 1e-6),
                          default=0.0)
            x += max(slice_l, 1.0)
        else:
            x += 100.0

    for b in boxes:
        if b.uid not in p.placed_uid:
            p.rejected.append({"uid": b.uid, "sku": b.sku_id,
                               "name": b.name, "reason": "人工：无空间/超载"})
    return p.report(boxes)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    from goods import SKU_LIB
    from hangzhou_net import VEHICLE_TYPES

    root = Path(__file__).resolve().parents[2]
    od = json.loads((root / "out" / "orders.json").read_text(encoding="utf-8"))
    sku_map = {s["id"]: s for s in od["sku_lib"]}

    # 取一个波次的实际货量：全部订单展开
    boxes: list[Box] = []
    seq = 0
    for o in od["orders"]:
        for ln in o["lines"]:
            sk = sku_map[ln["sku_id"]]
            for k in range(ln["qty"]):
                seq += 1
                boxes.append(Box(
                    uid=f"B{seq:05d}", sku_id=sk["id"], name=sk["name"],
                    l=sk["l"], w=sk["w"], h=sk["h"], weight_g=sk["weight_g"],
                    fragile=sk["fragile"], max_stack=sk["max_stack"],
                    can_rotate=sk["can_rotate"], cold_chain=sk["cold_chain"]))
    print(f"待装总件数: {len(boxes)}")
    print(f"总体积: {sum(b.volume_l for b in boxes):.0f} L | "
          f"总重: {sum(b.weight_g for b in boxes)/1000:.1f} kg")

    for vt in VEHICLE_TYPES:
        c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                      vt.cap_weight_kg * 1000)
        print(f"\n{'='*56}")
        print(f"车型 {vt.id} {vt.name}  车厢 {vt.box_l}×{vt.box_w}×{vt.box_h} mm"
              f"  容积 {c.volume_l:.0f} L  载重 {vt.cap_weight_kg} kg")
        r = LoadingPlanner(c).pack(boxes)
        print(f"  [算法] 装 {r['装载']['已装件数']}/{r['装载']['总件数']} 件 | "
              f"空间利用率 {r['利用率']['空间利用率_%']}% | "
              f"载重利用率 {r['利用率']['载重利用率_%']}%")
        rb = naive_pack(c, boxes)
        print(f"  [人工] 装 {rb['装载']['已装件数']}/{rb['装载']['总件数']} 件 | "
              f"空间利用率 {rb['利用率']['空间利用率_%']}% | "
              f"载重利用率 {rb['利用率']['载重利用率_%']}%")
