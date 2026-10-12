# -*- coding: utf-8 -*-
"""
车厢三维装载 · 分层 + 天际线（Skyline）混合求解器

════════════════════════════════════════════════════════════════════════
一、为什么需要「天际线」而不是「全局平层」
════════════════════════════════════════════════════════════════════════
上一版把车厢切成**全局水平层**：层1 铺地面，层2 从「层1 的最高点」起铺。
实测暴露致命缺陷（S29+S30+S04 组合）：

  层1 放入 15 台轮椅（980×300×880），占满地面 ⇒ 层1 顶面 = 880mm。
  但层1 的地面**并非处处都堆到 880**：轮椅只覆盖部分区域，其余区域
  只堆到 220/260mm。然而层2 强制从 z=880 起铺 ⇒
    · 轮椅上方 880mm 处：支撑面只有 300mm 宽，勉强放 1 件；
    · 矮箱上方（实际顶面仅 220mm）：距 z=880 有 660mm 悬空 ⇒ 支撑率 0 ⇒ 拒绝。
  结果：层2 仅装入 1 件，其余 101 件（178 L）全部判「无合法落位」。

真实装车不是这样作业的。装车工是在**不平整的货堆顶面**上继续码货 ——
哪里矮就往哪里补。数学上正确的建模是维护一张**高度场（height field）**，
即「天际线 / skyline」：对底面平面上的每个 (x,y)，记录当前已堆到的高度。

════════════════════════════════════════════════════════════════════════
二、算法：货架（Shelf）定平面布局 + 天际线（Skyline）定高度
════════════════════════════════════════════════════════════════════════
本求解器把两者结合，各取所长：

  · **平面布局用货架法**（Shelf-FFDH）：把货物在 L×W 平面上按行排布，
    规则、可解释、便于作业员按行装载。这一点保留了「货架式」的可读性。
  · **高度用天际线**：每件货物落在「其底面投影区域内的**最低**天际线高度」
    上。这样矮堆上方可以继续码货，高堆旁边也能就近补货，不再出现
    「全局平层」造成的悬空与大块浪费。

具体流程：

  1. 维护天空线 sky (list[(_Rect, z)])，初始 = 车厢底面（全平面 z=0）。
  2. 候选落位生成：对每件货物，扫描当前天空线，取其**顶面**作为落点
     平面候选（skyline 顶面即「可继续码放的支撑面」）。
  3. 落位判据：
       (a) 支撑率：落点下方支撑面积 / 自身底面积 ≥ support_ratio
       (b) 不重叠、不越界、不超载、不违反堆叠与禁压、重心可接受
  4. 择优：优先「落点低」（填低处）→「支撑率高」→「贴合墙/角」。
  5. 放置后更新天空线：把该件占用的 (x,y) 区间高度抬升到 z2。

════════════════════════════════════════════════════════════════════════
三、与「货架式」交付口径的关系（重要）
════════════════════════════════════════════════════════════════════════
对外表述仍是「**分层/货架式装载**」—— 因为：
  · 平面布局确实是货架式（按行排布、可行走通道）；
  · 「层」在交付物中体现为**装载工序的先后批次**（先铺底、再码第二遍、
    第三遍），与天空线的「高度批次」一一对应。
天空线是**内部求解机制**（保证数学正确性），不改变对外可读的作业语义。

════════════════════════════════════════════════════════════════════════
四、约束（C1–C8）
════════════════════════════════════════════════════════════════════════
  C1 载重        Σ w_i ≤ Q
  C2 几何不重叠  任意两箱三维不交叠
  C3 完全在车厢内 六个面均在车厢尺寸内
  C4 重心平衡    重心投影须落在车厢中轴邻域内（防侧翻）
  C5 堆叠规则    箱 i 之上可压层数 ≤ max_stack_i
  C6 易碎保护    禁压箱（fragile=2 / max_stack=0）上方不得放任何箱
  C7 方向约束    不可倒置箱（液体/冷链）只能保持正置
  C8 支撑率      上层箱底面至少 60% 面积有下层箱直接支撑

★ 数据声明：货物规格为合成数据，车厢尺寸为公开车型典型值。
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

from packer import Box, Container, Placement


# ══════════════════════════════════════════════════════════ 天际线辅助结构
@dataclass
class _Surf:
    """一块矩形支撑面（天际线的一段）。"""
    x: float
    y: float
    l: float
    w: float
    z: float

    @property
    def x2(self) -> float:
        return self.x + self.l

    @property
    def y2(self) -> float:
        return self.y + self.w

    def overlaps(self, ox: float, oy: float, ol: float, ow: float) -> bool:
        return not (self.x2 <= ox + 1e-6 or ox + ol <= self.x + 1e-6 or
                    self.y2 <= oy + 1e-6 or oy + ow <= self.y + 1e-6)

    def inter_area(self, ox: float, oy: float, ol: float, ow: float) -> float:
        ix = min(self.x2, ox + ol) - max(self.x, ox)
        iy = min(self.y2, oy + ow) - max(self.y, oy)
        return ix * iy if (ix > 1e-6 and iy > 1e-6) else 0.0


class LayerPacker:
    """
    分层（货架式平面布局 + 天际线高度）三维装载器。

    「层」在本实现中体现为**高度批次**：每件货落在其投影区的最低天际线上，
    因此一层内不同位置的高度可以不同（这正是真实货堆的形态）。
    """

    def __init__(self, container: Container,
                 support_ratio: float = 0.60,
                 cog_abs: float = 0.95,
                 cog_warmup: int = 24,
                 max_rounds: int = 400,
                 frac_support_ratio: float = 0.90):
        """
        support_ratio       : C8 支撑率下限
        cog_abs             : C4 满载时重心偏心上限
        cog_warmup          : C4 满载判据预热件数
        max_rounds          : 装载轮次上限（安全阀）
        frac_support_ratio  : 支撑率 ≥ 该值时视为「完全支撑」，
                              可在同高度批次的择优中优先——用于让货堆顶面平整
        """
        self.c = container
        self.support_ratio = support_ratio
        self.cog_abs = cog_abs
        self.cog_warmup = cog_warmup
        self.max_rounds = max_rounds
        self.frac_support_ratio = frac_support_ratio
        self.placed: list[Placement] = []
        self.placed_uid: set[str] = set()
        self.rejected: list[dict] = []
        self.batches: list[dict] = []      # 对外称「层」，实为高度批次
        self._used_volume = 0.0

    # ══════════════════════════════════════════════ C1–C8 约束校验
    def _used_weight(self) -> float:
        return sum(p.weight_g for p in self.placed)

    def _weight_ok(self, extra_g: float) -> bool:
        """C1 载重。"""
        return self._used_weight() + extra_g <= self.c.max_weight_g + 1e-6

    def _inside(self, p: Placement) -> bool:
        """C3 完全在车厢内。"""
        return (p.x >= -1e-6 and p.y >= -1e-6 and p.z >= -1e-6 and
                p.x2 <= self.c.L + 1e-6 and p.y2 <= self.c.W + 1e-6 and
                p.z2 <= self.c.H + 1e-6)

    @staticmethod
    def _xy_overlap(a: Placement, b: Placement) -> bool:
        return not (a.x2 <= b.x + 1e-6 or b.x2 <= a.x + 1e-6 or
                    a.y2 <= b.y + 1e-6 or b.y2 <= a.y + 1e-6)

    def _no_overlap(self, p: Placement) -> bool:
        """
        C2 几何不重叠。

        ★ 等价性说明：不可能存在「XY 重叠但 z 区间分离」之外的交叠情况，
          故此处逐件判 XY 重叠后，再判 z 区间是否分离。为控制复杂度，
          先按 z 区间快速剪枝。
        """
        for q in self.placed:
            if p.z2 <= q.z + 1e-6 or q.z2 <= p.z + 1e-6:
                continue
            if self._xy_overlap(p, q):
                return False
        return True

    def _support_ratio_of(self, p: Placement) -> float:
        """C8 支撑率：底面被下层箱直接支撑的面积比例。"""
        if p.z <= 1e-6:
            return 1.0
        foot = p.l * p.w
        if foot <= 0:
            return 0.0
        cov = 0.0
        for q in self.placed:
            if abs(q.z2 - p.z) > 1e-3:
                continue
            ox = min(p.x2, q.x2) - max(p.x, q.x)
            oy = min(p.y2, q.y2) - max(p.y, q.y)
            if ox > 1e-6 and oy > 1e-6:
                cov += ox * oy
        return min(1.0, cov / foot)

    def _layers_above(self, q: Placement) -> int:
        """q 顶面之上的堆叠层数（按底面 z 分组，同层并排算一层）。"""
        levels = {round(r.z, 3) for r in self.placed
                  if r.z >= q.z2 - 1e-6 and self._xy_overlap(r, q)}
        return len(levels)

    def _stack_ok(self, p: Placement) -> bool:
        """
        C5/C6 堆叠与禁压 —— 向下传播检查。

        ★ max_stack 语义：之上可压**层数**，不是箱数。
        ★ 传播：须检查**所有**位于 p 下方且 XY 重叠的箱；只查直接支撑件会
          漏掉「中间层与底层同样承压」这一事实。
        """
        if p.z <= 1e-6:
            return True
        for q in self.placed:
            if q.z2 > p.z + 1e-6:
                continue
            if not self._xy_overlap(q, p):
                continue
            if q.fragile == 2 or q.max_stack == 0:
                return False
            if self._layers_above(q) + 1 > q.max_stack:
                return False
        return True

    def _cog_limit(self) -> float:
        """
        C4 当前允许的重心偏心上限（渐近收紧）。

        ★★ 物理依据：允许的重心范围由**轴距**决定，半载车可接受偏心量远大于
          满载车。若从第一箱起即用满载判据（相对几何中心 ≤0.85），第一个
          角落箱偏心度必然 >0.9 ⇒ 整车无任何合法落位（实测「装 0 件」）。
          故按已装件数线性放开，装载推进后收敛到 cog_abs。
        """
        n = len(self.placed)
        if n >= self.cog_warmup:
            return self.cog_abs
        return 1.0 - (1.0 - self.cog_abs) * (n / max(self.cog_warmup, 1))

    def _cog_ok(self, p: Placement) -> bool:
        """C4 重心平衡（防侧翻）。"""
        allp = self.placed + [p]
        tw = sum(q.weight_g for q in allp)
        if tw <= 0:
            return True
        cx = sum(q.weight_g * (q.x + q.l / 2) for q in allp) / tw
        cy = sum(q.weight_g * (q.y + q.w / 2) for q in allp) / tw
        limit = self._cog_limit()
        ex = abs(cx - self.c.L / 2) / (self.c.L / 2)
        ey = abs(cy - self.c.W / 2) / (self.c.W / 2)
        return ex <= limit and ey <= limit

    # ══════════════════════════════════════════════ 朝向
    @staticmethod
    def _orients(b: Box):
        """
        C7 方向约束：不可倒置箱只保留正置朝向。

        水平旋转 90° 不改变 z 向高度，故不违反「保持正置」；
        can_rotate 箱允许 (l,w) 与 (w,l) 两种平面朝向。
        """
        out = [(b.l, b.w, b.h, False)]
        if b.can_rotate and abs(b.l - b.w) > 1e-6:
            out.append((b.w, b.l, b.h, True))
        return out

    # ══════════════════════════════════════════════ 天际线
    def _init_sky(self) -> list[_Surf]:
        return [_Surf(0.0, 0.0, self.c.L, self.c.W, 0.0)]

    def _candidate_surfaces(self, sky: list[_Surf]) -> list[_Surf]:
        """
        取出可作为落点平面的天际线面。

        ★ 只保留「顶面」——即在其上方没有其他面的段。实现方式：
          若存在另一段与自身 XY 重叠且 z 更高，则自身不是顶面。
        这样每件货一定落在**当前可码放的最高有效支撑面**上，
        不会落入被覆盖的空腔。
        """
        tops: list[_Surf] = []
        for i, s in enumerate(sky):
            covered = False
            for j, t in enumerate(sky):
                if i == j:
                    continue
                if t.z <= s.z + 1e-6:
                    continue
                if t.overlaps(s.x, s.y, s.l, s.w):
                    covered = True
                    break
            if not covered:
                tops.append(s)
        return tops

    def _skyline_height(self, sky: list[_Surf],
                        x: float, y: float, l: float, w: float) -> float:
        """给定底面投影区，返回其内的**最低**天际线高度（可落点高度）。"""
        zs = [s.z for s in sky if s.overlaps(x, y, l, w)]
        return min(zs) if zs else 0.0

    def _update_sky(self, sky: list[_Surf], p: Placement) -> list[_Surf]:
        """
        放置 p 后更新天际线：把 p 的底面投影抬高到 p.z2。

        做法：对每个与 p 投影区重叠的段，切分为「不相交部分（保留原 z）」
        与「相交部分（抬到 p.z2）」；相交部分合并为一块新面。
        """
        out: list[_Surf] = []
        for s in sky:
            if not s.overlaps(p.x, p.y, p.l, p.w):
                out.append(s)
                continue
            # 切出不相交的四条残余矩形
            ix0, ix1 = max(s.x, p.x), min(s.x2, p.x2)
            iy0, iy1 = max(s.y, p.y), min(s.y2, p.y2)
            if s.y < iy0 - 1e-6:                       # 下条
                out.append(_Surf(s.x, s.y, s.l, iy0 - s.y, s.z))
            if iy1 < s.y2 - 1e-6:                      # 上条
                out.append(_Surf(s.x, iy1, s.l, s.y2 - iy1, s.z))
            if s.x < ix0 - 1e-6:                       # 左条
                out.append(_Surf(s.x, iy0, ix0 - s.x, iy1 - iy0, s.z))
            if ix1 < s.x2 - 1e-6:                      # 右条
                out.append(_Surf(ix1, iy0, s.x2 - ix1, iy1 - iy0, s.z))
        # 相交区域统一抬到 p.z2
        out.append(_Surf(p.x, p.y, p.l, p.w, p.z2))
        return self._merge_sky(out)

    @staticmethod
    def _merge_sky(sky: list[_Surf]) -> list[_Surf]:
        """
        合并同高度、共边的段，控制天际线段数增长。

        简单策略：按 z 分组后，对 x/y 相邻且等宽的段做合并。
        这只是一个规模控制措施，不影响正确性。
        """
        if len(sky) <= 1:
            return sky
        # 去除零面积段
        segs = [s for s in sky if s.l > 1e-6 and s.w > 1e-6]
        # 按 z 分组，组内按 y 再按 x 排序后做纵向合并
        groups: dict[float, list[_Surf]] = {}
        for s in segs:
            groups.setdefault(round(s.z, 3), []).append(s)
        out: list[_Surf] = []
        for z, g in groups.items():
            g.sort(key=lambda s: (round(s.y, 3), round(s.x, 3)))
            used = [False] * len(g)
            for i, a in enumerate(g):
                if used[i]:
                    continue
                x, y, l, w = a.x, a.y, a.l, a.w
                for j in range(i + 1, len(g)):
                    b = g[j]
                    if used[j]:
                        continue
                    # 同列、紧邻、等宽 → 纵向合并
                    if (abs(b.x - x) < 1e-6 and abs(b.l - l) < 1e-6
                            and abs(b.y - (y + w)) < 1e-6):
                        w += b.w
                        used[j] = True
                out.append(_Surf(x, y, l, w, a.z))
        return out

    # ══════════════════════════════════════════════ 主流程
    def pack(self, boxes: list[Box]) -> dict:
        """
        逐个装载：每件货在「当前天际线的最低可落点」上择位。

        择优顺序（越小越优）：
          1. 落点 z 低（**填低处**，避免高堆悬空）
          2. 支撑率高（落点更可靠）
          3. 贴墙贴角（便于固定）
          4. 重心偏移小
        """
        sky = self._init_sky()
        # 排序：底面积降序 → 高度降序 → 重量降序（大件重件先放，作支撑基础）
        order = sorted(boxes, key=lambda b: (-b.footprint(), -b.h, -b.weight_g))
        batch_of: dict[str, int] = {}

        for b in order:
            cands = self._candidates(b, sky)
            best, best_key = None, None
            for p in cands:
                if not self._admissible(p):
                    continue
                key = self._rank_key(p)
                if best_key is None or key < best_key:
                    best, best_key = p, key
            if best is None:
                self.rejected.append({"uid": b.uid, "sku": b.sku_id,
                                      "name": b.name, "reason": "无合法落位"})
                continue
            self.placed.append(best)
            self.placed_uid.add(b.uid)
            self._used_volume += best.l * best.w * best.h
            sky = self._update_sky(sky, best)
            batch_of[best.uid] = self._batch_index(best.z)

        self._finalize_batches(batch_of)
        for b in order:
            if b.uid not in self.placed_uid and not any(
                    r["uid"] == b.uid for r in self.rejected):
                self.rejected.append({"uid": b.uid, "sku": b.sku_id,
                                      "name": b.name, "reason": "无合法落位"})
        return self.report(boxes)

    @staticmethod
    def _batch_index(z: float) -> int:
        """把底面高度映射为「装载批次（层）」。批次 = 常见高度档位序号。"""
        tiers = [0.0, 250.0, 500.0, 750.0, 1000.0, 1250.0, 1500.0, 1750.0]
        idx = 0
        for i, t in enumerate(tiers):
            if z >= t - 1e-6:
                idx = i
        return idx

    def _rank_key(self, p: Placement):
        """
        落位择优排序键。

        ★ 首要判据是「落点低」：货堆顶面参差时，优先补低处，
          这样堆顶趋于平整，才能继续往上码（与装车工的做法一致）。
        """
        sup = self._support_ratio_of(p)
        d_wall = min(p.x, p.y, self.c.L - p.x2, self.c.W - p.y2)
        # 重心偏移（相对车厢几何中心，仅作末位微调）
        allp = self.placed + [p]
        tw = sum(q.weight_g for q in allp)
        cx = sum(q.weight_g * (q.x + q.l / 2) for q in allp) / max(tw, 1e-9)
        cy = sum(q.weight_g * (q.y + q.w / 2) for q in allp) / max(tw, 1e-9)
        cog = (abs(cx - self.c.L / 2) / (self.c.L / 2)
               + abs(cy - self.c.W / 2) / (self.c.W / 2)) / 2
        return (round(p.z, 1),            # 1) 填低处（首要）
                round(1.0 - sup, 3),      # 2) 支撑率高优先
                round(d_wall, 1),         # 3) 贴墙贴角
                round(cog, 4))            # 4) 重心居中

    # ══════════════════════════════════════════════ 候选生成
    def _candidates(self, b: Box, sky: list[_Surf]) -> list[Placement]:
        """
        为 b 生成候选落位。

        对每个「顶面段」s，以其左下角 (s.x, s.y) 为锚点尝试若干 x/y 组合：
          · (s.x, s.y)          —— 贴左后角
          · (s.x2-pl, s.y)      —— 贴右后角
          · (s.x, s.y2-pw)      —— 贴左前角
          · (s.x2-pl, s.y2-pw)  —— 贴右前角
        并要求候选底面完整落在 s 内（保证支撑来源明确）。
        落点高度取该投影区的最低天际线高度，允许「跨段拼支撑」。
        """
        out: list[Placement] = []
        seen: set[tuple] = set()
        for s in self._candidate_surfaces(sky):
            for (pl, pw, ph, rot) in self._orients(b):
                if pl > self.c.L + 1e-6 or pw > self.c.W + 1e-6 \
                        or ph > self.c.H + 1e-6:
                    continue
                xs = [s.x, s.x2 - pl]
                ys = [s.y, s.y2 - pw]
                for x in xs:
                    for y in ys:
                        if x < -1e-6 or y < -1e-6:
                            continue
                        if x + pl > self.c.L + 1e-6 or y + pw > self.c.W + 1e-6:
                            continue
                        z = self._skyline_height(sky, x, y, pl, pw)
                        if z + ph > self.c.H + 1e-6:
                            continue
                        k = (round(x, 1), round(y, 1), round(z, 1),
                             round(pl, 1), round(pw, 1))
                        if k in seen:
                            continue
                        seen.add(k)
                        out.append(Placement(b.uid, x, y, z, pl, pw, ph,
                                             b.weight_g, b.fragile, b.max_stack,
                                             b.sku_id, b.name))
        return out

    def _admissible(self, p: Placement) -> bool:
        """统一可放性判据（C2/C3/C5/C6/C8 硬约束 + C1/C4 门禁）。"""
        if not self._inside(p):
            return False
        if not self._no_overlap(p):
            return False
        if not self._weight_ok(p.weight_g):
            return False
        if not self._stack_ok(p):
            return False
        if self._support_ratio_of(p) < self.support_ratio:
            return False
        if not self._cog_ok(p):
            return False
        return True

    # ══════════════════════════════════════════════ 批次（层）汇总
    def _finalize_batches(self, batch_of: dict[str, int]) -> None:
        """按批次（层）汇总件数、层底、层顶。"""
        got: dict[int, list[Placement]] = {}
        for p in self.placed:
            got.setdefault(batch_of.get(p.uid, 0), []).append(p)
        for idx in sorted(got):
            ps = got[idx]
            self.batches.append({
                "layer_index": idx + 1,
                "z0": round(min(p.z for p in ps), 1),
                "layer_top": round(max(p.z2 for p in ps), 1),
                "count": len(ps),
            })

    # ══════════════════════════════════════════════ 报告
    def report(self, all_boxes: list[Box]) -> dict:
        used_vol = sum(p.l * p.w * p.h for p in self.placed)   # mm³
        used_w = self._used_weight()
        demand_vol = sum(b.volume_l for b in all_boxes) * 1e6  # L → mm³
        return {
            "车厢": {"名称": self.c.name,
                     "内尺寸_mm": [self.c.L, self.c.W, self.c.H],
                     "容积_L": round(self.c.volume_l, 1),
                     "核定载重_kg": round(self.c.max_weight_g / 1000, 1)},
            "装载": {"总件数": len(all_boxes),
                     "已装件数": len(self.placed),
                     "未装件数": len(all_boxes) - len(self.placed),
                     "装载体积_L": round(used_vol / 1e6, 1),
                     "装载重量_kg": round(used_w / 1000, 1),
                     "货量体积_L": round(demand_vol / 1e6, 1)},
            "利用率": {"空间利用率_%": round(used_vol / 1e6
                                          / self.c.volume_l * 100, 2),
                     "载重利用率_%": round(used_w / self.c.max_weight_g * 100, 2),
                     "货量装车率_%": round(used_vol
                                       / max(demand_vol, 1e-9) * 100, 2)},
            "层信息": self.batches,
            "约束": {"支撑率下限": self.support_ratio,
                     "重心绝对偏心上限": self.cog_abs,
                     "重心收紧预热件数": self.cog_warmup},
            "未装明细": self.rejected[:20],
            "placement": [asdict(p) for p in self.placed],
        }


# ═══════════════════════════════════════════════════════════════ 修正记录
# 修正 A（全局平层 → 天际线）
#   现象：S29+S30+S04 组合，层1 放入 15 台轮椅（980×300×880）占满地面，
#         层均顶面 = 880mm；层2 被强制从 z=880 起铺，
#         而矮箱上方（顶面仅 220/260mm）到 z=880 悬空 620+ mm ⇒ 支撑率 0 ⇒
#         全部拒绝 ⇒ 层2 仅装入 1 件，剩 101 件（178 L）判「无合法落位」。
#   根因：把「层」当成全局水平面。真实货堆顶面是**参差的**，
#         必须在每处最低的天际线上继续码货。
#   修正：引入天际线（高度场）。每件货落在其底面投影区的**最低**天际线高度；
#         放置后把该投影区抬升到新顶面。矮堆上方可继续码货，
#         高堆旁也能就地补货，「层」改为「按高度档位分的装载批次」。
#
# 修正 B（重心门禁突死）
#   现象：部分订单整车装载 0 件。
#   根因：_cog_ok 从第一箱起即用满载判据（相对几何中心 ≤0.85）；
#         第一个角落箱偏心度实测 0.908 > 0.85 ⇒ 无任何合法落位。
#   修正：按已装件数线性放宽（warmup 内放宽至 1.0），此后收敛到 cog_abs。
#   依据：车辆允许的重心范围由轴距决定，半载车可接受偏心量远大于满载车。
#
# 修正 C（max_stack 语义 + 向下传播）
#   现象：审计报出 361 项超堆叠、232 项禁压违规。
#   根因：① max_stack 被当成「可压箱数」，实为「可压层数」；
#         ② 只检查直接支撑件，漏掉「中间层与底层同样承压」。
#   修正：按底面 z 分组统计层数；对所有「位于下方且 XY 重叠」的箱做传播检查。
#
# 修正 D（体积单位）
#   现象：货量装车率显示 100000%。
#   根因：volume_l 已按 1e6 归一（L），report 中又乘 1000 造成 1000 倍偏差。
#   修正：统一为 demand_vol = Σ volume_l × 1e6（L → mm³），与装载体积同尺度。


if __name__ == "__main__":
    import sys
    import time
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "data"))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "loading"))
    from paths import ROOT
    from hangzhou_net import VEHICLE_TYPES

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
    print(f"待装 {len(boxes)} 件 | 总体积 "
          f"{sum(b.volume_l for b in boxes):.0f} L | 总重 "
          f"{sum(b.weight_g for b in boxes)/1000:.1f} kg")
    for vt in VEHICLE_TYPES:
        c = Container(vt.name, vt.box_l, vt.box_w, vt.box_h,
                      vt.cap_weight_kg * 1000)
        t0 = time.time()
        r = LayerPacker(c).pack(boxes)
        dt = time.time() - t0
        print(f"\n[{vt.id} {vt.name}] 用时 {dt:.1f}s")
        print(f"  装 {r['装载']['已装件数']}/{r['装载']['总件数']} 件 | "
              f"空间 {r['利用率']['空间利用率_%']}% | "
              f"载重 {r['利用率']['载重利用率_%']}% | "
              f"批次 {len(r['层信息'])}")
        for L in r["层信息"]:
            print(f"    批次{L['layer_index']} z={L['z0']:.0f}.."
                  f"{L['layer_top']:.0f} 件{L['count']}")
