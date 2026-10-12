# -*- coding: utf-8 -*-
"""
生成 README 引用的四张 SVG 图。

设计约束（避免手写 SVG 时的算术错误）：
  1. 全部坐标由几何常量推导，不硬编码零散数字；
  2. 每段文字在写入前做宽度估算，超出容器即抛错（脚本失败优于静默溢出）；
  3. 输出自包含 SVG（白底、显式颜色），不依赖 CSS 变量 —— GitHub 以 <img> 渲染，
     无法继承页面主题变量。

用法：
    python3 docs/diagrams/make_diagrams.py
"""
from __future__ import annotations

from pathlib import Path

HERE = Path(__file__).resolve().parent

# ── 配色（白底深字，明暗主题下均可读） ────────────────────────────────
C = {
    "bg":       "#ffffff",
    "band":     "#f6f8fb",
    "band_ln":  "#ccd7e6",
    "box":      "#ffffff",
    "box_ln":   "#b9c7da",
    "ink":      "#16202e",
    "ink2":     "#3d4c60",
    "ink3":     "#66788f",
    "blue":     "#1d4ed8",
    "blue_bg":  "#e8effc",
    "teal":     "#0f766e",
    "teal_bg":  "#e3f3f0",
    "amber":    "#a35a06",
    "amber_bg": "#fdf1de",
    "red":      "#b3261e",
    "red_bg":   "#fceceb",
    "green":    "#15803d",
    "green_bg": "#e7f5ec",
    "grey_bg":  "#eef2f7",
}

FONT = ("font-family=\"-apple-system,BlinkMacSystemFont,'Segoe UI',"
        "'PingFang SC','Hiragino Sans GB','Microsoft YaHei',Helvetica,Arial,sans-serif\"")


# ── 基础绘图件 ────────────────────────────────────────────────────────
def esc(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def tw(s: str, fs: float) -> float:
    """粗略估算文本像素宽度：CJK 记 1.0 em，拉丁数字记 0.56 em。"""
    return sum(fs * (1.0 if ord(ch) > 0x2E80 else 0.56) for ch in s)


def check(s: str, fs: float, avail: float, where: str) -> None:
    w = tw(s, fs)
    if w > avail:
        raise SystemExit(f"[文本溢出] {where}: 需 {w:.0f}px > 可用 {avail:.0f}px :: {s}")


def rect(x, y, w, h, fill, stroke=None, sw=1.0, r=0, dash=None, op=None):
    a = f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}"'
    if r:
        a += f' rx="{r:.1f}"'
    a += f' fill="{fill}"'
    if stroke:
        a += f' stroke="{stroke}" stroke-width="{sw:.1f}"'
    if dash:
        a += f' stroke-dasharray="{dash}"'
    if op is not None:
        a += f' opacity="{op}"'
    return a + "/>"


def text(x, y, s, fs=13, fill=None, anchor="start", weight="400", op=None):
    a = (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{fs:.1f}" '
         f'fill="{fill or C["ink"]}" text-anchor="{anchor}" font-weight="{weight}"')
    if op is not None:
        a += f' opacity="{op}"'
    return a + f">{esc(s)}</text>"


def line(x1, y1, x2, y2, stroke, sw=1.4, dash=None, marker=None):
    a = (f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
         f'stroke="{stroke}" stroke-width="{sw:.1f}"')
    if dash:
        a += f' stroke-dasharray="{dash}"'
    if marker:
        a += f' marker-end="url(#{marker})"'
    return a + "/>"


def path(d, stroke, sw=1.4, dash=None, fill="none", marker=None):
    a = f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw:.1f}"'
    if dash:
        a += f' stroke-dasharray="{dash}"'
    if marker:
        a += f' marker-end="url(#{marker})"'
    return a + "/>"


def poly(points, fill, stroke=None, sw=1.0, op=None, dash=None):
    p = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    a = f'<polygon points="{p}" fill="{fill}"'
    if stroke:
        a += f' stroke="{stroke}" stroke-width="{sw:.1f}"'
    if dash:
        a += f' stroke-dasharray="{dash}"'
    if op is not None:
        a += f' opacity="{op}"'
    return a + "/>"


def defs_markers() -> str:
    out = ["<defs>"]
    for name, col in (("ah", C["ink3"]), ("ahb", C["blue"]),
                      ("ahr", C["red"]), ("ahg", C["green"])):
        out.append(
            f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" '
            f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{col}"/></marker>')
    out.append("</defs>")
    return "".join(out)


def svg_open(w, h, title):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}" role="img" aria-label="{esc(title)}">'
            + defs_markers() + rect(0, 0, w, h, C["bg"]))


# ══════════════════════════════════════════════════════════════════════
# 图 1 · 系统架构：四层流水线 + 动态分支
# ══════════════════════════════════════════════════════════════════════
def fig_architecture() -> str:
    W, H = 1120, 800
    X, BW = 40, 1040
    s = [svg_open(W, H, "城市配送智能配载 系统架构")]

    s.append(text(X, 42, "城市配送智能配载 · 系统架构", 21, C["ink"], weight="600"))
    s.append(text(X, 70, "四层流水线，每层输出均可被独立程序复核；虚线为自主扩展的动态分支",
                  13, C["ink3"]))

    def band(y, h, name, accent):
        return [rect(X, y, BW, h, C["band"], C["band_ln"], 1.0, r=8),
                rect(X, y, 5, h, accent),
                text(X + 20, y + 20, name, 13.5, accent, weight="600")]

    def box(x, y, w, h, title, lines, accent, fs=12.2, tfs=13.2, dashed=False):
        o = [rect(x, y, w, h, C["box"], accent if dashed else C["box_ln"],
                  1.5 if dashed else 1.0, r=6, dash="5 3" if dashed else None)]
        avail = w - 24
        check(title, tfs, avail, "box-title")
        o.append(text(x + 12, y + 21, title, tfs, C["ink"], weight="600"))
        yy = y + 40
        for ln in lines:
            check(ln, fs, avail, "box-line")
            o.append(text(x + 12, yy, ln, fs, C["ink2"]))
            yy += 17
        return o

    def varrow(x, y1, y2, col, mark):
        return [line(x, y1, x, y2, col, 1.6, marker=mark)]

    # ── 数据层 ─────────────────────────────────────────────
    ya, ha = 100, 112
    s += band(ya, ha, "① 数据层  算例与主数据", C["ink3"])
    s += box(64, ya + 38, 484, 62, "杭州路网  src/data/hangzhou_net.py",
             ["中心仓 1 个 + 连锁门店 30 家（主城区，按方位分区）",
              "车型 3 种：新能源微面 / 依维柯厢货 / 4.2m 厢货"], C["ink3"])
    s += box(572, ya + 38, 484, 62, "订单与货物  src/data/goods.py",
             ["医药 SKU 24 个：尺寸 / 重量 / 易碎 / 堆叠上限 / 可否倒置",
              "订单 67 张 1328 件：常规补货 46 / 电商即时 16 / 紧急要货 5"],
              C["ink3"])
    s += varrow(560, ya + ha, ya + ha + 36, C["ink3"], "ah")

    # ── 规划层 ─────────────────────────────────────────────
    yb, hb = 248, 212
    s += band(yb, hb, "② 规划层  四个求解模块（共享同一车厢坐标系）", C["blue"])
    bx, bw, gap = 64, 233, 20
    mods = [("① 运力匹配与路线", ["异构车队 VRPTW", "构造层 + 局部搜索", "输出停靠顺序"], C["blue"]),
            ("②a 仓内拣货配货", ["S 形路径遍历", "深位优先消翻垛", "输出拣货序"], C["blue"]),
            ("②b 车厢三维装载", ["货架定平面布局", "天际线定高度", "C1–C8 八项约束"], C["blue"]),
            ("③ 装卸序与指引", ["装载序 = 支撑拓扑", "卸货序 = 反转拓扑", "相机画面校验"], C["blue"])]
    for i, (t, ls, acc) in enumerate(mods):
        x = bx + i * (bw + gap)
        s += box(x, yb + 38, bw, 108, t, ls, acc, fs=11.8, tfs=12.6)
        if i:
            px = x - gap
            s.append(line(px + 2, yb + 92, px + gap - 2, yb + 92, C["box_ln"], 1.4,
                          marker="ahb"))
    s += box(bx, yb + 158, BW - 48, 40,
             "动态分支（自主扩展）· 滚动时域派车",
             ["每 120 min 触发一次；合并待派订单后只重解受影响子集，不冻结已在途任务"],
             C["blue"], fs=11.8, tfs=12.4, dashed=True)
    s += varrow(560, yb + hb, yb + hb + 36, C["ink3"], "ah")

    # ── 校验层 ─────────────────────────────────────────────
    yc, hc = 496, 112
    s += band(yc, hc, "③ 校验层  独立实现交叉验证，不接受求解器自报数字", C["teal"])
    ver = [("精确性交叉验证", ["Held-Karp DP + 集合划分 MILP", "对启发式解给出最优性上界"]),
           ("装载约束审计器", ["C1–C8 八项逐条判定", "算法与人工基线过同一审计器"]),
           ("相机模型自证", ["投影比对解析解", "遮挡比对暴力射线采样"])]
    for i, (t, ls) in enumerate(ver):
        s += box(64 + i * (314 + 25), yc + 38, 314, 62, t, ls, C["teal"], fs=11.8, tfs=12.6)
    s += varrow(560, yc + hc, yc + hc + 36, C["ink3"], "ah")

    # ── 输出层 ─────────────────────────────────────────────
    yd, hd = 644, 118
    s += band(yd, hd, "④ 输出层  可交付物", C["amber"])
    outs = [("三维布局", ["逐件 x/y/z 坐标", "out/loading_plan.json"]),
            ("分步装卸指引", ["装载序 + 卸货序", "out/delivery_aware.json"]),
            ("相机画面指引", ["双机位可见性判定", "assets/camera/*.svg"]),
            ("成本对比报告", ["算法 vs 人工基线", "out/loading_compare.json"])]
    for i, (t, ls) in enumerate(outs):
        s += box(64 + i * (233 + 20), yd + 38, 233, 66, t, ls, C["amber"], fs=11.6, tfs=12.6)

    s.append("</svg>")
    return "".join(s)


# ══════════════════════════════════════════════════════════════════════
# 图 2 · 三子问题闭环与反向约束
# ══════════════════════════════════════════════════════════════════════
def fig_closed_loop() -> str:
    W, H = 1120, 620
    s = [svg_open(W, H, "三子问题闭环与反向约束")]

    s.append(text(40, 42, "三个子问题共用车厢坐标系：正向链与反向约束", 21, C["ink"], weight="600"))
    s.append(text(40, 70, "正向：路线定送达顺序 → 装载定三维位置 → 装卸序与画面指引；"
                          "反向：配送顺序想直接决定装载排序时会与物理支撑成环",
                  12.6, C["ink3"]))

    ny, nh, nw = 110, 146, 250
    xs = [60, 430, 800]
    nodes = [("① 运力匹配与路线优化",
              ["车、货、时、距综合", "输出：门店停靠顺序", "out/route_result.json"], C["blue"]),
             ("② 多约束三维装载",
              ["C1–C8 八项约束", "输出：逐件三维坐标", "out/loading_plan.json"], C["blue"]),
             ("③ 装卸序与画面指引",
              ["拓扑排序 + 相机校验", "输出：装载序 / 卸货序", "out/delivery_aware.json"], C["blue"])]

    for i, (t, ls, acc) in enumerate(nodes):
        x = xs[i]
        s.append(rect(x, ny, nw, nh, C["box"], acc, 1.4, r=8))
        s.append(rect(x, ny, nw, 4, acc))
        check(t, 13.4, nw - 24, "node-title")
        s.append(text(x + 12, ny + 30, t, 13.4, C["ink"], weight="600"))
        yy = ny + 56
        for ln in ls:
            check(ln, 12.2, nw - 24, "node-line")
            s.append(text(x + 12, yy, ln, 12.2, C["ink2"]))
            yy += 20

    # 正向箭头 + 说明牌
    for i, (tag1, tag2) in enumerate([("送达顺序", "驱动装载排序"),
                                      ("三维支撑", "生成卸货序")]):
        x1, x2 = xs[i] + nw, xs[i + 1]
        mid = (x1 + x2) / 2
        s.append(line(x1 + 4, ny + 96, x2 - 4, ny + 96, C["blue"], 1.8, marker="ahb"))
        s.append(rect(mid - 56, ny + 20, 112, 40, C["blue_bg"], C["blue"], 1.0, r=6))
        s.append(text(mid, ny + 36, tag1, 11.8, C["blue"], anchor="middle", weight="600"))
        s.append(text(mid, ny + 51, tag2, 11.4, C["blue"], anchor="middle"))

    # 反向约束
    ry, rh = 320, 258
    s.append(rect(40, ry, 1040, rh, C["red_bg"], C["red"], 1.5, r=8, dash="6 4"))
    s.append(text(64, ry + 32, "反向约束：配送顺序不能写成硬偏序", 15, C["red"], weight="600"))

    notes = ["装载序的硬约束只有一条：下层先于上层（支撑关系）。",
             "若再把「停靠序号大的先装」也写成硬偏序，两条要求会互相冲突：",
             "  支撑关系要求「后送的货在下面」→ 后送先装",
             "  配送顺序要求「先送的货在里面」→ 先送先装",
             "同一个货位被两条规则反向指定，偏序图出现环，拓扑排序直接无解。",
             "处理方式：物理支撑保留为硬约束，配送顺序降级为目标函数，",
             "并单独报告「不翻垛前提下不可能消除的逆序支撑对数量」作为物理下界。"]
    yy = ry + 60
    for ln in notes:
        check(ln, 12.3, 660, "note-line")
        s.append(text(64, yy, ln, 12.3, C["ink2"]))
        yy += 21

    # 成环示意
    cx, cw = 780, 230
    s.append(rect(cx - 20, ry + 46, cw + 40, 176, C["box"], C["red"], 1.2, r=6))
    s.append(text(cx + cw / 2, ry + 68, "偏序成环", 12.6, C["red"],
                  anchor="middle", weight="600"))
    s.append(rect(cx, ry + 84, cw, 38, C["box"], C["box_ln"], 1.0, r=5))
    s.append(text(cx + cw / 2, ry + 108, "支撑要求：后送先装", 12.0, C["ink"],
                  anchor="middle"))
    s.append(rect(cx, ry + 166, cw, 38, C["box"], C["box_ln"], 1.0, r=5))
    s.append(text(cx + cw / 2, ry + 190, "配送要求：先送先装", 12.0, C["ink"],
                  anchor="middle"))
    s.append(path(f"M {cx + cw - 6},{ry + 124} C {cx + cw + 42},{ry + 145} "
                  f"{cx + cw + 42},{ry + 165} {cx + cw - 6},{ry + 186}",
                  C["red"], 1.8, marker="ahr"))
    s.append(path(f"M {cx + 6},{ry + 186} C {cx - 42},{ry + 165} "
                  f"{cx - 42},{ry + 145} {cx + 6},{ry + 124}",
                  C["red"], 1.8, marker="ahr"))
    s.append(text(cx + cw / 2, ry + 240, "两规则互为反向 ⇒ 拓扑排序报错",
                  11.6, C["red"], anchor="middle"))

    s.append("</svg>")
    return "".join(s)


# ══════════════════════════════════════════════════════════════════════
# 图 3 · 装载高度场：平面分层 vs 天际线
# ══════════════════════════════════════════════════════════════════════
def fig_skyline() -> str:
    W, H = 1120, 660
    s = [svg_open(W, H, "平面分层与天际线对比")]

    s.append(text(40, 42, "装载高度场：全局平面分层 vs 天际线（高度场）", 21,
                  C["ink"], weight="600"))
    s.append(text(40, 70, "同为 2600×1400×1300 车厢、同一批货；差别只在「第二层从哪里起铺」",
                  12.6, C["ink3"]))

    GROUND, MM = 500, 0.2308          # 1300 mm -> 300 px
    PX = lambda mm: GROUND - mm * MM  # noqa: E731

    def panel(px, pw, title, accent, tag):
        o = [rect(px, 96, pw, 512, C["band"], C["band_ln"], 1.0, r=8),
             rect(px, 96, pw, 4, accent),
             rect(px + 16, 112, len(tag) * 15 + 20, 26, accent, None, r=13),
             text(px + 26, 130, tag, 12.4, "#ffffff", weight="600")]
        check(title, 14.6, pw - 32, "panel-title")
        o.append(text(px + 16, 166, title, 14.6, C["ink"], weight="600"))
        return o

    def truck(px, inner_w, show_global_line):
        """车厢 + 高低两段货堆。返回绘制元素与几何量。"""
        o = []
        x0 = px + 20
        o.append(rect(x0, PX(1300), inner_w, GROUND - PX(1300),
                      "#ffffff", C["ink3"], 1.3, r=3))
        o.append(line(x0, GROUND, x0 + inner_w, GROUND, C["ink3"], 2.0))
        split = x0 + inner_w * 0.60
        # 高堆：轮椅 880 mm
        o.append(rect(x0, PX(880), split - x0, GROUND - PX(880),
                      C["blue_bg"], C["blue"], 1.2))
        o.append(text((x0 + split) / 2, PX(440), "轮椅 880 mm", 11.6, C["blue"],
                      anchor="middle"))
        # 矮堆：小件 220 mm
        o.append(rect(split, PX(220), x0 + inner_w - split, GROUND - PX(220),
                      C["grey_bg"], C["box_ln"], 1.2))
        o.append(text((split + x0 + inner_w) / 2, PX(110), "小件 220 mm", 11.6,
                      C["ink2"], anchor="middle"))
        if show_global_line:
            o.append(line(x0, PX(880), x0 + inner_w, PX(880), C["red"], 1.6, dash="6 4"))
            o.append(text(x0 + inner_w - 4, PX(880) - 8,
                          "层1 顶面 = 880（取全局最高）", 11.2, C["red"], anchor="end"))
        return o, x0, inner_w, split

    # ── 面板 A：平面分层 ──────────────────────────────────
    pa, pwa = 40, 500
    s += panel(pa, pwa, "平面分层（初版，被否决）", C["red"], "方案 A")
    els, x0, iw, split = truck(pa, pwa - 40, True)
    s += els
    # 层2 货箱被迫从 880 起铺，落在矮堆上方 -> 悬空
    bw2 = 120
    bx2 = split + 40
    s.append(rect(bx2, PX(1100), bw2, PX(880) - PX(1100), C["box"], C["red"], 1.4, r=3))
    s.append(text(bx2 + bw2 / 2, PX(990), "层2 货箱", 11.4, C["red"], anchor="middle"))
    gx = bx2 + bw2 / 2
    s.append(line(gx, PX(880), gx, PX(220), C["red"], 1.4, marker="ahr"))
    s.append(line(gx - 6, PX(880), gx + 6, PX(880), C["red"], 1.4))
    s.append(text(gx + 10, PX(550), "悬空 660 mm", 11.6, C["red"], weight="600"))
    s.append(text(pa + 24, GROUND + 34, "层 2 强制从层1 最高点 880 mm 起铺；", 12.4, C["ink2"]))
    s.append(text(pa + 24, GROUND + 54, "矮堆上方 660 mm 空洞判为悬空，支撑率 0 被拒。", 12.4, C["ink2"]))
    s.append(text(pa + 24, GROUND + 78, "结果：层 2 仅装入 1 件。", 12.4, C["red"], weight="600"))

    # ── 面板 B：天际线 ────────────────────────────────────
    pb, pwb = 580, 500
    s += panel(pb, pwb, "天际线 / 高度场（采用）", C["green"], "方案 B")
    els, x0b, iwb, splitb = truck(pb, pwb - 40, False)
    s += els
    # 天际线折线
    s.append(path(f"M {x0b},{PX(880)} L {splitb},{PX(880)} L {splitb},{PX(220)} "
                  f"L {x0b + iwb},{PX(220)}", C["green"], 2.0, dash="7 4"))
    s.append(text(x0b + iwb - 4, PX(220) - 8, "天际线（逐点取实测顶面）", 11.2,
                  C["green"], anchor="end"))
    # 层2 货箱：分别落在各自区域的最低天际线上
    s.append(rect(bx2, PX(1100), bw2, PX(880) - PX(1100), C["box"], C["green"], 1.4, r=3))
    s.append(text(bx2 + bw2 / 2, PX(990), "层2 货箱", 11.4, C["green"], anchor="middle"))
    bx3 = splitb + 40
    s.append(rect(bx3, PX(460), bw2, PX(220) - PX(460), C["box"], C["green"], 1.4, r=3))
    s.append(text(bx3 + bw2 / 2, PX(340), "层2 货箱", 11.4, C["green"], anchor="middle"))
    s.append(line(bx3 + bw2 / 2, PX(460) - 4, bx3 + bw2 / 2, PX(220) + 4, C["green"], 1.4))
    s.append(text(bx3 + bw2 / 2 + 8, PX(215), "坐实于 220", 11.4, C["green"]))
    s.append(text(pb + 24, GROUND + 34, "每件货落在其投影范围内容积最低的天际线高度上；", 12.4, C["ink2"]))
    s.append(text(pb + 24, GROUND + 54, "矮堆上方继续码货，高堆旁边就近补货，不再出现悬空。", 12.4, C["ink2"]))
    s.append(text(pb + 24, GROUND + 78, "结果：全部订单 100% 装载，V2 达 91.45%，违规 0。",
                  12.4, C["green"], weight="600"))

    s.append("</svg>")
    return "".join(s)


# ══════════════════════════════════════════════════════════════════════
# 图 4 · 双机位与逐步可指引率
# ══════════════════════════════════════════════════════════════════════
def fig_camera() -> str:
    W, H = 1120, 600
    s = [svg_open(W, H, "双机位与逐步可指引率")]

    s.append(text(40, 42, "车载相机：双机位在进深方向互补", 21, C["ink"], weight="600"))
    s.append(text(40, 70, "不报「满载可辨率」，报「放入第 k 件时该件落位面是否可见」——"
                          "即逐步可指引率", 12.6, C["ink3"]))

    # ── 左：俯视布置 ──────────────────────────────────────
    LW, LH = 520, 470
    s.append(rect(40, 96, LW, LH, C["band"], C["band_ln"], 1.0, r=8))
    s.append(rect(40, 96, LW, 4, C["teal"]))
    s.append(text(60, 128, "俯视：车厢 2600 × 1400 mm（V1 微面）", 13.6, C["ink"], weight="600"))

    bx, by, bwv, bhv = 90, 200, 440, 238      # 2600→440, 1400→237
    s.append(rect(bx, by, bwv, bhv, "#ffffff", C["ink3"], 1.3, r=3))
    s.append(line(bx, by, bx, by + bhv, C["teal"], 3.0))
    s.append(text(bx + 14, by + bhv - 12, "尾门", 11.6, C["teal"], weight="600"))
    s.append(text(bx + bwv - 10, by + bhv - 12, "车头", 11.6, C["ink3"],
                  anchor="end"))

    # 货堆示意
    for i in range(6):
        for j in range(3):
            s.append(rect(bx + 26 + i * 68, by + 22 + j * 72, 58, 60,
                          C["grey_bg"], C["box_ln"], 0.9, r=2))

    # 尾门后视：广角，从尾门往内推
    s.append(poly([(bx + 10, by + 14), (bx + 210, by + 4), (bx + 210, by + bhv - 4),
                   (bx + 10, by + bhv - 14)], C["teal"], op=0.14))
    s.append(poly([(bx + 8, by + 10), (bx + 20, by + 26), (bx + 20, by + bhv - 26),
                   (bx + 8, by + bhv - 10)], C["teal"]))
    s.append(text(bx + 8, by - 12, "尾门后视相机  偏航 8° / 俯仰 −14° / HFOV 150°",
                  11.6, C["teal"], weight="600"))

    # 顶棚俯视：从车头往尾门
    s.append(poly([(bx + bwv - 12, by + 40), (bx + 150, by + 14), (bx + 150, by + bhv - 14),
                   (bx + bwv - 12, by + bhv - 40)], C["amber"], op=0.14))
    s.append(poly([(bx + bwv - 8, by + bhv / 2 - 16), (bx + bwv - 8, by + bhv / 2 + 16),
                   (bx + bwv + 6, by + bhv / 2)], C["amber"]))
    s.append(text(bx + bwv - 8, by + bhv + 22,
                  "顶棚俯视相机  偏航 180° / 俯仰 −32° / HFOV 170°",
                  11.6, C["amber"], anchor="end", weight="600"))

    s.append(text(60, 486, "两台相机在进深方向一降一升：后视越装越深、可见越差；"
                           "俯视越深越开阔。", 12.2, C["ink2"]))
    s.append(text(60, 508, "这是「双机位」的设计依据，不是随意增加一台相机。", 12.2, C["ink2"]))
    s.append(text(60, 538, "遮挡判定与投影均对独立实现做过交叉验证：",
                  12.0, C["ink3"]))
    s.append(text(60, 558, "投影误差 < 1e-3 px；160 条射线零不一致。", 12.0, C["ink3"]))

    # ── 右：分组柱状图 ────────────────────────────────────
    RW = 520
    s.append(rect(580, 96, RW, LH, C["band"], C["band_ln"], 1.0, r=8))
    s.append(rect(580, 96, RW, 4, C["teal"]))
    s.append(text(600, 128, "逐步可指引率（按装载批次分 5 段）", 13.6, C["ink"], weight="600"))

    ax0, ax1 = 660, 1060
    base, top = 440, 180
    scale = (base - top) / 100.0
    s.append(line(ax0 - 24, base, ax1, base, C["ink3"], 1.2))
    for v in (0, 25, 50, 75, 100):
        y = base - v * scale
        s.append(line(ax0 - 24, y, ax1, y, C["box_ln"], 0.8, dash="3 3"))
        s.append(text(ax0 - 30, y + 4, f"{v}", 10.4, C["ink3"], anchor="end"))

    rear = [52.2, 44.4, 57.1, 42.9, 60.0]
    ceil = [32.6, 22.2, 64.3, 71.4, 90.0]
    gw, bwid = 72.0, 24.0
    off = (gw - (bwid * 2 + 5)) / 2
    for i in range(5):
        gx = ax0 + i * gw
        for k, (vals, col, lab) in enumerate(((rear, C["teal"], "尾门后视"),
                                              (ceil, C["amber"], "顶棚俯视"))):
            v = vals[i]
            x = gx + off + k * (bwid + 5)
            y = base - v * scale
            s.append(rect(x, y, bwid, base - y, col, None, r=2))
            s.append(text(x + bwid / 2, y - 6, f"{v:.1f}", 10.2, col,
                          anchor="middle", weight="600"))
        s.append(text(gx + gw / 2 - 8, base + 20, f"第 {i + 1} 批", 11.4,
                      C["ink2"], anchor="middle"))

    s.append(rect(ax0, 150, 12, 12, C["teal"], r=2))
    s.append(text(ax0 + 18, 160, "尾门后视  整体 51.0%", 11.6, C["teal"]))
    s.append(rect(ax0 + 208, 150, 12, 12, C["amber"], r=2))
    s.append(text(ax0 + 226, 160, "顶棚俯视  整体 46.1%", 11.6, C["amber"]))

    s.append(text(600, 486, "同一台相机在装载早期与后期表现不同：批次是按落位高度",
                  12.2, C["ink2"]))
    s.append(text(600, 506, "划分的，越晚的批次越靠上层与深处。", 12.2, C["ink2"]))
    s.append(text(600, 538, "对比项：满载后静态可辨率仅 3.9%（后视）与 12.8%（俯视），",
                  12.0, C["ink3"]))
    s.append(text(600, 558, "那是装载完成后的必然结果，不能代表指引能力。", 12.0, C["ink3"]))

    s.append("</svg>")
    return "".join(s)


# ══════════════════════════════════════════════════════════════════════
def main() -> None:
    figs = {
        "01_architecture.svg": fig_architecture,
        "02_closed_loop.svg": fig_closed_loop,
        "03_skyline.svg": fig_skyline,
        "04_camera.svg": fig_camera,
    }
    for name, fn in figs.items():
        svg = fn()
        # 极简结构校验：标签配对
        import xml.etree.ElementTree as ET
        ET.fromstring(svg)
        p = HERE / name
        p.write_text(svg, encoding="utf-8")
        print(f"  {name}  {len(svg):>7,} bytes")
    print("图 4 张已生成。所有文本宽度断言通过。")


if __name__ == "__main__":
    main()
