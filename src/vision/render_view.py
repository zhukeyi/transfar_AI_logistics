# -*- coding: utf-8 -*-
"""
子问题 2b 重点 · 相机视角**渲染**（把装载方案画成相机看到的画面）

════════════════════════════════════════════════════════════════════════
一、为什么要渲染
════════════════════════════════════════════════════════════════════════
`camera_check.py` 输出的是**数字**（可辨件数、逐步可指引率）。但「摄像头
能不能当作业指引」本质是个**视觉**问题 —— 评委/客户需要看到画面本身：
    · 车厢内框在不在画面里？
    · 货堆有没有超宽、超高、挡住通道？
    · 「下一件该放这里」能不能用画面框表达出来？

因此本模块把三维装载方案**投影成图像**（SVG），得到「车载相机所见」。

════════════════════════════════════════════════════════════════════════
二、渲染方法
════════════════════════════════════════════════════════════════════════
不走 z-buffer 光栅化（车载场景箱体数量少，没必要），改用**画家算法**：
  1. 把每件货的 6 个面拆成四边形，面法向背对相机者剔除（背面剔除）；
  2. 对所有可见面按「面心到相机距离」**从远到近**排序；
  3. 由远及近依次绘制 ⇒ 近处自然覆盖远处，得到正确遮挡关系。

★ 与 `camera_check.py` 的**独立性声明**：
   两者都是「相机外参 + 面投影」，但**遮挡判定算法完全不同**
   （这里是排序绘制，那里是射线求交）。若两者结论一致，
   说明遮挡关系不是某一种算法的产物 —— 这构成一次有效的交叉验证。

★ 数据声明：相机参数与货位为**合成示例数据**，画面不代表任何真实仓库的
  现场拍摄结果。渲染图为**方案示意**，不含实拍成分。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from camera_check import Camera, default_cameras   # noqa: E402


# ══════════════════════════════════════════════ 颜色（按品类分组）
def _hue_for(sku_id: str) -> float:
    """由 SKU 稳定映射到色相（同一品类恒同色，便于肉眼归类）。"""
    n = sum(ord(c) for c in sku_id)
    return (n * 47 % 360)


def _box_color(sku_id: str, lit: float, fragile: int = 0) -> str:
    """
    给一个箱面配色。lit ∈ [0,1] 表示该面受光程度（用面法向与光向夹角算）。

    ★ 红=易碎（fragile=2），橙=需轻放（fragile=1），其余按品类色相。
    """
    if fragile >= 2:
        h = 0.0
    elif fragile == 1:
        h = 30.0
    else:
        h = _hue_for(sku_id)
    s = 0.42
    l = 0.34 + 0.36 * lit          # 明度随受光变化 ⇒ 立体感
    return f"hsl({h:.0f}, {s*100:.0f}%, {l*100:.0f}%)"


# ══════════════════════════════════════════════ 几何
_FACES = [
    # (顶点索引, 法向)  顶点按右手序（从外部看逆时针）
    ((0, 1, 3, 2), (-1, 0, 0)),   # x−
    ((4, 6, 7, 5), (+1, 0, 0)),   # x+
    ((0, 4, 5, 1), (0, -1, 0)),   # y−
    ((2, 3, 7, 6), (0, +1, 0)),   # y+
    ((0, 2, 6, 4), (0, 0, -1)),   # z−
    ((1, 5, 7, 3), (0, 0, +1)),   # z+
]


def _corners(b: dict) -> list[tuple[float, float, float]]:
    x0, y0, z0 = b["x"], b["y"], b["z"]
    x1, y1, z1 = x0 + b["l"], y0 + b["w"], z0 + b["h"]
    return [(x0, y0, z0), (x1, y0, z0), (x0, y1, z0), (x1, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x0, y1, z1), (x1, y1, z1)]


def _faces_of(b: dict, cam: Camera) -> list[tuple[float, list, str]]:
    """返回该件的可见面：[ (深度, 多边形像素点列, 填充色) ]。"""
    cs = _corners(b)
    out = []
    light = (-0.35, 0.45, -0.82)            # 光照方向（自车顶前上方来）
    for idx, nrm in _FACES:
        # 背面剔除：面法向是否朝向相机
        p0 = cs[idx[0]]
        ctr = [sum(cs[i][k] for i in idx) / 4 for k in range(3)]
        view = [cam.pos[k] - ctr[k] for k in range(3)]
        if sum(nrm[k] * view[k] for k in range(3)) <= 0:
            continue
        pts = []
        ok = True
        for i in idx:
            uv = cam.project(cs[i])
            if uv is None:
                ok = False
                break
            pts.append(uv)
        if not ok or len(pts) < 3:
            continue
        lit = max(0.0, sum(nrm[k] * light[k] for k in range(3)))
        dist = math.dist(cam.pos, tuple(ctr))
        out.append((dist, pts, _box_color(b.get("sku_id", ""), lit,
                                          int(b.get("fragile", 0) or 0))))
    return out


def _wire_of(cam: Camera, L: float, W: float, H: float) -> list[tuple[list, str, float]]:
    """车厢内框 12 条棱的投影（只画两端都在画面内的棱）。"""
    c = [(0, 0, 0), (L, 0, 0), (L, W, 0), (0, W, 0),
         (0, 0, H), (L, 0, H), (L, W, H), (0, W, H)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0),
             (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    out = []
    for a, b in edges:
        ua, ub = cam.project(c[a]), cam.project(c[b])
        if ua and ub:
            out.append(([ua, ub], "#4aa3ff", 2.0))
    return out


# ══════════════════════════════════════════════ SVG 输出
def render_camera_svg(cam: Camera, boxes: list[dict],
                      L: float, W: float, H: float,
                      title: str = "", highlight: str | None = None) -> str:
    """
    渲染单个相机的第一人称视图为 SVG。

    highlight: 若给定 uid，该件用高亮描边圈出（演示「画面指引」如何表达）。
    """
    VW, VH = 960, 540                    # 输出 SVG 尺寸（与画幅同比）
    sx, sy = VW / cam.res_w, VH / cam.res_h

    # 收集所有可见面，按深度排序（远处先画）
    allf = []
    for b in boxes:
        for f in _faces_of(b, cam):
            allf.append((f[0], f[1], f[2], b["uid"]))
    allf.sort(key=lambda t: -t[0])

    body = []
    # 车厢内框（先画，作为底）
    for pts, col, wd in _wire_of(cam, L, W, H):
        d = " ".join(f"{u*sx:.1f},{v*sy:.1f}" for u, v in pts)
        body.append(f'<polyline points="{d}" fill="none" stroke="{col}" '
                    f'stroke-width="{wd}" stroke-opacity="0.55"/>')

    for _, pts, col, uid in allf:
        d = " ".join(f"{u*sx:.1f},{v*sy:.1f}" for u, v in pts)
        extra = ""
        if highlight and uid == highlight:
            extra = (' stroke="#ffd447" stroke-width="3" '
                     'stroke-linejoin="round"')
        else:
            extra = ' stroke="rgba(0,0,0,0.35)" stroke-width="0.7"'
        body.append(f'<polygon points="{d}" fill="{col}"{extra}/>')

    # 高亮件的画面框 + 箭头提示
    if highlight:
        hb = next((b for b in boxes if b["uid"] == highlight), None)
        if hb:
            uv = [cam.project(p) for p in _corners(hb)]
            uv = [u for u in uv if u]
            if uv:
                x0 = min(u[0] for u in uv) * sx
                y0 = min(u[1] for u in uv) * sy
                x1 = max(u[0] for u in uv) * sx
                y1 = max(u[1] for u in uv) * sy
                body.append(f'<rect x="{x0-6:.1f}" y="{y0-6:.1f}" '
                            f'width="{x1-x0+12:.1f}" height="{y1-y0+12:.1f}" '
                            f'fill="none" stroke="#ffd447" stroke-width="3" '
                            f'stroke-dasharray="10 6" rx="4"/>')
                body.append(f'<text x="{x0-6:.1f}" y="{y0-14:.1f}" '
                            f'fill="#ffd447" font-size="16" '
                            f'font-family="sans-serif">'
                            f'下一件放这里 · {hb["uid"]}</text>')

    head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {VW} {VH}" '
            f'width="{VW}" height="{VH}">')
    bg = (f'<rect width="{VW}" height="{VH}" fill="#0d1218"/>')
    cap = (f'<text x="16" y="28" fill="#e8eef5" font-size="18" '
           f'font-family="sans-serif">{title}</text>')
    sub = (f'<text x="16" y="{VH-14}" fill="#7f8fa3" font-size="13" '
           f'font-family="sans-serif">安装 '
           f'{tuple(round(p) for p in cam.pos)} mm · 偏航 {cam.yaw:.0f}° · '
           f'俯仰 {cam.pitch:.0f}° · HFOV {cam.hfov_deg:.0f}° · '
           f'{cam.res_w}×{cam.res_h} · fx {cam.fx:.0f}px · 渲染为方案示意</text>')
    return head + bg + "".join(body) + cap + sub + "</svg>"


def render_all(plan_path: Path, out_dir: Path, highlight: str | None = None) -> list[Path]:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    L, W, H = plan["车厢"]["内尺寸_mm"]
    boxes = plan["三维布局"]
    # 补 fragile（渲染易碎用色）
    for b in boxes:
        b.setdefault("fragile", 0)
    cams = default_cameras(L, W, H)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for cam in cams:
        svg = render_camera_svg(
            cam, boxes, L, W, H,
            title=f'车载相机视图 · {cam.name} · {plan["车厢"]["名称"]}'
                  f'（{len(boxes)} 件）', highlight=highlight)
        p = out_dir / f"view_{'rear' if 'rear' in cam.name or '尾门' in cam.name else 'top'}.svg"
        p.write_text(svg, encoding="utf-8")
        paths.append(p)
    return paths


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import ROOT
    ps = render_all(ROOT / "out" / "loading_plan.json", ROOT / "assets" / "camera")
    for p in ps:
        print("写出", p, p.stat().st_size, "bytes")
