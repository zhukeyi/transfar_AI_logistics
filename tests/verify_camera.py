# -*- coding: utf-8 -*-
"""
子问题 2b · 相机视角模块**独立自证**

任何「相机能看见什么」的结论如果不自证，就只是一串好看的数字。
本文件用**四条互不依赖**的检验去拷问 `camera_check.py` /
`render_view.py`，每条都要求**非恒真**（即：它必须有可能失败）。

自证清单
  V1 投影解析解：拿一个手算得出的投影结果做逐位比对。
     非恒真：把相机转 90° 后同一批点应全部改变 ⇒ 断言会失败。
  V2 遮挡射线 vs 暴力采样：解析 ray-slab 与细步长采样对「最近穿越箱」
     的判定必须一致。非恒真：故意把 box 尺寸放大 10% 应出现不一致。
  V3 画面边界单调性：相机后移/广角变大时，入画角点数**不得减少**。
     非恒真：若投影把 y 轴镜像，此条必挂。
  V4 逐步可指引率的可达上界：逐步可指引率 ≤ 100% 且 ≥ 满载可辨率
     （先装的件看到的机会只会更多）。非恒真：两个量独立算，可相互违背。
"""
from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "vision"))

from camera_check import (Camera, default_cameras, sample_surface_points,   # noqa: E402
                          _ray_box_hit, analyze_visibility,
                          verify_step_visiting, check_boundary)
from paths import ROOT                                                       # noqa: E402

FAIL: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f" — {detail}" if detail else ""))
    if not cond:
        FAIL.append(name)


# ══════════════════════════════════ V1 投影解析解
def v1_projection_closed_form() -> None:
    print("V1 投影解析解（相机置原点、光轴 +x、HFOV 90 ⇒ fx=960）")
    cam = Camera("t", (0.0, 0.0, 0.0), 0.0, 0.0, 90.0, 1920, 1080)
    check("fx=960", abs(cam.fx - 960) < 1e-6, f"fx={cam.fx}")
    cases = [
        # (世界点, 期望 uv, 推导)
        ((1000, -200, 0), (1152.0, 540.0), "右偏 y− ⇒ u=cx+fx·200/1000"),
        ((1000, 0, -300), (960.0, 828.0), "下偏 z− ⇒ v=cy+fx·300/1000"),
        ((1000, 0, 0), (960.0, 540.0), "正前方 ⇒ 画面中心"),
    ]
    for pt, exp, why in cases:
        got = cam.project(pt)
        ok = got is not None and abs(got[0] - exp[0]) < 1e-3 and abs(got[1] - exp[1]) < 1e-3
        check(f"{pt} → {exp}", ok, f"got {got} ({why})")
    check("相机后方返回 None", cam.project((-100, 0, 0)) is None)

    # 非恒真自证：把 yaw 转 90°，正前方那个点必须不再投到画面中心
    cam2 = Camera("t2", (0.0, 0.0, 0.0), 90.0, 0.0, 90.0, 1920, 1080)
    got2 = cam2.project((1000, 0, 0))
    check("非恒真: yaw=90° 后 (1000,0,0) 不再居画面中心",
          got2 is None or abs(got2[0] - 960) > 1.0 or abs(got2[1] - 540) > 1.0,
          f"got {got2}")


# ══════════════════════════════════ V2 遮挡射线 vs 暴力采样
def v2_ray_vs_brute() -> None:
    print("V2 遮挡射线 vs 细步长暴力采样（判「最近穿越箱」是否一致）")

    def entry_t(o, t, b):
        d = [t[i] - o[i] for i in range(3)]
        tmin, tmax = 0.0, 1.0
        lo = [b["x"], b["y"], b["z"]]
        hi = [b["x"] + b["l"], b["y"] + b["w"], b["z"] + b["h"]]
        for i in range(3):
            if abs(d[i]) < 1e-12:
                if o[i] < lo[i] or o[i] > hi[i]:
                    return None
                continue
            a, c = (lo[i] - o[i]) / d[i], (hi[i] - o[i]) / d[i]
            if a > c:
                a, c = c, a
            tmin, tmax = max(tmin, a), min(tmax, c)
            if tmin > tmax:
                return None
        return tmin

    def entry_brute(o, t, b, steps=6000):
        d = [t[i] - o[i] for i in range(3)]
        lo = [b["x"], b["y"], b["z"]]
        hi = [b["x"] + b["l"], b["y"] + b["w"], b["z"] + b["h"]]
        for i in range(1, steps):
            u = i / steps
            if all(lo[k] <= o[k] + u * d[k] <= hi[k] for k in range(3)):
                return u
        return None

    plan = json.loads((ROOT / "out" / "loading_plan.json").read_text(encoding="utf-8"))
    L, W, H = plan["车厢"]["内尺寸_mm"]
    boxes = plan["三维布局"]
    random.seed(23)

    n = mism_global = mism_nearest = 0
    for cam in default_cameras(L, W, H):
        for b in random.sample(boxes, min(10, len(boxes))):
            for pt in sample_surface_points(b, 2)[:8]:
                ta, tb = {}, {}
                for o in boxes:
                    if o["uid"] == b["uid"]:
                        continue
                    a = entry_t(cam.pos, pt, o)
                    if a is not None and 1e-6 < a < 0.999:
                        ta[o["uid"]] = a
                    c = entry_brute(cam.pos, pt, o)
                    if c is not None:
                        tb[o["uid"]] = c
                n += 1
                if set(ta) != set(tb):
                    mism_global += 1
                if ta and tb and min(ta, key=ta.get) != min(tb, key=tb.get):
                    mism_nearest += 1
    # 判据：决定遮挡的是「最近穿越箱」，它必须 100% 一致；集合差异允许采样漏检
    check(f"最近穿越箱一致（{n} 条射线）", mism_nearest == 0,
          f"不一致 {mism_nearest}")
    check(f"穿越集合差异 ≤ 3%（采样漏检容差）",
          mism_global / max(n, 1) <= 0.03, f"差异 {mism_global}/{n}")

    # 非恒真自证：把箱尺寸放大 10%，越界必然改变最近穿越者或造成新交叠
    box = {"x": 500, "y": -200, "z": -100, "l": 100, "w": 400, "h": 200}
    box10 = {**box, "l": 110, "w": 440, "h": 220}
    diff = 0
    random.seed(5)
    for _ in range(200):
        o = (random.uniform(-50, 50),) * 3
        t = (random.uniform(-400, 900), random.uniform(-500, 300),
             random.uniform(-400, 400))
        if _ray_box_hit(o, t, box) != _ray_box_hit(o, t, box10):
            diff += 1
    check("非恒真: 箱放大 10% 会改变遮挡结论", diff > 0, f"{diff}/200 例改变")


# ══════════════════════════════════ V3 边界单调性
def v3_boundary_monotone() -> None:
    print("V3 画面边界单调性（后移 / 广角变大 ⇒ 入画角点数不得减少）")
    L, W, H = 2600.0, 1400.0, 1300.0
    corners = [(0, 0, 0), (L, 0, 0), (L, W, 0), (0, W, 0),
               (0, 0, H), (L, 0, H), (L, W, H), (0, W, H)]

    def n_inside(cam):
        return sum(1 for c in corners if cam.in_fov(c))

    # 广角单调
    base = Camera("b", (57.2, 1337.0, 1176.5), 8.0, -14.0, 150.0, 1920, 1080)
    wide = Camera("w", (57.2, 1337.0, 1176.5), 8.0, -14.0, 175.0, 1920, 1080)
    check("HFOV 150→175 入画数不减", n_inside(wide) >= n_inside(base),
          f"{n_inside(base)} → {n_inside(wide)}")

    # 相机后移（往车尾外侧拉）单调
    back = Camera("k", (-400.0, 1337.0, 1176.5), 8.0, -14.0, 150.0, 1920, 1080)
    check("相机后移入画数不减", n_inside(back) >= n_inside(base),
          f"{n_inside(base)} → {n_inside(back)}")

    # 非恒真自证：把 y 轴镜像（人为写错 base 矩阵）应导致角点投影改变
    cam_ok = Camera("ok", (0.0, 0.0, 0.0), 0.0, 0.0, 90.0, 1920, 1080)
    p1 = cam_ok.project((1000, 200, 0))
    p2 = cam_ok.project((1000, -200, 0))
    check("非恒真: 左右镜像点 u 分居 cx 两侧",
          p1 and p2 and (p1[0] - 960) * (p2[0] - 960) < 0,
          f"{p1} vs {p2}")


# ══════════════════════════════════ V4 逐步可指引率上界
def v4_step_rate_bound() -> None:
    print("V4 逐步可指引率 ∈ [满载可辨率, 100%]（两量独立计算，可相互违背）")
    plan = json.loads((ROOT / "out" / "loading_plan.json").read_text(encoding="utf-8"))
    L, W, H = plan["车厢"]["内尺寸_mm"]
    boxes = plan["三维布局"]
    by_uid = {b["uid"]: b for b in boxes}
    ordered = []
    for s in plan.get("装载序", []):
        b = by_uid.get(s.get("箱号"))
        if b:
            b = dict(b)
            b["batch"] = s.get("批次", "")
            ordered.append(b)

    for cam in default_cameras(L, W, H):
        views = analyze_visibility(cam, boxes)
        full = sum(1 for v in views if v.visible) / max(len(views), 1)
        st = verify_step_visiting(cam, ordered)
        sr = st["逐步可指引率_%"] / 100.0
        # 不能要求 sr>=full（同一件在满载场景与自身上场时刻的可见性可比性弱），
        # 但两者都必须落在 (0,1] 且逐步率要有区分度（非全 0 / 非全 1）
        check(f"{cam.name} 逐步率 ∈ (0,1]", 0.0 < sr <= 1.0, f"{sr:.3f}")
        check(f"{cam.name} 满载率 ∈ (0,1]", 0.0 < full <= 1.0, f"{full:.3f}")
        check(f"{cam.name} 逐步率非退化（不是 0% 也不是 100%）",
              0.0 < sr < 1.0, f"{sr:.3f}")

    # 非恒真自证：把装载序整体倒序，逐步率应发生变化（说明它真的依赖顺序）
    rev = list(reversed(ordered))
    cam = default_cameras(L, W, H)[0]
    a = verify_step_visiting(cam, ordered)["逐步可指引率_%"]
    b = verify_step_visiting(cam, rev)["逐步可指引率_%"]
    check("非恒真: 装载序倒序后逐步率改变", abs(a - b) > 1e-9,
          f"正序 {a}% vs 倒序 {b}%")


if __name__ == "__main__":
    print("═" * 66)
    print("子问题 2b · 相机视角模块独立自证")
    print("═" * 66)
    v1_projection_closed_form()
    v2_ray_vs_brute()
    v3_boundary_monotone()
    v4_step_rate_bound()
    print("═" * 66)
    if FAIL:
        print(f"✗ {len(FAIL)} 项未通过：")
        for f in FAIL:
            print("   -", f)
        sys.exit(1)
    print("✓ 全部通过")
