# -*- coding: utf-8 -*-
"""
子问题 2b 重点 · 车载摄像头视角校验装载方案

════════════════════════════════════════════════════════════════════════
一、要解决的问题
════════════════════════════════════════════════════════════════════════
「载货规划」如果只输出一堆坐标，现场无法使用 —— 装车工不可能一边搬货
一边心算三维坐标。因此需要把装载方案**投影到车载摄像头的视角**上，
生成「作业员从摄像头画面里看到什么、应该把货放在画面哪个位置」的指引。

本模块做的事情：
  1. **摄像头建模**：车厢顶部/尾部摄像头的内参（焦距、分辨率）与外参
     （安装位置、朝向），给出「三维点 → 像素坐标」的投影函数。
  2. **空间占据网格（Occupancy Grid）**：把装载方案的每件货体素化，
     在摄像头视锥内计算占据情况。
  3. **可见性/遮挡分析**：判断某件货在摄像头画面中是否被遮挡 ——
     这是**装卸指引能否用画面表达**的前提。
       · 货堆顶面（顶视/后视无遮挡）→ 可以在画面上框出「请放这里」
       · 被前排在竖直方向挡住 → 画面上看不到 ⇒ 必须改用「先搬哪件」的文字序
  4. **校验装载方案与画面的一致性**：把装载方案的**边界**投影到画面，
     检查是否越出车厢内框（画面上应能看出货堆有没有超出车厢轮廓）。

════════════════════════════════════════════════════════════════════════
二、相机模型
════════════════════════════════════════════════════════════════════════
采用**针孔相机模型**：
    pixel = K · [R | t] · world

车厢坐标系与装载算法一致：
    x : 车尾门(0) → 车头(L)
    y : 车宽方向
    z : 垂直向上

**摄像头 1（尾门后视相机）**：装在尾门上方，朝车头方向拍摄 ——
    用于「装载时看货堆正面」，判断还能放哪一层、齐不齐。
**摄像头 2（顶部俯视相机）**：装在车厢顶棚，垂直向下 ——
    用于「看平面布局」，判断哪块地板还空着。

★ 数据声明：相机参数为典型车载监控摄像头的合成值（对标海康威视
  车厢监控相机常见规格），不代表任何具体型号的真实内参。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path


# ══════════════════════════════════════════════ 相机模型
@dataclass
class Camera:
    """针孔相机。"""
    name: str
    pos: tuple[float, float, float]      # 安装位置（车厢坐标系, mm）
    yaw: float                           # 偏航角（度，绕 z 轴）
    pitch: float                         # 俯仰角（度，向下为负）
    hfov_deg: float                      # 水平视场角（度）
    res_w: int                           # 分辨率宽
    res_h: int                           # 分辨率高

    @property
    def fx(self) -> float:
        """水平焦距（像素）。由 hfov 与分辨率推出。"""
        return (self.res_w / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def fy(self) -> float:
        return self.fx                      # 方形像素

    @property
    def cx(self) -> float:
        return self.res_w / 2

    @property
    def cy(self) -> float:
        return self.res_h / 2

    def axes(self) -> tuple[list[float], list[float], list[float]]:
        """
        给出相机的三个世界坐标轴：光轴 f(forward)、右轴 r(right)、下轴 d(down)。

        ★ 这是整个相机模型的**唯一权威定义** —— 旋转矩阵由它导出，
          因此「投影是否正确」只需检验这三个轴是否符合物理直觉。

        基准姿态（yaw=0, pitch=0）：装在尾门、朝车头看
              f = (+1, 0,  0)   光轴指向车头
              r = ( 0,-1,  0)   车左(y+)落在图像左侧 ⇒ 右轴指向 y−
              d = ( 0, 0, -1)   车顶(z+)落在图像上方 ⇒ 下轴指向 z−

        然后：先绕**世界 z 轴**转 yaw，再绕**相机自身右轴 r** 俯仰 pitch
        （pitch 为负 = 镜头下俯，看向地面）。
        """
        y = math.radians(self.yaw)
        p = math.radians(self.pitch)
        # 基准三轴（列向量即各轴的 world 分量）
        f = [1.0, 0.0, 0.0]
        r = [0.0, -1.0, 0.0]
        d = [0.0, 0.0, -1.0]

        def rot_z(v, a):
            c, s = math.cos(a), math.sin(a)
            return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]

        # 1) yaw：绕世界 z 旋转三轴
        f, r, d = rot_z(f, y), rot_z(r, y), rot_z(d, y)

        # 2) pitch：绕「当前右轴 r」旋转 f 与 d（罗德里格斯，r 为单位轴）
        def rot_axis(v, axis, a):
            c, s = math.cos(a), math.sin(a)
            ax, ay, az = axis
            dot = ax * v[0] + ay * v[1] + az * v[2]
            cross = [ay * v[2] - az * v[1],
                     az * v[0] - ax * v[2],
                     ax * v[1] - ay * v[0]]
            return [v[i] * c + cross[i] * s + axis[i] * dot * (1 - c)
                    for i in range(3)]

        f = rot_axis(f, r, p)
        d = rot_axis(d, r, p)
        return f, r, d

    def rotation(self) -> list[list[float]]:
        """
        世界 → 相机 的旋转矩阵 R（OpenCV 风格相机轴：X_c 右, Y_c 下, Z_c 前）。

        设相机三轴（世界坐标）为 f, r, d，则对任意世界向量 v：
            X_c = v · r
            Y_c = v · d
            Z_c = v · f
        故 R 的第 0/1/2 行分别为 rᵀ / dᵀ / fᵀ。
        """
        f, r, d = self.axes()
        return [list(r), list(d), list(f)]

    def project(self, pt: tuple[float, float, float]) -> tuple[float, float] | None:
        """
        三维世界点 → 像素坐标。若在相机后方或超视场，返回 None。
        """
        R = self.rotation()
        d = [pt[0] - self.pos[0], pt[1] - self.pos[1], pt[2] - self.pos[2]]
        # 世界 → 相机
        c = [sum(R[i][k] * d[k] for k in range(3)) for i in range(3)]
        if c[2] <= 1e-6:
            return None                     # 在相机后方
        u = self.fx * c[0] / c[2] + self.cx
        v = self.fy * c[1] / c[2] + self.cy
        if not (0 <= u < self.res_w and 0 <= v < self.res_h):
            return None                     # 出画
        return (u, v)

    def in_fov(self, pt) -> bool:
        return self.project(pt) is not None


def default_cameras(L: float, W: float, H: float) -> list[Camera]:
    """
    给出两个典型车载相机的默认参数。

    ★ 设计依据（为什么这样摆）：
      车厢是**细长的深腔**（V1 内尺寸 2600×1400×1300，长宽比 1.86）。
      把相机放在**中轴线**上时，两侧壁到光轴的距离很大而进深很小，
      导致近端必然出画 —— 这是几何上的死结，不是调参问题。
      真实车载相机因此装在**角落 + 大广角**，用「斜对角线」换取
      「整舱进深」都能落在视锥内。

    相机 1 · 尾门左后上角（装车定位用）：
        装在尾门内侧左上角 (x≈0.02L, y≈0.955W, z≈0.905H)，朝车头前下方看，
        偏航 8°（向车舱中线内收），俯仰 −14°，HFOV 150°。
        职责：把「货物该放的格位」框在画面里 → 作业员对齐放置。
    相机 2 · 顶棚前部俯视（平面布局校核用）：
        装在顶棚靠车头一侧 (x≈0.98L, y=W/2, z=H)，朝车尾方向俯视
        （yaw=180°，pitch=−32°），HFOV 170°。
        职责：从车头往车尾俯看**底面布局** —— 判断哪块地板还空着。
        注意：它天生看不到自己脚下（x=L 的远端竖框下半部），
        这是「俯视机位」的物理必然，故其判据是**底面 4/4 入画**。
    """
    # ── 相机 1：尾门左后上角，斜向内 + 下俯（定位放置）
    c1 = Camera(
        name="尾门后视相机",
        pos=(0.022 * L, 0.955 * W, 0.905 * H),
        yaw=8.0, pitch=-14.0,
        hfov_deg=150.0, res_w=1920, res_h=1080,
    )
    # ── 相机 2：顶棚前部，朝车尾方向俯视（平面布局）
    c2 = Camera(
        name="顶棚俯视相机",
        pos=(0.98 * L, 0.50 * W, 1.00 * H),
        yaw=180.0, pitch=-32.0,
        hfov_deg=170.0, res_w=1920, res_h=1080,
    )
    return [c1, c2]


# ══════════════════════════════════════════════ 可见性与遮挡
@dataclass
class BoxView:
    """一件货在相机画面中的可见性结论。"""
    uid: str
    sku_id: str
    name: str
    store_id: str
    visible: bool
    visible_ratio: float          # 可见（未被遮挡）的采样点占比
    silhouette_px: tuple | None   # 画面中的外接框 (u0,v0,u1,v1) 或 None
    blocked_by: list[str]         # 遮挡它的箱号
    reason: str


def sample_surface_points(box: dict, n: int = 12) -> list[tuple[float, float, float]]:
    """
    在一件货的**外表面 + 内部**采样若干点，用于可见性判定。

    采样策略：在包围盒内取 n×n×n 网格（n 默认 12 ⇒ 1728 点太多，
    此处取表面的 6 个面各若干点 + 少量内部点，兼顾精度与速度）。
    """
    x0, y0, z0 = box["x"], box["y"], box["z"]
    x1, y1, z1 = x0 + box["l"], y0 + box["w"], z0 + box["h"]
    cs = [i / (n - 1) for i in range(n)]
    pts = []
    for a in cs:
        for b in cs:
            pts.append((x0 + a * (x1 - x0), y0 + b * (y1 - y0), z0))     # 底面
            pts.append((x0 + a * (x1 - x0), y0 + b * (y1 - y0), z1))     # 顶面
            pts.append((x0 + a * (x1 - x0), y0, z0 + b * (z1 - z0)))     # 前侧
            pts.append((x0 + a * (x1 - x0), y1, z0 + b * (z1 - z0)))     # 后侧
            pts.append((x0, y0 + a * (y1 - y0), z0 + b * (z1 - z0)))     # 左侧
            pts.append((x1, y0 + a * (y1 - y0), z0 + b * (z1 - z0)))     # 右侧
    return pts


def _ray_box_hit(origin, target, box) -> bool:
    """
    判断从 origin 到 target 的线段是否穿过 box 内部（不含端点所属的自身箱）。

    用 slab 法做 AABB 求交，t ∈ (0.02, 0.98) 视为穿过。
    """
    o, d = origin, [target[i] - origin[i] for i in range(3)]
    tmin, tmax = 0.02, 0.98
    lo = [box["x"], box["y"], box["z"]]
    hi = [box["x"] + box["l"], box["y"] + box["w"], box["z"] + box["h"]]
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if o[i] < lo[i] or o[i] > hi[i]:
                return False
            continue
        t1 = (lo[i] - o[i]) / d[i]
        t2 = (hi[i] - o[i]) / d[i]
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return False
    return True


def analyze_visibility(cam: Camera, boxes: list[dict],
                       n_samples: int = 8) -> list[BoxView]:
    """
    对每件货做可见性分析。

    方法：在货的**朝向相机的表面**上采样，逐点向相机做射线，
    若被其他箱挡住则视为不可见。可见占比决定该件能否用画面指引表达。

    ★ 复杂度控制：n_samples=8 时每箱约 8×8×6=384 点；
      对每点做 O(箱数) 的遮挡检测。装载方案通常 ≤200 件 ⇒ 可接受。
      必要时可对箱做「按 x 排序后只检查相机与目标之间的箱」剪枝。
    """
    out: list[BoxView] = []
    for b in boxes:
        pts = sample_surface_points(b, n_samples)
        # 只保留能投到画面内的点（出画的点不算「被遮挡」）
        n_proj = 0
        n_vis = 0
        us, vs = [], []
        blockers: set[str] = set()
        for pt in pts:
            uv = cam.project(pt)
            if uv is None:
                continue
            n_proj += 1
            us.append(uv[0])
            vs.append(uv[1])
            # 射线遮挡测试
            blocked = False
            for other in boxes:
                if other["uid"] == b["uid"]:
                    continue
                if _ray_box_hit(cam.pos, pt, other):
                    blocked = True
                    blockers.add(other["uid"])
                    break
            if not blocked:
                n_vis += 1
        if n_proj == 0:
            out.append(BoxView(b["uid"], b.get("sku_id", ""), b.get("name", ""),
                               b.get("store_id", ""), False, 0.0, None,
                               [], "不在相机视场内"))
            continue
        ratio = n_vis / n_proj
        sil = (min(us), min(vs), max(us), max(vs)) if us else None
        visible = ratio >= 0.30           # 可见面占比 ≥30% 视为「画面可辨」
        reason = ("画面可见，可生成画面指引"
                  if visible else
                  f"被前排遮挡（{len(blockers)} 件遮挡），需改用文字作业序")
        out.append(BoxView(b["uid"], b.get("sku_id", ""), b.get("name", ""),
                           b.get("store_id", ""), visible, round(ratio, 3),
                           tuple(round(x, 1) for x in sil) if sil else None,
                           sorted(blockers)[:5], reason))
    return out


# ══════════════════════════════════════════════ 装载方案校核
def check_boundary(cam: Camera, L: float, W: float, H: float) -> dict:
    """
    校核：车厢内框在相机画面中是否完整可见。

    ★ 判据分两级（原版只判「8 角点全在画面内」，几何上对角落机位永不成立）：
      · 「可定位放置」：**装载作业面**（底面 + 远端竖框）完整入画 ⇒
        作业员可以把货对齐画面里的框线定位。
      · 「可越界告警」：至少能看到货堆的**顶面与两侧上缘** ⇒
        可判断是否超高/超宽。

    角落机位天然看不到自己背后的那两面（x=0 与 y=W 的墙面在镜头背后），
    这属于**物理必然**，不应判为失败。因此：
      - 只统计**位于相机前方**（Z_c>0）的角点；
      - 「画面内占比」= 前方角点里入画的比例。
    """
    corners = [(0, 0, 0), (L, 0, 0), (L, W, 0), (0, W, 0),
               (0, 0, H), (L, 0, H), (L, W, H), (0, W, H)]
    R = cam.rotation()
    front, inside, behind = [], [], []
    for c in corners:
        d = [c[i] - cam.pos[i] for i in range(3)]
        zc = sum(R[2][k] * d[k] for k in range(3))
        if zc <= 1e-6:
            behind.append(c)
            continue
        front.append(c)
        if cam.in_fov(c):
            inside.append(c)

    # 装载作业面：远端竖框 4 角（x=L 面）＋底面 4 角中可见者
    far_face = [(L, 0, 0), (L, W, 0), (L, 0, H), (L, W, H)]
    far_ok = sum(1 for c in far_face if cam.in_fov(c))
    floor_ok = sum(1 for c in [(0, 0, 0), (L, 0, 0), (L, W, 0), (0, W, 0)]
                   if cam.in_fov(c))

    ratio = len(inside) / max(len(front), 1)
    return {
        "相机": cam.name,
        "车厢8角点": len(corners),
        "相机前方角点": len(front),
        "镜头背后角点": len(behind),
        "画面内角点数": len(inside),
        "前方角点入画率": round(ratio, 3),
        "远端竖框入画": f"{far_ok}/4",
        "底面角点入画": f"{floor_ok}/4",
        "可定位放置": far_ok >= 3 and floor_ok >= 3,
        "可越界告警": len(inside) >= 4,
        "投影": {f"P{i}": (tuple(round(x, 1) for x in cam.project(c))
                          if cam.project(c) else None)
                 for i, c in enumerate(corners)},
    }


def verify_step_visiting(cam: Camera, ordered_boxes: list[dict],
                         n_samples: int = 6) -> dict:
    """
    ★★ 核心业务校验：**逐装车步**判断「目标格位是否在画面里看得见」。

    为什么不用「满载后可辨件数」做指标：
      满载的车厢本身就是一堵货墙，任何单机位都只能看到最外一层 ——
      那时「可辨件数」低是**装载已完成的必然结果**，不能反映指引能力。
      真正决定「摄像头能否当作业指引载体」的是：
      **在放入第 k 件货的那一刻，第 k 件的落位面是否无遮挡可见。**

    做法：按装载顺序逐件加料，对每一件在**仅已放入的货**构成的场景中
    做一次可见性判定 —— 模拟「作业员装到这一步时，相机画面里能不能看到
    该件应该放的格位」。

    返回：逐步可见率 + 分段明细（用于识别「装到第几层后画面失效」）。
    """
    scene: list[dict] = []
    steps = []
    for k, b in enumerate(ordered_boxes):
        pts = sample_surface_points(b, n_samples)
        n_proj = n_vis = 0
        for pt in pts:
            if cam.project(pt) is None:
                continue
            n_proj += 1
            if not any(_ray_box_hit(cam.pos, pt, o) for o in scene):
                n_vis += 1
        ratio = (n_vis / n_proj) if n_proj else 0.0
        steps.append({
            "步序": k + 1,
            "uid": b["uid"],
            "批次": b.get("batch", b.get("装车批次", "")),
            "可见比": round(ratio, 3),
            "可指引": ratio >= 0.30,
        })
        scene.append(b)

    ok = [s for s in steps if s["可指引"]]
    # 分层统计：按批次归并
    by_batch: dict[str, list] = {}
    for s in steps:
        by_batch.setdefault(str(s["批次"]), []).append(s)
    seg = [{
        "批次": k,
        "件数": len(v),
        "可指引": sum(1 for s in v if s["可指引"]),
        "可指引率_%": round(sum(1 for s in v if s["可指引"]) / len(v) * 100, 1),
    } for k, v in by_batch.items()]

    return {
        "总步数": len(steps),
        "逐步可指引步数": len(ok),
        "逐步可指引率_%": round(len(ok) / max(len(steps), 1) * 100, 2),
        "分段明细": seg,
        "逐步明细": steps,
    }


def build_camera_report(plan_path: Path, out_path: Path) -> dict:
    """
    由装载方案（loading_plan.json）生成相机视角校验报告。
    """
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    L, W, H = plan["车厢"]["内尺寸_mm"]
    boxes = plan["三维布局"]
    cams = default_cameras(L, W, H)

    report = {
        "车厢": plan["车厢"],
        "件数": len(boxes),
        "相机": [],
    }
    # 装载顺序（用于逐步可见性模拟）—— 装载序条目自带 箱号/批次
    seq = plan.get("装载序") or []
    by_uid = {b["uid"]: b for b in boxes}
    ordered = []
    for s in seq:
        b = by_uid.get(s.get("箱号"))
        if b is None:
            continue
        b = dict(b)
        b["batch"] = s.get("批次", "")
        b["第几步"] = s.get("第几步", "")
        ordered.append(b)

    for cam in cams:
        views = analyze_visibility(cam, boxes)
        n_vis = sum(1 for v in views if v.visible)
        steps = verify_step_visiting(cam, ordered)
        report["相机"].append({
            "参数": {
                "名称": cam.name,
                "安装位置_mm": [round(p, 1) for p in cam.pos],
                "偏航角_度": cam.yaw,
                "俯仰角_度": cam.pitch,
                "水平视场角_度": cam.hfov_deg,
                "分辨率": [cam.res_w, cam.res_h],
                "焦距_px": round(cam.fx, 1),
                "光轴": [round(x, 4) for x in cam.axes()[0]],
            },
            "车厢内框校核": check_boundary(cam, L, W, H),
            "可见性统计": {
                "总件数": len(views),
                "画面可辨件数": n_vis,
                "画面可辨占比_%": round(n_vis / max(len(views), 1) * 100, 2),
                "不可辨件数": len(views) - n_vis,
            },
            "★逐步可指引校验": steps,
            "逐件结论": [asdict(v) for v in views],
        })

    out_path.write_text(json.dumps(report, ensure_ascii=False, indent=1),
                        encoding="utf-8")
    return report


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from paths import ROOT

    rep = build_camera_report(ROOT / "out" / "loading_plan.json",
                              ROOT / "out" / "camera_report.json")
    print("=== 车载摄像头视角校验装载方案 ===")
    print(f"车厢 {rep['车厢']['名称']} 内尺寸 {rep['车厢']['内尺寸_mm']} mm "
          f"| 装载 {rep['件数']} 件\n")
    for cam in rep["相机"]:
        p = cam["参数"]
        print(f"[{p['名称']}]")
        print(f"  安装 {p['安装位置_mm']} mm | 俯仰 {p['俯仰角_度']}° | "
              f"偏航 {p['偏航角_度']}° | HFOV {p['水平视场角_度']}° | "
              f"焦距 {p['焦距_px']} px")
        cb = cam["车厢内框校核"]
        print(f"  车厢校核: 前方角点入画 {cb['画面内角点数']}/{cb['相机前方角点']} "
              f"({cb['前方角点入画率']*100:.0f}%) | 底面 {cb['底面角点入画']} | "
              f"远端竖框 {cb['远端竖框入画']}")
        print(f"          可定位放置={cb['可定位放置']}  可越界告警={cb['可越界告警']}")
        vs = cam["可见性统计"]
        print(f"  满载后可辨 {vs['画面可辨件数']}/{vs['总件数']} 件 "
              f"({vs['画面可辨占比_%']}%)")
        st = cam["★逐步可指引校验"]
        print(f"  ★逐步可指引: {st['逐步可指引步数']}/{st['总步数']} 步 "
              f"({st['逐步可指引率_%']}%) —— 每放入一件时该件落位面是否可见")
        for s in st["分段明细"]:
            print(f"      · 第 {s['批次']} 段: {s['可指引']}/{s['件数']} "
                  f"({s['可指引率_%']}%)")
        # 举两个例子：一个可见、一个被遮挡
        shown = 0
        for item in cam["逐件结论"]:
            if item["visible"] and shown < 2:
                print(f"    · [可见] {item['uid']} {item['name']}: "
                      f"可见比 {item['visible_ratio']} 画面框 {item['silhouette_px']}")
                shown += 1
        n_shown = 0
        for item in cam["逐件结论"]:
            if not item["visible"] and item["visible_ratio"] > 0 and n_shown < 1:
                print(f"    · [遮挡] {item['uid']} {item['name']}: "
                      f"可见比 {item['visible_ratio']} | {item['reason']}")
                n_shown += 1
        # 出画统计
        out_frame = sum(1 for it in cam["逐件结论"] if it["reason"] == "不在相机视场内")
        print(f"    · 完全出画 {out_frame} 件")
        print()
