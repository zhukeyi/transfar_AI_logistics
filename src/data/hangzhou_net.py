# -*- coding: utf-8 -*-
"""
杭州城市配送算例 · 配送网络定义

数据口径声明（重要）：
  - 坐标    ：真实经纬度（WGS84），取自杭州市公开地理信息，用于构造路网几何
  - 门店集合：以杭州市区连锁药店典型布点为原型构造，**非任何单一企业的真实门店清单**
  - 订单/货量：合成数据（模拟），按电商动态要货场景生成
  - 时间窗  ：合成数据（模拟）

因此本模块产出的一切均为 **模拟算例（synthetic instance）**，
不代表任何企业的真实运营数据。可复现：固定随机种子。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path

# ---------------------------------------------------------------- 回收站/仓库
# 中心仓：以杭州传化公路港（萧山区）为原型方位，作为城市配送的中心仓
WAREHOUSE = {
    "id": "WH",
    "name": "杭州中心仓",
    "lat": 30.1830,
    "lon": 120.2680,
}

# ---------------------------------------------------------------- 门店（30 家）
# 覆盖杭州主城区：上城/拱墅/西湖/滨江/萧山/余杭/钱塘
# 坐标为真实经纬度（约到小数点后 4 位，精度 ~11m，足够城市配送建模）
STORES_RAW = [
    ("S01", "上城清波店",   30.2440, 120.1650),
    ("S02", "上城湖滨店",   30.2540, 120.1720),
    ("S03", "上城彭埠店",   30.2920, 120.2180),
    ("S04", "上城丁桥店",   30.3180, 120.2180),
    ("S05", "拱墅武林店",   30.2780, 120.1580),
    ("S06", "拱墅大关店",   30.3050, 120.1420),
    ("S07", "拱墅半山店",   30.3480, 120.1620),
    ("S08", "拱墅祥符店",   30.3220, 120.1080),
    ("S09", "西湖文新店",   30.2900, 120.1120),
    ("S10", "西湖翠苑店",   30.2810, 120.1290),
    ("S11", "西湖三墩店",   30.3090, 120.0720),
    ("S12", "西湖转塘店",   30.1690, 120.0790),
    ("S13", "西湖留下店",   30.2600, 120.0530),
    ("S14", "滨江长河店",   30.1860, 120.1870),
    ("S15", "滨江西兴店",   30.2030, 120.2150),
    ("S16", "滨江浦沿店",   30.1670, 120.1520),
    ("S17", "萧山北干店",   30.1900, 120.2680),
    ("S18", "萧山市心店",   30.1730, 120.2740),
    ("S19", "萧山宁围店",   30.2130, 120.2830),
    ("S20", "萧山新街店",   30.1870, 120.3340),
    ("S21", "萧山瓜沥店",   30.1910, 120.4120),
    ("S22", "萧山临浦店",   30.0520, 120.2280),
    ("S23", "余杭仓前店",   30.2960, 120.0120),
    ("S24", "余杭良渚店",   30.3720, 120.0480),
    ("S25", "余杭五常店",   30.2570, 120.0280),
    ("S26", "余杭闲林店",   30.2360, 119.9920),
    ("S27", "钱塘下沙店",   30.3160, 120.3420),
    ("S28", "钱塘白杨店",   30.3020, 120.3720),
    ("S29", "临平南苑店",   30.4080, 120.3020),
    ("S30", "临平星桥店",   30.3620, 120.2680),
]


@dataclass
class Node:
    id: str
    name: str
    lat: float
    lon: float
    # 交付时间窗（分钟，自 06:00 起算）
    tw_open: int = 0
    tw_close: int = 0
    # 服务时长（分钟）
    service: int = 6
    # 门店等级：影响时间窗严格度与优先级权重
    tier: str = "B"


@dataclass
class VehicleType:
    id: str
    name: str
    cap_weight_kg: float
    # 车厢内尺寸 (长×宽×高, mm)
    box_l: int
    box_w: int
    box_h: int
    max_volume_l: float      # 容积，单位：升 (L)
    fixed_cost: float
    per_km: float
    avg_speed_kmh: float
    count: int


# 车型库：城市配送常用车型（新能源微面/依维柯/4.2m 厢货）
# max_volume_l = box_l*box_w*box_h / 1e6 （mm³ → L，1 L = 1e6 mm³）
VEHICLE_TYPES = [
    VehicleType("V1", "新能源微面", 800.0, 2600, 1400, 1300, 4732,
                180.0, 1.30, 32.0, 8),
    VehicleType("V2", "依维柯厢货", 1500.0, 3600, 1800, 1800, 11664,
                320.0, 1.75, 28.0, 6),
    VehicleType("V3", "4.2m 厢货",  3000.0, 4200, 2000, 1900, 15960,
                460.0, 2.10, 25.0, 4),
]


def haversine_km(lat1, lon1, lat2, lon2) -> float:
    """球面大圆距离（km）。"""
    R = 6371.0088
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# 城市道路迂回系数（曼哈顿式路网，实际里程 > 直线距离）
ROAD_DETOUR_FACTOR = 1.32


def build_network() -> dict:
    """构造配送网络：节点 + 距离矩阵 + 时间矩阵。"""
    import random
    rng = random.Random(20261011)

    nodes: list[Node] = [Node(WAREHOUSE["id"], WAREHOUSE["name"],
                              WAREHOUSE["lat"], WAREHOUSE["lon"],
                              tw_open=0, tw_close=1440, service=0, tier="WH")]

    for sid, name, lat, lon in STORES_RAW:
        # 门店分三级：A 级（大店，窗口窄、优先级高）B 级 C 级
        r = rng.random()
        if r < 0.25:
            tier, open_h, close_h, svc = "A", 8.0, 11.5, 8
        elif r < 0.72:
            tier, open_h, close_h, svc = "B", 9.0, 15.0, 6
        else:
            tier, open_h, close_h, svc = "C", 10.0, 17.5, 5
        nodes.append(Node(
            sid, name, lat, lon,
            tw_open=int((open_h - 6.0) * 60),
            tw_close=int((close_h - 6.0) * 60),
            service=svc, tier=tier,
        ))

    n = len(nodes)
    dist = [[0.0] * n for _ in range(n)]
    time_m = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            gc = haversine_km(nodes[i].lat, nodes[i].lon,
                              nodes[j].lat, nodes[j].lon)
            real = gc * ROAD_DETOUR_FACTOR
            dist[i][j] = round(real, 3)
            # 时间按车型分别计算，此处给出基准速度 28 km/h 的行驶分钟
            time_m[i][j] = round(real / 28.0 * 60.0, 2)

    return {"nodes": [asdict(x) for x in nodes],
            "dist_km": dist,
            "time_min": time_m}


if __name__ == "__main__":
    net = build_network()
    print(f"节点数: {len(net['nodes'])}")
    print(f"距离矩阵: {len(net['dist_km'])}x{len(net['dist_km'])}")
    print(f"仓→S21(瓜沥) 距离 {net['dist_km'][0][21]:.2f} km")
    print(f"仓→S26(闲林) 距离 {net['dist_km'][0][26]:.2f} km")
    tiers = {}
    for nd in net["nodes"][1:]:
        tiers[nd["tier"]] = tiers.get(nd["tier"], 0) + 1
    print("门店分级:", tiers)
    out = Path(__file__).resolve().parents[2] / "out"
    out.mkdir(parents=True, exist_ok=True)
    (out / "network.json").write_text(
        json.dumps(net, ensure_ascii=False, indent=1), encoding="utf-8")
    print("已写出 out/network.json")
