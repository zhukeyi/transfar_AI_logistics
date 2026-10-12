# -*- coding: utf-8 -*-
"""
主求解器自一致性校验

对 VRPTWSolver 产出的每条路线做**独立重算**，断言：
  ① 路线自报的 dist_km / cost / late_min 与独立重算一致
  ② 载重与容积不超车型上限
  ③ 时间窗穿越情况与自报的 late_min 一致
  ④ 所有活跃门店恰好被覆盖一次（无遗漏、无重复）

方法学：这是「非恒真」校验——若求解器记账有 bug，断言立即失败。
  开发中精确层曾因局部/全局索引混用出现「自报成本 ≠ 实际成本」，
  故主求解器也必须过同一道自检。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "route"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "data"))

from paths import ROOT
from vrptw import load_scenario, Instance, VRPTWSolver, human_baseline, \
    run_vt_import


def recompute_route(inst, route, vtype_map, n0):
    """独立重算一条路线的里程、迟到与成本。"""
    vt = vtype_map[route.vehicle_type]
    t, prev, km, late = 0.0, n0, 0.0, 0.0
    lw = lv = 0.0
    for st in route.stops:
        i = st.node_idx
        arrive = t + inst.time[prev][i]
        op, cl, svc = inst.twt[i]
        start = max(arrive, op)
        late += max(0.0, start - cl)
        t = start + svc
        km += inst.dist[prev][i]
        lw += inst.demand_w.get(i, 0.0)
        lv += inst.demand_v.get(i, 0.0)
        prev = i
    km += inst.dist[prev][n0]
    cost = vt.fixed_cost + vt.per_km * km + inst.late_weight * late
    return {"km": km, "late": late, "cost": cost, "w": lw, "v": lv, "vt": vt}


def check_solution(inst, sol, tag):
    vtype_map = {v.id: v for v in inst.vehicle_types}
    n0 = 0
    for k, nd in enumerate(inst.nodes):
        if nd["id"] == "WH":
            n0 = k
    problems = []
    served = []
    for r in sol.routes:
        rc = recompute_route(inst, r, vtype_map, n0)
        served.extend(st.node_idx for st in r.stops)
        if abs(rc["km"] - r.dist_km) > 0.05:
            problems.append(f"  路线{r.key()} 里程不符: 自报{r.dist_km:.2f} "
                            f"vs 重算{rc['km']:.2f}")
        if abs(rc["cost"] - r.cost) > 0.05:
            problems.append(f"  路线{r.key()} 成本不符: 自报{r.cost:.2f} "
                            f"vs 重算{rc['cost']:.2f}")
        if abs(rc["late"] - r.late_min) > 0.05:
            problems.append(f"  路线{r.key()} 迟到不符: 自报{r.late_min:.2f} "
                            f"vs 重算{rc['late']:.2f}")
        if rc["w"] > rc["vt"].cap_weight_kg + 1e-6:
            problems.append(f"  路线{r.key()} 超载: {rc['w']:.1f} > "
                            f"{rc['vt'].cap_weight_kg}")
        if rc["v"] > rc["vt"].max_volume_l + 1e-6:
            problems.append(f"  路线{r.key()} 超容: {rc['v']:.1f} > "
                            f"{rc['vt'].max_volume_l}")

    active = set(inst.active_stores())
    got = set(served)
    if got - active:
        problems.append(f"  配送了非活跃门店: {sorted(got - active)}")
    if active - got:
        problems.append(f"  遗漏门店 {len(active - got)} 家: "
                        f"{sorted(inst.store_of_node[i] for i in (active - got))[:6]}")
    dup = [s for s in served if served.count(s) > 1]
    if dup:
        problems.append(f"  重复配送门店: {sorted(set(dup))}")

    print(f"\n[{tag}] 路线 {len(sol.routes)} 条 | 覆盖 {len(got)}/{len(active)} 店 "
          f"| 一致性问题 {len(problems)}")
    if problems:
        for p in problems[:12]:
            print(p)
    return not problems


if __name__ == "__main__":
    net, orders, sku_lib = load_scenario(ROOT)
    VTs = run_vt_import()
    inst = Instance(net, orders, sku_lib, available_at=0.0, cutoff_at=None)
    inst.set_vehicle_types(VTs)
    stores = inst.active_stores()

    ok = True
    base = human_baseline(inst, stores)
    ok &= check_solution(inst, base, "人工经验基线")
    sol = VRPTWSolver(inst).solve(stores, time_budget=40.0)
    ok &= check_solution(inst, sol, "算法解")
    print(f"\n自一致性校验总结论: {'全部通过' if ok else '存在问题'}")
    sys.exit(0 if ok else 1)
