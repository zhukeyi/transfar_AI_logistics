# -*- coding: utf-8 -*-
"""
独立校验：用暴力全排列验证 Held-Karp DP 的最优性

方法学意义：
  实现「精确解」时最危险的不是算得慢，而是**算错了还自称精确**。
  本项目在开发中曾两次出现 DP 结果劣于启发式（gap 为负），
  根因是枚举上限与 Pareto 支配逻辑。因此这里用**独立的暴力法**
  （对该子集做全排列穷举）交叉验证 DP，确保「精确层」名副其实。

  这不是性能优化，是正确性自证（non-trivially falsifiable self-check）：
  若 DP 有 bug，暴力法会给出更优解，断言立即失败。
"""
from __future__ import annotations

import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "route"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "data"))

from paths import ROOT as _ROOT
from vrptw import load_scenario, Instance, run_vt_import
from milp_verify import _best_order


def brute_force_order(inst, combo, vtype):
    """对 combo 做全排列穷举，返回真正的最优成本与顺序。"""
    n0 = 0
    for i, nd in enumerate(inst.nodes):
        if nd["id"] == "WH":
            n0 = i
    best_c, best_s = float("inf"), None
    for perm in itertools.permutations(combo):
        t, prev = 0.0, n0
        km = 0.0
        late = 0.0
        for i in perm:
            arrive = t + inst.time[prev][i]
            op, cl, svc = inst.twt[i]
            start = max(arrive, op)
            late += max(0.0, start - cl)
            km += inst.dist[prev][i]
            t = start + svc
            prev = i
        km += inst.dist[prev][n0]
        c = vtype.fixed_cost + vtype.per_km * km + inst.late_weight * late
        if c < best_c:
            best_c, best_s = c, perm
    return best_c, list(best_s)


def recompute_cost(inst, seq, vtype, n0=None):
    """独立重算一条序列的真实成本（不依赖 DP 内部记账）。"""
    if n0 is None:
        for i, nd in enumerate(inst.nodes):
            if nd["id"] == "WH":
                n0 = i
    t, prev, km, late = 0.0, n0, 0.0, 0.0
    for i in seq:
        arrive = t + inst.time[prev][i]
        op, cl, svc = inst.twt[i]
        start = max(arrive, op)
        late += max(0.0, start - cl)
        km += inst.dist[prev][i]
        t = start + svc
        prev = i
    km += inst.dist[prev][n0]
    return vtype.fixed_cost + vtype.per_km * km + inst.late_weight * late


def main(k: int = 7, trials: int = 5):
    root = _ROOT
    net, orders, sku_lib = load_scenario(root)
    VTs = run_vt_import()
    inst = Instance(net, orders, sku_lib)
    inst.set_vehicle_types(VTs)
    stores = inst.active_stores()

    print("=" * 60)
    print(f"DP vs 暴力全排列 交叉验证（每例 {k} 家门店，共 {trials} 例）")
    print("  检查项① DP 成本 == 暴力最优成本（最优性）")
    print("  检查项② DP 自报成本 == 其返回序列的真实成本（自一致性）")
    print("=" * 60)
    ok = True
    for t_i in range(trials):
        sub = stores[t_i * k:(t_i + 1) * k] if (t_i + 1) * k <= len(stores) \
            else stores[-k:]
        vt = VTs[t_i % len(VTs)]
        c_bf, s_bf = brute_force_order(inst, tuple(sub), vt)
        c_dp, s_dp = _best_order(inst, tuple(sub), vt)
        c_dp_recomp = recompute_cost(inst, s_dp, vt)

        match_opt = abs(c_bf - c_dp) < 1e-6
        match_self = abs(c_dp_recomp - c_dp) < 1e-6
        ok = ok and match_opt and match_self

        ids_bf = [inst.store_of_node[i] for i in s_bf]
        ids_dp = [inst.store_of_node[i] for i in s_dp]
        print(f"\n例 {t_i+1} [{vt.id}] 门店 {[inst.store_of_node[i] for i in sub]}")
        print(f"  暴力最优   {c_bf:9.2f}  顺序 {ids_bf}")
        print(f"  DP         {c_dp:9.2f}  顺序 {ids_dp}")
        print(f"  DP序列重算 {c_dp_recomp:9.2f}")
        print(f"  ① 最优性    : {'通过' if match_opt else '失败 ★'}")
        print(f"  ② 自一致性  : {'通过' if match_self else '失败 ★'}")
    print("\n" + "=" * 60)
    print(f"总结论: {'全部通过 — 精确层可信' if ok else '存在不一致 — 需修正'}")
    print("=" * 60)
    return ok


if __name__ == "__main__":
    good = main()
    sys.exit(0 if good else 1)
