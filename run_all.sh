#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════
# 传化黑客松 · 城市配送智能配载 · 一键复现
#
# 依次执行：数据生成 → 路线优化 → 精确性验证 → 仓内拣货 → 三维装载
#          → 配送感知优化 → 相机校验与渲染 → 装载对比 → 全部自证
#
# 任何一步失败即以非零码退出（set -e）。
# ══════════════════════════════════════════════════════════════════════
set -euo pipefail

PY="${PY:-python3}"
cd "$(dirname "$0")"

# 依赖检查：精确层用 numpy + scipy(HiGHS)。缺失时给出明确提示而不是中途报错。
if ! "$PY" -c 'import numpy, scipy' 2>/dev/null; then
  printf '缺少依赖：请先执行  %s -m pip install -r requirements.txt\n' "$PY" >&2
  exit 1
fi

hr() { printf '%s\n' "────────────────────────────────────────────────────────────"; }
step() { hr; printf '▶ %s\n' "$1"; hr; }

step "1/9 生成杭州路网与订单算例"
"$PY" src/data/hangzhou_net.py
"$PY" src/data/goods.py

step "2/9 运力匹配与路径优化（含动态要货）"
"$PY" src/route/vrptw.py

step "3/9 精确性验证（Held-Karp DP + 集合划分 MILP）"
"$PY" src/route/milp_verify.py

step "4/9 仓内最近路径与最小翻垛配货"
"$PY" src/picking/pick_planner.py

step "5/9 三维装载（切分层/货架式）"
"$PY" src/loading/layer_packer.py

step "6/9 3D 装载布局与分步装卸指引"
"$PY" src/unload/unload_guide.py
"$PY" src/unload/delivery_aware.py

step "7/9 车载摄像头视角校验与渲染"
"$PY" src/vision/camera_check.py
"$PY" src/vision/render_view.py

step "8/9 三维装载 · 算法 vs 人工基线对比"
"$PY" compare_loading.py

step "9/9 全部自证"
"$PY" tests/verify_exact_dp.py
"$PY" tests/verify_solution_consistency.py
"$PY" tests/verify_camera.py

hr
printf '✓ 全部完成。结果见 out/ ，相机渲染见 assets/camera/\n'
hr
