# -*- coding: utf-8 -*-
"""
项目路径解析（放在 src 下，供所有模块与测试统一调用）

★ 教训：最初各模块各自用 Path(__file__).parents[N] 推导项目根，
  但 src/**/*.py 与 tests/*.py 的层级不同（parents[2] vs parents[1]），
  导致同一函数在不同调用点解析出不同目录、报 FileNotFoundError。
  统一改为「向上查找含 out/ 与 src/ 的目录」作为项目根，与文件深度无关。
"""
from __future__ import annotations

from pathlib import Path

_MARKERS = ("out", "src")


def project_root(start: Path | None = None) -> Path:
    """向上查找同时含 out/ 与 src/ 的目录作为项目根。"""
    p = (start or Path(__file__)).resolve()
    for cand in [p, *p.parents]:
        if all((cand / m).is_dir() for m in _MARKERS):
            return cand
    raise RuntimeError(f"未找到项目根（缺少 {_MARKERS}）：{p}")


ROOT = project_root()
OUT = ROOT / "out"
ASSETS = ROOT / "assets"
DOCS = ROOT / "docs"
