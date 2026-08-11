"""Offline LQR schedule generator for the wheel-leg controller.

The firmware receives a gain supplied by its caller. This tool consumes
measured/modelled ``A(L0)`` and ``B(L0)`` matrices and emits a C table; it
never invents mass, inertia or COM values.

Input YAML/JSON format::

    Q: [[1, 0, 0, 0, 0, 0], ...]
    R: [[1, 0], [0, 0.25]]
    models:
      - L0: 0.18
        A: [[...], ...]
        B: [[...], ...]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np
from scipy.linalg import solve_continuous_are


def solve_lqr(A: Sequence[Sequence[float]],
              B: Sequence[Sequence[float]],
              Q: Sequence[Sequence[float]],
              R: Sequence[Sequence[float]]) -> np.ndarray:
    """Return continuous-time ``K`` for ``u = -Kx``."""

    matrix_a = np.asarray(A, dtype=float)
    matrix_b = np.asarray(B, dtype=float)
    matrix_q = np.asarray(Q, dtype=float)
    matrix_r = np.asarray(R, dtype=float)
    if matrix_a.shape != (6, 6) or matrix_b.shape != (6, 2):
        raise ValueError("wheel-leg A/B must have shapes (6,6)/(6,2)")
    if matrix_q.shape != (6, 6) or matrix_r.shape != (2, 2):
        raise ValueError("wheel-leg Q/R must have shapes (6,6)/(2,2)")
    solution = solve_continuous_are(matrix_a, matrix_b, matrix_q, matrix_r)
    return np.linalg.solve(matrix_r, matrix_b.T @ solution)


def solve_schedule(models: Iterable[Mapping[str, object]],
                   Q: Sequence[Sequence[float]],
                   R: Sequence[Sequence[float]]) -> list[tuple[float, np.ndarray]]:
    schedule = []
    for model in models:
        if "L0" not in model or "A" not in model or "B" not in model:
            raise ValueError("each model needs L0, A and B")
        gain = solve_lqr(model["A"], model["B"], Q, R)
        schedule.append((float(model["L0"]), gain))
    return sorted(schedule, key=lambda item: item[0])


def emit_c_header(schedule: Sequence[tuple[float, np.ndarray]],
                  symbol: str = "WL_LQR_SCHEDULE") -> str:
    lines = [
        "/* Generated offline; verify CAD dynamics before enabling output. */",
        "#ifndef WL_LQR_SCHEDULE_GENERATED_H",
        "#define WL_LQR_SCHEDULE_GENERATED_H",
        f"#define {symbol}_COUNT {len(schedule)}u",
        f"static const float {symbol}_L0[{symbol}_COUNT] = {{",
        "    " + ", ".join(f"{length:.9g}f" for length, _ in schedule) + ",",
        "};",
        f"static const float {symbol}_K[{symbol}_COUNT][2][6] = {{",
    ]
    for _, gain in schedule:
        lines.append("    {")
        for row in gain:
            lines.append("        {" + ", ".join(f"{value:.9g}f" for value in row) + "},")
        lines.append("    },")
    lines.extend(["};", "#endif", ""])
    return "\n".join(lines)


def _load(path: Path) -> Mapping[str, object]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        return json.loads(text)
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - depends on host setup
        raise RuntimeError("PyYAML is required for YAML model files") from exc
    return yaml.safe_load(text)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--symbol", default="WL_LQR_SCHEDULE")
    args = parser.parse_args()
    document = _load(args.model)
    q = document.get("Q")
    r = document.get("R", [[1.0, 0.0], [0.0, 0.25]])
    if q is None:
        q = np.diag([1.0, 1.0, 500.0, 100.0, 5000.0, 1.0]).tolist()
    schedule = solve_schedule(document.get("models", []), q, r)
    if not schedule:
        raise ValueError("model file contains no L0 models")
    args.output.write_text(emit_c_header(schedule, args.symbol), encoding="utf-8")


if __name__ == "__main__":
    main()
