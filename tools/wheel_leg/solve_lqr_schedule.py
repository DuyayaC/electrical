"""Offline LQR schedule generator for the wheel-leg controller.

The firmware receives a gain supplied by its caller. This tool can consume
measured/modelled ``A(L0)`` and ``B(L0)`` matrices, or derive them from the
paper's wheel-legged inverted-pendulum equations and the CAD parameter file.
It never invents mass, inertia or COM values.

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


def solve_continuous_are_hamiltonian(A: Sequence[Sequence[float]],
                                     B: Sequence[Sequence[float]],
                                     Q: Sequence[Sequence[float]],
                                     R: Sequence[Sequence[float]]) -> np.ndarray:
    """Solve the continuous ARE using the stable Hamiltonian subspace.

    This is the same invariant-subspace construction used by standard CARE
    implementations, expressed with NumPy only so the offline tool does not
    require SciPy on the developer workstation.
    """
    matrix_a = np.asarray(A, dtype=float)
    matrix_b = np.asarray(B, dtype=float)
    matrix_q = np.asarray(Q, dtype=float)
    matrix_r = np.asarray(R, dtype=float)
    if matrix_a.shape != (6, 6) or matrix_b.shape != (6, 2):
        raise ValueError("wheel-leg A/B must have shapes (6,6)/(6,2)")
    if matrix_q.shape != (6, 6) or matrix_r.shape != (2, 2):
        raise ValueError("wheel-leg Q/R must have shapes (6,6)/(2,2)")
    if not np.allclose(matrix_q, matrix_q.T, atol=1e-12):
        raise ValueError("Q must be symmetric")
    if not np.allclose(matrix_r, matrix_r.T, atol=1e-12):
        raise ValueError("R must be symmetric")
    r_inv = np.linalg.inv(matrix_r)
    hamiltonian = np.block([
        [matrix_a, -matrix_b @ r_inv @ matrix_b.T],
        [-matrix_q, -matrix_a.T],
    ])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.flatnonzero(np.real(eigenvalues) < -1e-9)
    if stable.size != 6:
        raise ValueError(
            f"Hamiltonian has {stable.size} stable eigenvalues, expected 6")
    subspace = eigenvectors[:, stable]
    u1 = subspace[:6, :]
    u2 = subspace[6:, :]
    if np.linalg.matrix_rank(u1, tol=1e-9) != 6:
        raise ValueError("CARE stable subspace is singular")
    solution = np.real(u2 @ np.linalg.inv(u1))
    solution = 0.5 * (solution + solution.T)
    residual = (matrix_a.T @ solution + solution @ matrix_a
                - solution @ matrix_b @ r_inv @ matrix_b.T @ solution
                + matrix_q)
    if not np.all(np.isfinite(solution)) or np.linalg.norm(residual, ord=np.inf) > 1e-5:
        raise ValueError("CARE residual is too large")
    return np.linalg.solve(matrix_r, matrix_b.T @ solution)


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
    return solve_continuous_are_hamiltonian(matrix_a, matrix_b, matrix_q, matrix_r)


def paper_dynamics(state: Sequence[float],
                   control: Sequence[float],
                   params: Mapping[str, float]) -> np.ndarray:
    """Return the six-state derivative from paper equations (3)-(9).

    The model is a symmetric half-vehicle: M and I_M supplied to this
    function are already per-side values, while m_p/m_w/I_p/I_w are the
    measured single-side quantities.
    """
    theta, dtheta, _xb, dxb, phi, dphi = np.asarray(state, dtype=float)
    torque, pitch_torque = np.asarray(control, dtype=float)
    R = params["R"]
    L = params["L"]
    L_M = params["L_M"]
    l = params["l"]
    m_w = params["m_w"]
    m_p = params["m_p"]
    mass = params["M"]
    I_w = params["I_w"]
    I_p = params["I_p"]
    I_M = params["I_M"]
    gravity = params.get("g", 9.81)
    span = L + L_M
    sin_theta, cos_theta = np.sin(theta), np.cos(theta)
    sin_phi, cos_phi = np.sin(phi), np.cos(phi)
    # Unknown accelerations and internal forces:
    # [xdd, theta_dd, xb_dd, phi_dd, N, P, N_M, P_M].
    matrix = np.zeros((8, 8), dtype=float)
    rhs = np.zeros(8, dtype=float)
    wheel_denominator = I_w / R + m_w * R
    matrix[0, 0] = wheel_denominator
    matrix[0, 4] = R
    rhs[0] = torque
    matrix[1, 4] = 1.0
    matrix[1, 6] = -1.0
    matrix[1, 0] = -m_p
    matrix[1, 1] = -m_p * L * cos_theta
    rhs[1] = -m_p * L * dtheta * dtheta * sin_theta
    matrix[2, 5] = 1.0
    matrix[2, 7] = -1.0
    matrix[2, 1] = m_p * L * sin_theta
    rhs[2] = m_p * gravity - m_p * L * dtheta * dtheta * cos_theta
    matrix[3, 1] = I_p
    matrix[3, 5] = -L * sin_theta
    matrix[3, 7] = -L_M * sin_theta
    matrix[3, 4] = L * cos_theta
    matrix[3, 6] = L_M * cos_theta
    rhs[3] = -torque + pitch_torque
    matrix[4, 6] = 1.0
    matrix[4, 2] = -mass
    matrix[4, 3] = mass * l * cos_phi
    rhs[4] = mass * l * dphi * dphi * sin_phi
    matrix[5, 7] = 1.0
    matrix[5, 1] = mass * span * sin_theta
    matrix[5, 3] = mass * l * sin_phi
    rhs[5] = (mass * gravity - mass * span * dtheta * dtheta * cos_theta
              - mass * l * dphi * dphi * cos_phi)
    matrix[6, 3] = I_M
    matrix[6, 6] = -l * cos_phi
    matrix[6, 7] = -l * sin_phi
    rhs[6] = pitch_torque
    matrix[7, 0] = 1.0
    matrix[7, 2] = -1.0
    matrix[7, 1] = span * cos_theta
    rhs[7] = span * dtheta * dtheta * sin_theta
    accelerations = np.linalg.solve(matrix, rhs)
    return np.asarray([dtheta, accelerations[1], dxb, accelerations[2],
                       dphi, accelerations[3]], dtype=float)


def linearize_paper_dynamics(params: Mapping[str, float],
                             state_step: float = 1e-6,
                             control_step: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """Finite-difference A/B at the upright zero-input equilibrium."""
    equilibrium_state = np.zeros(6, dtype=float)
    equilibrium_control = np.zeros(2, dtype=float)
    A = np.empty((6, 6), dtype=float)
    B = np.empty((6, 2), dtype=float)
    for column in range(6):
        offset = np.zeros(6, dtype=float)
        offset[column] = state_step
        A[:, column] = (paper_dynamics(equilibrium_state + offset,
                                       equilibrium_control, params)
                        - paper_dynamics(equilibrium_state - offset,
                                         equilibrium_control, params)) / (2.0 * state_step)
    for column in range(2):
        offset = np.zeros(2, dtype=float)
        offset[column] = control_step
        B[:, column] = (paper_dynamics(equilibrium_state,
                                       equilibrium_control + offset, params)
                        - paper_dynamics(equilibrium_state,
                                         equilibrium_control - offset, params)) / (2.0 * control_step)
    return A, B


def build_cad_models(cad: Mapping[str, object]) -> list[Mapping[str, object]]:
    """Build A/B models from the CAD table using the symmetric half-car rule."""
    fixed = cad["fixed"]
    models = []
    for sample in cad["leg_samples"]:
        params = {
            "R": float(fixed["R"]),
            "l": float(fixed["l"]),
            "m_w": float(fixed["m_w"]),
            "m_p": float(fixed["m_p"]),
            "M": float(fixed["M"]) / 2.0,
            "I_M": float(fixed["I_M"]) / 2.0,
            "I_w": float(fixed["I_w"]),
            "L": float(sample["L"]),
            "L_M": float(sample["L_M"]),
            "I_p": float(sample["I_p"]),
        }
        A, B = linearize_paper_dynamics(params)
        models.append({"L0": float(sample["L0"]), "A": A.tolist(), "B": B.tolist()})
    return models


def solve_schedule(models: Iterable[Mapping[str, object]],
                   Q: Sequence[Sequence[float]],
                   R: Sequence[Sequence[float]]) -> list[tuple[float, np.ndarray]]:
    schedule = []
    for model in models:
        if "L0" not in model or "A" not in model or "B" not in model:
            raise ValueError("each model needs L0, A and B")
        gain = solve_lqr(model["A"], model["B"], Q, R)
        closed_loop = np.linalg.eigvals(np.asarray(model["A"], dtype=float)
                                        - np.asarray(model["B"], dtype=float) @ gain)
        if not np.all(np.real(closed_loop) < 1e-7):
            raise ValueError(f"unstable closed-loop model at L0={model['L0']}")
        schedule.append((float(model["L0"]), gain))
    return sorted(schedule, key=lambda item: item[0])


def emit_c_header(schedule: Sequence[tuple[float, np.ndarray]],
                  symbol: str = "WL_LQR_SCHEDULE") -> str:
    lines = [
        "/* Generated offline from CAD half-vehicle dynamics; verify before output. */",
        "#ifndef WL_LQR_SCHEDULE_GENERATED_H",
        "#define WL_LQR_SCHEDULE_GENERATED_H",
        '#include "wheel_leg_lqr_schedule.h"',
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
    lines.extend([
        "};",
        f"static const WheelLegLqrSchedule {symbol} = {{",
        f"    {symbol}_COUNT, {symbol}_L0, {symbol}_K",
        "};",
        "#endif",
        "",
    ])
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
    if "cad" in document:
        models = build_cad_models(document["cad"])
    else:
        models = document.get("models", [])
    q = document.get("Q")
    r = document.get("R", [[1.0, 0.0], [0.0, 0.25]])
    if q is None:
        q = np.diag([1.0, 1.0, 500.0, 100.0, 5000.0, 1.0]).tolist()
    schedule = solve_schedule(models, q, r)
    if not schedule:
        raise ValueError("model file contains no L0 models")
    args.output.write_text(emit_c_header(schedule, args.symbol), encoding="utf-8")


if __name__ == "__main__":
    main()
