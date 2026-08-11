"""Reference five-bar kinematics used to cross-check the STM32 implementation.

The default geometry in this module is a test fixture only. Production code
must receive measured geometry explicitly. Angles are radians; the returned
leg angle is measured from the body +Y axis, making the upright pose zero.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin, sqrt
from typing import Tuple


@dataclass(frozen=True)
class FiveBarGeometry:
    l1: float = 0.140
    l2: float = 0.200
    l3: float = 0.200
    l4: float = 0.140
    base_width: float = 0.100
    branch_sign: float = -1.0
    min_length: float = 0.05
    max_length: float = 0.40
    tolerance: float = 1e-6
    finite_difference_step: float = 1e-4


def _wrap(angle: float) -> float:
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle < -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle


def position(q1: float, q4: float, geometry: FiveBarGeometry = FiveBarGeometry()) -> Tuple[float, float]:
    """Return ``(L0, leg_axis_body)`` or raise ``ValueError`` if unsolved."""

    if min(geometry.l1, geometry.l2, geometry.l3, geometry.l4,
           geometry.base_width) <= 0.0:
        raise ValueError("five-bar lengths must be positive")
    A = (0.0, 0.0)
    E = (geometry.base_width, 0.0)
    B = (geometry.l1 * cos(q1), geometry.l1 * sin(q1))
    D = (E[0] + geometry.l4 * cos(q4), geometry.l4 * sin(q4))
    dx = D[0] - B[0]
    dy = D[1] - B[1]
    distance = hypot(dx, dy)
    if distance <= geometry.tolerance:
        raise ValueError("coincident circle centers")
    projection = (geometry.l2 ** 2 - geometry.l3 ** 2 + distance ** 2) / (2.0 * distance)
    height_squared = geometry.l2 ** 2 - projection ** 2
    if height_squared < -geometry.tolerance:
        raise ValueError("five-bar circles do not intersect")
    height = sqrt(max(0.0, height_squared))
    ux, uy = dx / distance, dy / distance
    px, py = B[0] + projection * ux, B[1] + projection * uy
    C = (px + geometry.branch_sign * height * -uy,
         py + geometry.branch_sign * height * ux)
    ox = geometry.base_width * 0.5
    length = hypot(C[0] - ox, C[1])
    if not geometry.min_length <= length <= geometry.max_length:
        raise ValueError("leg length outside configured range")
    return length, atan2(C[0] - ox, C[1])


def solve(q1: float, q4: float, dq1: float = 0.0, dq4: float = 0.0,
          geometry: FiveBarGeometry = FiveBarGeometry()) -> dict:
    """Return FK, numerical Jacobian, and velocities."""

    step = geometry.finite_difference_step
    length, angle = position(q1, q4, geometry)
    lp, ap = position(q1 + step, q4, geometry)
    lm, am = position(q1 - step, q4, geometry)
    lq, aq = position(q1, q4 + step, geometry)
    lr, ar = position(q1, q4 - step, geometry)
    J = (
        ((lp - lm) / (2.0 * step), (lq - lr) / (2.0 * step)),
        (_wrap(ap - am) / (2.0 * step), _wrap(aq - ar) / (2.0 * step)),
    )
    determinant = J[0][0] * J[1][1] - J[0][1] * J[1][0]
    dlength = J[0][0] * dq1 + J[0][1] * dq4
    dangle = J[1][0] * dq1 + J[1][1] * dq4
    return {
        "L0": length,
        "leg_axis_body": angle,
        "dL0": dlength,
        "dleg_axis_body": dangle,
        "J": J,
        "detJ": determinant,
    }


def virtual_work_residual(J, qdot, tau, wrench) -> float:
    """Return ``tau.T*qdot - wrench.T*(J*qdot)`` for a VMC sign check."""

    dx = (J[0][0] * qdot[0] + J[0][1] * qdot[1],
          J[1][0] * qdot[0] + J[1][1] * qdot[1])
    return tau[0] * qdot[0] + tau[1] * qdot[1] - (wrench[0] * dx[0] + wrench[1] * dx[1])


if __name__ == "__main__":
    print(solve(1.0471975512, 2.0943951024))
