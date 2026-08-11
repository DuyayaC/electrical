import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[3]
SRC = ROOT / "FINAL-2026" / "User" / "WheelLeg"
TEST = ROOT / "tools" / "wheel_leg" / "tests" / "phase2_test.c"
BUILD = pathlib.Path("/tmp/wheel_leg_phase2_test")


def test_algorithm_chain_math():
    BUILD.mkdir(parents=True, exist_ok=True)
    executable = BUILD / "phase2"
    sources = [
        SRC / "five_bar.c",
        SRC / "wheel_leg_state_estimator.c",
        SRC / "wheel_leg_lqr.c",
        SRC / "wheel_leg_vmc.c",
        TEST,
    ]
    command = [
        "gcc",
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(SRC),
        *(str(source) for source in sources),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


def test_python_fivebar_reference_and_virtual_work():
    import sys

    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from five_bar_reference import FiveBarGeometry, solve, virtual_work_residual

    test_geometry = FiveBarGeometry(
        l1=0.140,
        l2=0.200,
        l3=0.200,
        l4=0.140,
        base_width=0.100,
        branch_sign=-1.0,
    )
    result = solve(1.0471975512, 2.0943951024, geometry=test_geometry)
    assert abs(result["leg_axis_body"]) < 1e-5
    assert abs(result["detJ"]) > 1e-3
    wrench = (3.0, 0.7)
    qdot = (0.2, -0.1)
    J = result["J"]
    tau = (J[0][0] * wrench[0] + J[1][0] * wrench[1],
           J[0][1] * wrench[0] + J[1][1] * wrench[1])
    assert abs(virtual_work_residual(J, qdot, tau, wrench)) < 1e-9
