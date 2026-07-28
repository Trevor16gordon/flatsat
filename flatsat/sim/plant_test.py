"""Rigid-body plant: the physics the scenario missions stand on."""

import pytest

from flatsat.sim.plant import RigidBody

DIAG = [[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]]


def test_no_torque_no_change_for_spherical_body() -> None:
    body = RigidBody(DIAG, (0.1, -0.2, 0.3))
    for _ in range(100):
        body.step((0.0, 0.0, 0.0), 0.01)
    assert body.omega == pytest.approx([0.1, -0.2, 0.3])


def test_constant_torque_integrates_linearly() -> None:
    body = RigidBody(DIAG, (0.0, 0.0, 0.0))
    for _ in range(100):
        body.step((0.005, 0.0, 0.0), 0.01)
    # omega = tau * t / I = 0.005 * 1.0 / 0.05
    assert body.omega[0] == pytest.approx(0.1, rel=1e-6)
    assert body.omega[1] == pytest.approx(0.0)


def test_gyroscopic_coupling_appears_for_asymmetric_body() -> None:
    """An asymmetric tumbler exchanges rate between axes — Euler's term."""
    inertia = [[0.09, 0.0, 0.0], [0.0, 0.08, 0.0], [0.0, 0.0, 0.06]]
    body = RigidBody(inertia, (0.3, 0.2, 0.0))
    body.step((0.0, 0.0, 0.0), 0.01)
    assert body.omega[2] != 0.0, "omega x (I omega) coupling missing"


def test_opposing_torque_damps_the_rate() -> None:
    body = RigidBody(DIAG, (0.5, 0.0, 0.0))
    for _ in range(400):
        torque = -0.1 * body.omega[0]  # ideal PD-ish damping
        body.step((torque, 0.0, 0.0), 0.01)
    assert body.rate_magnitude() < 0.001
