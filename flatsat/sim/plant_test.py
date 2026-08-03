"""Rigid-body plant: the physics the scenario missions stand on."""

import numpy as np
import pytest

from flatsat.sim import orbit
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


# ------------------------------------------------- attitude and environment --


def test_attitude_integrates_from_body_rates() -> None:
    """A spin about z must show up as an MRP about z, and nowhere else."""
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    body = RigidBody(identity, (0.0, 0.0, 1.0))
    for _ in range(1000):
        body.step((0.0, 0.0, 0.0), 1e-3)
    assert abs(body.sigma[0]) < 1e-9
    assert abs(body.sigma[1]) < 1e-9
    assert body.sigma[2] > 0.0


def test_mrp_switches_to_the_shadow_set() -> None:
    """Past a full turn the MRP must flip rather than run to infinity."""
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    body = RigidBody(identity, (0.0, 0.0, 4.0))
    for _ in range(4000):
        body.step((0.0, 0.0, 0.0), 1e-3)
        assert sum(v * v for v in body.sigma) <= 1.0 + 1e-9, "MRP left the unit ball"


def test_dcm_is_a_rotation() -> None:
    """Orthonormal with determinant +1, at an arbitrary attitude."""
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dcm = np.array(RigidBody(identity, (0.0, 0.0, 0.0), (0.2, -0.1, 0.35)).dcm_body_from_inertial())
    assert np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-12)
    assert float(np.linalg.det(dcm)) == pytest.approx(1.0, rel=1e-12)


def test_body_field_magnitude_is_attitude_independent() -> None:
    """Rotating the vehicle cannot change how strong the field IS.

    The body-frame components must change and the magnitude must not —
    the cheapest check that the rotation is a rotation and not a
    scaling, and that a magnetorquer model will not gain free authority
    by turning.
    """
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    field_eci = orbit.magnetic_field_eci(np.array([orbit.R_EARTH_M + 5e5, 0.0, 0.0]), 0.0)
    magnitudes = []
    for sigma in ((0.0, 0.0, 0.0), (0.2, -0.1, 0.35), (0.0, 0.5, 0.0)):
        dcm = np.array(RigidBody(identity, (0.0, 0.0, 0.0), sigma).dcm_body_from_inertial())
        magnitudes.append(float(np.linalg.norm(dcm @ field_eci)))
    assert max(magnitudes) - min(magnitudes) < 1e-15
    assert magnitudes[0] == pytest.approx(float(np.linalg.norm(field_eci)), rel=1e-12)
