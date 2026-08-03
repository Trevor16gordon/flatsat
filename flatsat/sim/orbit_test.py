"""Orbit and field: checked against known physics, not against itself.

A self-consistent propagator that is wrong is easy to write and hard to
notice, so the assertions here are anchored to values that can be looked
up: the ISS period, the surface field strength, the sun-synchronous
inclination everyone quotes for a 500 km orbit, the factor of two
between the magnetic equator and the poles.
"""

import math

import numpy as np
import pytest

from flatsat.sim import orbit


def test_iss_altitude_gives_the_iss_period() -> None:
    """~420 km should come out near 92-93 minutes, as it observably does."""
    minutes = orbit.circular(420e3, math.radians(51.6)).period_s / 60.0
    assert 92.0 < minutes < 93.5, f"{minutes:.1f} min is not an ISS-like period"


def test_period_follows_keplers_third_law() -> None:
    """Quadrupling the semi-major axis must octuple the period."""
    low = orbit.circular(500e3, 0.0)
    high = orbit.OrbitalElements(low.semi_major_axis_m * 4, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert high.period_s / low.period_s == pytest.approx(8.0, rel=1e-9)


def test_geostationary_radius_gives_a_sidereal_day() -> None:
    """The classic sanity check: 42164 km must take one sidereal day."""
    geo = orbit.OrbitalElements(42_164_000.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert geo.period_s == pytest.approx(86164.0, rel=2e-3)


# ------------------------------------------------------------ propagation --


def test_circular_orbit_keeps_its_radius() -> None:
    """No eccentricity, no radius variation — the cheapest integrator check."""
    elements = orbit.circular(500e3, math.radians(97.4))
    radii = [
        float(np.linalg.norm(orbit.propagate_eci(elements, t)[0]))
        for t in np.linspace(0, elements.period_s, 40)
    ]
    assert max(radii) - min(radii) < 1.0, "circular orbit radius drifted"


def test_speed_matches_the_vis_viva_equation() -> None:
    """Speed on a circle is sqrt(mu/r), derived independently of the code."""
    elements = orbit.circular(500e3, 0.0)
    _, velocity = orbit.propagate_eci(elements, 1234.0)
    expected = math.sqrt(orbit.MU_EARTH_M3_S2 / elements.semi_major_axis_m)
    assert float(np.linalg.norm(velocity)) == pytest.approx(expected, rel=1e-9)
    assert 7.5e3 < expected < 7.7e3, "LEO orbital speed should be ~7.6 km/s"


def test_one_period_returns_to_the_start() -> None:
    """Closure after a full revolution, allowing for nodal drift."""
    elements = orbit.circular(500e3, math.radians(97.4))
    start, _ = orbit.propagate_eci(elements, 0.0)
    later, _ = orbit.propagate_eci(elements, elements.period_s)
    # J2 moves the node a little each orbit, so this is close but not exact.
    assert float(np.linalg.norm(later - start)) < 2e4


def test_position_and_velocity_are_perpendicular_on_a_circle() -> None:
    elements = orbit.circular(600e3, math.radians(45.0))
    position, velocity = orbit.propagate_eci(elements, 900.0)
    cosine = float(
        np.dot(position, velocity) / (np.linalg.norm(position) * np.linalg.norm(velocity))
    )
    assert abs(cosine) < 1e-9


def test_inclination_bounds_the_latitude() -> None:
    """A 51.6 degree orbit must never fly over Reykjavik."""
    elements = orbit.circular(420e3, math.radians(51.6))
    for t in np.linspace(0, elements.period_s, 60):
        position, _ = orbit.propagate_eci(elements, float(t))
        latitude = math.degrees(math.asin(position[2] / np.linalg.norm(position)))
        assert abs(latitude) <= 51.7


# ------------------------------------------------------ sun-synchronicity --


def test_sun_synchronous_inclination_matches_the_published_value() -> None:
    """500 km SSO is ~97.4 degrees — the number every mission quotes."""
    degrees = math.degrees(orbit.sun_synchronous_inclination_rad(500e3))
    assert degrees == pytest.approx(97.4, abs=0.15)


def test_sun_synchronous_orbits_are_retrograde() -> None:
    """The drift has to be eastward, which needs inclination past 90."""
    for altitude in (400e3, 600e3, 800e3):
        assert math.degrees(orbit.sun_synchronous_inclination_rad(altitude)) > 90.0


def test_sun_synchronous_node_drifts_one_turn_per_year() -> None:
    """The defining property, checked directly rather than assumed."""
    inclination = orbit.sun_synchronous_inclination_rad(500e3)
    drift = orbit.raan_drift_rad_s(orbit.circular(500e3, inclination))
    per_year = drift * 365.2422 * 86400.0
    assert per_year == pytest.approx(2.0 * math.pi, rel=1e-3)


def test_prograde_orbits_regress_westward() -> None:
    """Sign convention, and the reason SSO must be retrograde."""
    assert orbit.raan_drift_rad_s(orbit.circular(500e3, math.radians(51.6))) < 0.0


def test_no_sun_synchronous_orbit_exists_far_out() -> None:
    with pytest.raises(ValueError, match="no sun-synchronous"):
        orbit.sun_synchronous_inclination_rad(20_000e3)


# ------------------------------------------------------------------ field --


def test_surface_field_at_the_magnetic_equator() -> None:
    """~31 uT at the equator: the number the model is defined by."""
    # Straight out along x at t=0 the dipole is tilted in the x-z plane,
    # so step round to a point genuinely perpendicular to the axis.
    axis = orbit.dipole_axis_eci(0.0)
    perpendicular = np.cross(axis, np.array([0.0, 0.0, 1.0]))
    perpendicular /= np.linalg.norm(perpendicular)
    field = orbit.magnetic_field_eci(perpendicular * orbit.R_EARTH_M, 0.0)
    assert float(np.linalg.norm(field)) == pytest.approx(orbit.B0_TESLA, rel=1e-6)


def test_polar_field_is_twice_the_equatorial_field() -> None:
    """The factor of two is intrinsic to a dipole, so it must hold exactly."""
    axis = orbit.dipole_axis_eci(0.0)
    polar = orbit.magnetic_field_eci(axis * orbit.R_EARTH_M, 0.0)
    assert float(np.linalg.norm(polar)) == pytest.approx(2.0 * orbit.B0_TESLA, rel=1e-6)


def test_field_falls_off_as_one_over_r_cubed() -> None:
    axis = orbit.dipole_axis_eci(0.0)
    near = np.linalg.norm(orbit.magnetic_field_eci(axis * orbit.R_EARTH_M, 0.0))
    far = np.linalg.norm(orbit.magnetic_field_eci(axis * 2.0 * orbit.R_EARTH_M, 0.0))
    assert float(near / far) == pytest.approx(8.0, rel=1e-6)


def test_leo_field_is_in_the_tens_of_microtesla() -> None:
    """The magnitude a magnetorquer actually has to work against."""
    elements = orbit.circular(500e3, math.radians(97.4))
    magnitudes = [
        float(np.linalg.norm(orbit.magnetic_field_eci(orbit.propagate_eci(elements, t)[0], t)))
        for t in np.linspace(0, elements.period_s, 50)
    ]
    assert 15e-6 < min(magnitudes) < 60e-6
    assert 15e-6 < max(magnitudes) < 60e-6


def test_the_field_rotates_along_the_orbit() -> None:
    """B-dot detumble depends on this and nothing else.

    A field that merely changed magnitude would give a torque that
    always pointed the same way and could not remove angular momentum
    about every axis.
    """
    elements = orbit.circular(500e3, math.radians(97.4))
    first = orbit.magnetic_field_eci(orbit.propagate_eci(elements, 0.0)[0], 0.0)
    later_t = elements.period_s / 4.0
    later = orbit.magnetic_field_eci(orbit.propagate_eci(elements, later_t)[0], later_t)
    cosine = float(np.dot(first, later) / (np.linalg.norm(first) * np.linalg.norm(later)))
    assert cosine < 0.8, "field direction barely moved over a quarter orbit"


def test_dipole_axis_turns_with_the_earth() -> None:
    """Half a sidereal day should flip the tilt to the other side."""
    start = orbit.dipole_axis_eci(0.0)
    half_day = math.pi / orbit.EARTH_ROTATION_RAD_S
    later = orbit.dipole_axis_eci(half_day)
    assert later[0] == pytest.approx(-start[0], abs=1e-6)
    assert later[2] == pytest.approx(start[2], abs=1e-9), "tilt from the spin axis is fixed"


def test_dipole_axis_is_a_unit_vector() -> None:
    for t in (0.0, 1e3, 4e4):
        assert float(np.linalg.norm(orbit.dipole_axis_eci(t))) == pytest.approx(1.0, rel=1e-12)


def test_field_at_the_origin_does_not_explode() -> None:
    """A degenerate position must return zero, not infinity."""
    assert np.all(orbit.magnetic_field_eci(np.zeros(3), 0.0) == 0.0)


# ---------------------------------------------------------- sun and shadow --


def test_sun_vector_is_a_unit_vector_and_completes_a_year() -> None:
    year = 365.2422 * 86400.0
    start = orbit.sun_direction_eci(0.0)
    assert float(np.linalg.norm(start)) == pytest.approx(1.0, rel=1e-12)
    assert np.allclose(orbit.sun_direction_eci(year), start, atol=1e-6)


def test_half_a_year_puts_the_sun_on_the_other_side() -> None:
    half = 365.2422 * 86400.0 / 2.0
    assert np.allclose(orbit.sun_direction_eci(half), -orbit.sun_direction_eci(0.0), atol=1e-6)


def test_eclipse_only_behind_the_earth() -> None:
    sun = np.array([1.0, 0.0, 0.0])
    radius = orbit.R_EARTH_M + 500e3
    assert orbit.in_eclipse(np.array([-radius, 0.0, 0.0]), sun), "anti-sunward must be dark"
    assert not orbit.in_eclipse(np.array([radius, 0.0, 0.0]), sun), "sunward must be lit"
    assert not orbit.in_eclipse(np.array([0.0, radius, 0.0]), sun), "terminator must be lit"


def test_a_leo_orbit_spends_part_of_each_pass_in_shadow() -> None:
    """Eclipse fraction drives the power budget, so it must be plausible."""
    elements = orbit.circular(500e3, math.radians(51.6))
    sun = orbit.sun_direction_eci(0.0)
    samples = [
        orbit.in_eclipse(orbit.propagate_eci(elements, float(t))[0], sun)
        for t in np.linspace(0, elements.period_s, 200)
    ]
    fraction = sum(samples) / len(samples)
    assert 0.2 < fraction < 0.45, f"eclipse fraction {fraction:.2f} is not LEO-like"
