"""Faster-than-real-time law verification against REAL-scale vehicles.

The scenario tier flies toy vehicles because it runs at wall clock;
these tests fly the actual flight vehicle through hours of sim time in
seconds of CPU — the check the user actually wants: do the control
algorithms work at the scales the hardware really has?
"""

import math
import statistics
import time

import pytest

from flatsat.core.config import VehicleSpec, load_vehicle
from flatsat.sim import orbit
from flatsat.sim.fastloop import run_fast_loop

STARLINK_INCLINATION_RAD = 0.9250245035569946  # 53 degrees


def _starlink() -> orbit.OrbitalElements:
    return orbit.circular(550e3, STARLINK_INCLINATION_RAD)


def _sun_point_vehicle() -> VehicleSpec:
    """flatsat_v1 rewired to the SUN-pointing posture (strategy + TRIAD).

    The committed vehicle flies nadir_point + star_attitude; these tests
    pin the sun stack specifically, so they select it explicitly — a
    mission posture is a config edit, and so is a test's.

    Returns:
        The modified vehicle.
    """
    vehicle = load_vehicle()
    control = vehicle.control
    control.sun_point.point_axis[:] = [0.0, 0.0, 1.0]
    control.sun_point.k_align = 0.001
    control.sun_point.kp = 0.02
    control.sun_point.kd = 0.005
    control.sun_point.max_torque_n_m = 0.05
    control.sun_point.dump_gain = 0.02
    control.sun_point.max_dipole_a_m2 = 1.2
    control.triad.orbit = "config/orbits/starlink_leo.txtpb"
    return vehicle


def _triad_only_vehicle() -> VehicleSpec:
    """flatsat_v1 with the star tracker removed from the estimator ladder.

    Returns:
        The modified vehicle (keeps nadir_point; estimator = plain triad).
    """
    vehicle = load_vehicle()
    vehicle.control.triad.orbit = "config/orbits/starlink_leo.txtpb"
    return vehicle


@pytest.mark.verifies("FSW-ADCS-007", "FSW-ADCS-012")
def test_flight_vehicle_detumbles_the_aggressive_tumble() -> None:
    """flatsat_v1 kills a 20 deg/s class tumble on its real time constant.

    Ten sim-minutes of the REAL vehicle — the run that takes ten wall
    minutes in HIL — must cost only seconds here, or the tier has lost
    its reason to exist.
    """
    started = time.monotonic()
    result = run_fast_loop(
        load_vehicle(),
        duration_s=600.0,
        omega0=(0.25, -0.2, 0.15),
        sigma0=(0.2, -0.1, 0.35),
        orbit_elements=_starlink(),
    )
    wall_s = time.monotonic() - started
    assert result.final_omega_mag_rad_s < 0.01, (
        f"still tumbling at {result.final_omega_mag_rad_s:.4f} rad/s"
    )
    # Not a benchmark, a tripwire: "fast" must beat real time by a lot.
    assert wall_s < 120.0, f"600 s of sim took {wall_s:.0f} s wall — no longer fast"


@pytest.mark.verifies("FSW-ADCS-012")
def test_real_rods_drain_the_wheels_over_orbital_time() -> None:
    """The 1.2 A·m² rods measurably dump momentum — at their real tempo.

    Two sim-hours at a coarse step: the wheels absorb the tumble, then
    the dump must remove most of the perpendicular momentum. The bound
    is deliberately above the along-B floor — magnetics cannot finish
    the job in any amount of tuning, only orbit geometry can.
    """
    result = run_fast_loop(
        load_vehicle(),
        duration_s=7200.0,
        omega0=(0.25, -0.2, 0.15),
        sigma0=(0.2, -0.1, 0.35),
        orbit_elements=_starlink(),
        dt_s=0.1,
    )
    peak = max(result.wheel_momentum_n_m_s)
    final = result.final_wheel_momentum_n_m_s
    assert peak > 0.25, f"wheels never caught the tumble (peak |h| {peak:.3f})"
    assert final < 0.5 * peak, f"dump ineffective: |h| {peak:.3f} -> {final:.3f} N·m·s over 2 h"
    assert result.final_omega_mag_rad_s < 0.01, "attitude control lost during the dump"


def test_thermal_model_breathes_with_the_eclipse() -> None:
    """The die temperature must actually cycle as the orbit shadows it."""
    result = run_fast_loop(
        load_vehicle(),
        duration_s=7200.0,
        omega0=(0.01, 0.0, 0.0),
        orbit_elements=_starlink(),
        dt_s=0.5,
    )
    assert any(result.in_eclipse), "a 53 degree orbit at this epoch must see shadow"
    assert not all(result.in_eclipse), "and must also see sun"
    swing = max(result.imu_temperature_c) - min(result.imu_temperature_c)
    assert swing > 5.0, f"temperature swing {swing:.1f} C — thermal model not breathing"


@pytest.mark.verifies("FSW-ADCS-013")
def test_triad_tracks_truth_within_sensor_noise() -> None:
    """The full estimation chain: corrupted CSS + mag through TRIAD.

    The onboard models in the triad block are starlink_leo — the SAME
    orbit and epoch this run flies — so the residual error is the
    sensors' to own: css0's 0.02 rad cone noise dominates.
    """
    result = run_fast_loop(
        _triad_only_vehicle(),
        duration_s=300.0,
        omega0=(0.02, -0.01, 0.015),
        sigma0=(0.2, -0.1, 0.35),
        orbit_elements=_starlink(),
        dt_s=0.1,
    )
    errors = [e for e in result.attitude_error_deg if not math.isnan(e)]
    assert len(errors) > 100, "TRIAD produced almost no valid attitudes in full sun"
    median = statistics.median(errors)
    assert median < 3.0, f"median attitude error {median:.2f} deg vs the ~1.2 deg sensor floor"


@pytest.mark.verifies("FSW-ADCS-014")
def test_sun_pointing_acquires_and_holds_the_sun() -> None:
    """From a tumble, +z must end up on the sun and stay there.

    An hour of sim time guarantees settled sunlit samples on the far
    side of whatever eclipse arc the window contains.
    """
    result = run_fast_loop(
        _sun_point_vehicle(),
        duration_s=3600.0,
        omega0=(0.05, -0.04, 0.03),
        sigma0=(0.2, -0.1, 0.35),
        orbit_elements=_starlink(),
        dt_s=0.2,
    )
    sunlit = [a for a in result.sun_angle_deg if not math.isnan(a)]
    assert len(sunlit) > 1000, "the window saw almost no sun"
    tail = sunlit[-200:]
    held = statistics.mean(tail)
    assert held < 5.0, f"pointing settled at {held:.1f} deg off-sun"
    assert result.final_omega_mag_rad_s < 0.01, "vehicle still moving after acquisition"


@pytest.mark.verifies("FSW-ADCS-013")
def test_eclipse_blinds_triad_honestly() -> None:
    """In shadow the estimator must refuse an attitude, never invent one."""
    result = run_fast_loop(
        _triad_only_vehicle(),
        duration_s=7200.0,
        omega0=(0.01, 0.0, 0.0),
        orbit_elements=_starlink(),
        dt_s=0.5,
    )
    assert any(result.in_eclipse), "this orbit must see shadow"
    dark_errors = [
        e for e, dark in zip(result.attitude_error_deg, result.in_eclipse, strict=True) if dark
    ]
    assert all(math.isnan(e) for e in dark_errors), "TRIAD claimed an attitude in the dark"
    lit_errors = [
        e for e, dark in zip(result.attitude_error_deg, result.in_eclipse, strict=True) if not dark
    ]
    valid_fraction = sum(1 for e in lit_errors if not math.isnan(e)) / len(lit_errors)
    assert valid_fraction > 0.9, f"only {valid_fraction:.0%} of sunlit steps had an attitude"


@pytest.mark.verifies("FSW-ADCS-015")
def test_nadir_pointing_acquires_earth_and_tracks_it() -> None:
    """flatsat_v1 as committed: +z onto Earth, tracked at orbit rate.

    Tracking nadir means rotating once per orbit, so the equilibrium
    carries a small steady offset (alignment torque balancing damping,
    ~1.3 deg at these gains) plus attitude-knowledge error. The bound
    is set above that PHYSICS, not tuned to flatter.
    """
    result = run_fast_loop(
        load_vehicle(),
        duration_s=3600.0,
        omega0=(0.05, -0.04, 0.03),
        sigma0=(0.2, -0.1, 0.35),
        orbit_elements=_starlink(),
        dt_s=0.2,
    )
    tail = [a for a in result.nadir_angle_deg[-500:] if not math.isnan(a)]
    assert len(tail) > 400
    held = statistics.mean(tail)
    assert held < 5.0, f"nadir pointing settled at {held:.1f} deg off-Earth"
    assert result.final_omega_mag_rad_s < 0.01


@pytest.mark.verifies("FSW-ADCS-015", "FSW-ADCS-016")
def test_star_tracker_keeps_earth_pointing_through_eclipse() -> None:
    """The payoff: attitude knowledge survives shadow, so nadir does too.

    With plain TRIAD the alignment pauses for the whole eclipse arc;
    with the star tracker the estimator ladder never loses attitude
    (the -z boresight keeps Earth out of view, and a sun inside the
    exclusion cone implies daylight — where TRIAD covers).
    """
    result = run_fast_loop(
        load_vehicle(),
        duration_s=7200.0,
        omega0=(0.02, -0.01, 0.015),
        orbit_elements=_starlink(),
        dt_s=0.5,
    )
    assert any(result.in_eclipse), "this orbit must see shadow"
    dark_errors = [
        e
        for e, dark in zip(result.attitude_error_deg, result.in_eclipse, strict=True)
        if dark and not math.isnan(e)
    ]
    total_dark = sum(1 for dark in result.in_eclipse if dark)
    assert len(dark_errors) > 0.9 * total_dark, "the tracker must carry attitude through shadow"
    assert statistics.median(dark_errors) < 0.1, "arcsecond-class device, sub-0.1 deg knowledge"
    settle = len(result.nadir_angle_deg) // 4
    tail = [a for a in result.nadir_angle_deg[settle:] if not math.isnan(a)]
    worst = max(tail)
    assert worst < 8.0, f"nadir pointing excursion {worst:.1f} deg — eclipse hole not closed"


def test_no_orbit_means_no_magnetic_authority() -> None:
    """Attitude-only runs must silence every magnetic device, not invent B."""
    result = run_fast_loop(
        load_vehicle(),
        duration_s=60.0,
        omega0=(0.1, -0.05, 0.02),
    )
    assert max(result.dipole_mag_a_m2) == 0.0, "dipole commanded in a universe with no field"
    assert result.final_omega_mag_rad_s < 0.05, "wheels alone should still damp the tumble"
