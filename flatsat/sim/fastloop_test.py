"""Faster-than-real-time law verification against REAL-scale vehicles.

The scenario tier flies toy vehicles because it runs at wall clock;
these tests fly the actual flight vehicle through hours of sim time in
seconds of CPU — the check the user actually wants: do the control
algorithms work at the scales the hardware really has?
"""

import time

import pytest

from flatsat.core.config import load_vehicle
from flatsat.sim import orbit
from flatsat.sim.fastloop import run_fast_loop

STARLINK_INCLINATION_RAD = 0.9250245035569946  # 53 degrees


def _starlink() -> orbit.OrbitalElements:
    return orbit.circular(550e3, STARLINK_INCLINATION_RAD)


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


def test_no_orbit_means_no_magnetic_authority() -> None:
    """Attitude-only runs must silence every magnetic device, not invent B."""
    result = run_fast_loop(
        load_vehicle(),
        duration_s=60.0,
        omega0=(0.1, -0.05, 0.02),
    )
    assert max(result.dipole_mag_a_m2) == 0.0, "dipole commanded in a universe with no field"
    assert result.final_omega_mag_rad_s < 0.05, "wheels alone should still damp the tumble"
