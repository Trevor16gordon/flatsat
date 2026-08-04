"""HIL contract: the bridge's safety sink and the shared-plant property.

Most tests import the bridge module directly (no Basilisk needed) — the
properties under test are the feedback-timeout rule,
plant-from-vehicle-file derivation, and the shared-environment helpers.
The tests that exercise real Basilisk dynamics skip themselves where
Basilisk is not installed (the flight computer, CI).
"""

import time

import numpy as np
import pytest
import zenoh

from flatsat.core.config import load_vehicle
from flatsat.msgs import sim_pb2
from flatsat.sim import orbit
from flatsat.sim.basilisk_hil import (
    BasiliskPlant,
    DipoleSink,
    WheelTorqueSink,
    fill_environment,
    magnetorquer_torque,
    mrp_to_dcm,
    plant_from_vehicle,
    rods_from_vehicle,
)


@pytest.mark.verifies("FSW-SIM-002")
def test_wheel_sink_zeroes_when_feedback_stops() -> None:
    """Defense in depth behind the daemon's own stale-command zeroing.

    Learned from a real run: when the flight loop exited, the sim held the
    final torque and spun the vehicle back up.
    """
    session = zenoh.open(zenoh.Config())
    sink = WheelTorqueSink(session, "wheel_sink_test", timeout_s=0.2)
    pub = session.declare_publisher("sim/wheel/wheel_sink_test/torque")

    msg = sim_pb2.WheelAxisTorque()
    msg.wheel = "wheel_sink_test"
    msg.torque_n_m = 0.01
    pub.put(msg.SerializeToString())

    deadline = time.monotonic() + 5.0
    while sink.messages_received == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert sink.messages_received >= 1, "feedback never arrived"
    assert sink.latest() == pytest.approx(0.01)

    time.sleep(0.35)  # exceed the timeout with no further feedback
    assert sink.latest() == 0.0, "stale feedback must not keep torquing the plant"
    session.close()


def test_sink_with_no_feedback_ever_is_zero() -> None:
    session = zenoh.open(zenoh.Config())
    sink = WheelTorqueSink(session, "wheel_never_test", timeout_s=10.0)
    assert sink.latest() == 0.0
    session.close()


@pytest.mark.verifies("FSW-SIM-003")
def test_plant_is_built_from_the_vehicle_file() -> None:
    """Sim/flight agreement by construction: same file, same numbers."""
    vehicle = load_vehicle()
    mass_kg, inertia, wheels = plant_from_vehicle(vehicle)

    body = vehicle.require_body()
    flat = list(body.inertia_kg_m2)
    assert mass_kg == body.mass_kg
    assert inertia == [flat[0:3], flat[3:6], flat[6:9]]
    declared_wheels = [
        (a.name, tuple(a.mounting.axis))
        for a in vehicle.actuators
        if a.WhichOneof("options") in ("sim_reaction_wheel", "basilisk_reaction_wheel")
    ]
    assert wheels == declared_wheels
    assert wheels, "default vehicle must declare at least one wheel"


# ----------------------------------------------------------- magnetorquers --


def test_wheels_and_rods_are_split_by_driver_kind() -> None:
    """A rod must never be summed into the plant as a wheel."""
    vehicle = load_vehicle("config/vehicles/test_scenario_bdot.txtpb")
    _, _, wheels = plant_from_vehicle(vehicle)
    rods = rods_from_vehicle(vehicle)
    assert wheels == [], "the bdot vehicle declares no wheels"
    assert [name for name, _ in rods] == ["bdt_mtq_x", "bdt_mtq_y", "bdt_mtq_z"]


@pytest.mark.verifies("FSW-ACT-007")
def test_magnetorquer_torque_is_m_cross_b() -> None:
    """Torque comes from the field, is zero along it, and scales with it.

    This function IS the architecture note "a magnetorquer has no maximum
    torque": halve the field and the same dipole produces half the
    torque; align the dipole with the field and it produces none.
    """
    rods = [((1.0, 0.0, 0.0), 2.0)]  # 2 A·m² along body x
    field = (0.0, 2.6e-5, 0.0)
    torque = magnetorquer_torque(rods, field)
    assert torque == pytest.approx((0.0, 0.0, 2.0 * 2.6e-5))
    # No torque about the field line — underactuated at every instant.
    aligned = magnetorquer_torque([((0.0, 1.0, 0.0), 2.0)], field)
    assert aligned == pytest.approx((0.0, 0.0, 0.0))
    # Half the field, half the authority: the environment IS the envelope.
    half = magnetorquer_torque(rods, (0.0, 1.3e-5, 0.0))
    assert half[2] == pytest.approx(torque[2] / 2.0)


def test_dipole_sink_zeroes_when_feedback_stops() -> None:
    """Same defense-in-depth as the wheel sink, for rods."""
    session = zenoh.open(zenoh.Config())
    sink = DipoleSink(session, "mtq_sink_test", timeout_s=0.2)
    pub = session.declare_publisher("sim/mtq/mtq_sink_test/dipole")

    msg = sim_pb2.MagnetorquerDipole()
    msg.rod = "mtq_sink_test"
    msg.dipole_a_m2 = 1.5
    pub.put(msg.SerializeToString())

    deadline = time.monotonic() + 5.0
    while sink.messages_received == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert sink.messages_received >= 1, "feedback never arrived"
    assert sink.latest() == pytest.approx(1.5)

    time.sleep(0.35)  # exceed the timeout with no further feedback
    assert sink.latest() == 0.0, "stale feedback must not keep torquing the plant"
    session.close()


# ------------------------------------------------------ shared environment --


def test_fill_environment_uses_the_shared_orbit_models() -> None:
    """Position copied, field rotated (not scaled), eclipse from orbit.py.

    Both plants call this helper, so pinning it to the orbit module's
    own outputs pins the plant-parity property: whatever integrates the
    motion, the universe the vehicle sees is the same one.
    """
    position = np.array([orbit.R_EARTH_M + 525e3, 0.0, 0.0])
    truth = sim_pb2.TruthState()
    fill_environment(truth, position, mrp_to_dcm((0.2, -0.1, 0.35)), 0.0, 0.0, 0.0)

    assert truth.position_x_m == pytest.approx(float(position[0]))
    assert truth.position_y_m == 0.0
    field_body = (truth.mag_field_x_t, truth.mag_field_y_t, truth.mag_field_z_t)
    expected = float(np.linalg.norm(orbit.magnetic_field_eci(position, 0.0, 0.0)))
    assert float(np.linalg.norm(field_body)) == pytest.approx(expected, rel=1e-12)
    assert not truth.in_eclipse, "sunward of the Earth at solar angle 0 must be lit"

    shadowed = sim_pb2.TruthState()
    fill_environment(shadowed, -position, mrp_to_dcm((0.0, 0.0, 0.0)), 0.0, 0.0, 0.0)
    assert shadowed.in_eclipse, "antisunward at LEO radius must be shadowed"


def test_mrp_to_dcm_matches_basilisk() -> None:
    """The shared rotation must agree with Basilisk's own kinematics.

    The local plant rotates the field with :func:`mrp_to_dcm`; Basilisk
    integrates attitude with its own MRP conventions. If the two ever
    disagreed on what an MRP means, the same TruthState field would
    point different ways under the two plants.
    """
    pytest.importorskip("Basilisk")
    from Basilisk.utilities import RigidBodyKinematics

    for sigma in ((0.0, 0.0, 0.0), (0.2, -0.1, 0.35), (-0.6, 0.3, 0.5)):
        ours = np.array(mrp_to_dcm(sigma))
        theirs = np.array(RigidBodyKinematics.MRP2C(list(sigma)))
        assert np.allclose(ours, theirs, atol=1e-12), f"DCM mismatch at sigma {sigma}"


def test_basilisk_plant_flies_the_orbit_and_reports_the_environment() -> None:
    """The gap this pins closed: Basilisk gets gravity, orbit, sigma0, field.

    A short real-time run (the plant paces to wall clock) is enough to
    assert presence and initial parity: the vehicle is AT the orbit
    radius and moving, the attitude was not discarded, and the field is
    LEO-magnitude — an empty-universe regression fails every one.
    """
    pytest.importorskip("Basilisk")
    vehicle = load_vehicle()
    inclination = orbit.sun_synchronous_inclination_rad(525e3)
    elements = orbit.circular(525e3, inclination)
    sigma0 = (0.2, -0.1, 0.35)

    session = zenoh.open(zenoh.Config())
    received: list[sim_pb2.TruthState] = []

    def on_truth(sample: zenoh.Sample) -> None:
        """Collect published truth.

        Args:
            sample: Incoming TruthState.
        """
        received.append(sim_pb2.TruthState.FromString(bytes(sample.payload.to_bytes())))

    topic = "test/sim/basilisk_orbit/truth"
    _sub = session.declare_subscriber(topic, on_truth)
    plant = BasiliskPlant(
        vehicle,
        session,
        truth_topic=topic,
        omega0=(0.05, -0.04, 0.03),
        rate_hz=200.0,
        sigma0=sigma0,
        orbit_elements=elements,
        report_every_s=0.0,
    )
    plant.start()
    try:
        deadline = time.monotonic() + 15.0
        while len(received) < 100 and time.monotonic() < deadline:
            time.sleep(0.02)
    finally:
        plant.stop()
        session.close()
    assert len(received) >= 100, "plant published too little truth"

    first, last = received[0], received[-1]
    # sigma0 threaded through, not discarded (rates barely move it early on).
    assert first.sigma_x == pytest.approx(sigma0[0], abs=0.02)
    assert first.sigma_y == pytest.approx(sigma0[1], abs=0.02)
    assert first.sigma_z == pytest.approx(sigma0[2], abs=0.02)
    # At the orbit radius, and staying there.
    for state in (first, last):
        radius = float(np.linalg.norm([state.position_x_m, state.position_y_m, state.position_z_m]))
        assert radius - orbit.R_EARTH_M == pytest.approx(525e3, abs=20e3)
        field = float(
            np.linalg.norm([state.mag_field_x_t, state.mag_field_y_t, state.mag_field_z_t])
        )
        assert 15e-6 < field < 60e-6, "field is not LEO-magnitude"
    # Actually moving along the orbit, not parked: ~7.6 km/s at 525 km.
    displacement = float(
        np.linalg.norm(
            [
                last.position_x_m - first.position_x_m,
                last.position_y_m - first.position_y_m,
                last.position_z_m - first.position_z_m,
            ]
        )
    )
    assert displacement > 1e3, "vehicle is not orbiting"
    assert not first.in_eclipse, "epoch point is sunward of the Earth at solar angle 0"


def test_sun_gauge_extractors_read_the_css_sample() -> None:
    """The Vizard sun gauges: visibility flag and off-axis angle."""
    import math

    from flatsat.msgs import hal_pb2
    from flatsat.sim.basilisk_hil import _sun_off_axis_deg, _sun_visible

    msg = hal_pb2.SunSensorSample()
    msg.sun_x, msg.sun_y, msg.sun_z = 1.0, 0.0, 0.0
    msg.sun_visible = True
    lit = msg.SerializeToString()
    assert _sun_visible(lit) == 1.0
    # +z axis vs a +x sun: 90 degrees off.
    assert _sun_off_axis_deg((0.0, 0.0, 1.0))(lit) == pytest.approx(90.0)
    assert _sun_off_axis_deg((2.0, 0.0, 0.0))(lit) == pytest.approx(0.0)
    dark = hal_pb2.SunSensorSample().SerializeToString()
    assert _sun_visible(dark) == 0.0
    assert _sun_off_axis_deg((0.0, 0.0, 1.0))(dark) == 0.0, "darkness has no angle"
    assert math.isfinite(_sun_off_axis_deg((0.0, 0.0, 1.0))(lit))


def test_sun_point_axis_read_from_the_vehicle() -> None:
    """The off-sun gauge exists only when the vehicle flies sun_point."""
    from flatsat.core.config import load_vehicle
    from flatsat.sim.basilisk_hil import _sun_point_axis

    vehicle = load_vehicle()  # flatsat_v1 flies sun_point with +z
    assert _sun_point_axis(vehicle) == (0.0, 0.0, 1.0)
    vehicle.control.rate_damping.SetInParent()  # switch the strategy oneof
    assert _sun_point_axis(vehicle) is None
