"""Magnetorquer model: the dipole envelope is the only authority there is."""

import pytest

from flatsat.hardware import devices_pb2
from flatsat.hardware.models.magnetorquer import MagnetorquerModel
from flatsat.msgs import hal_pb2


def _spec(limit: float = 1.2) -> devices_pb2.MagnetorquerDevice:
    return devices_pb2.MagnetorquerDevice(name="mtq_test", max_dipole_a_m2=limit)


@pytest.mark.verifies("FSW-ACT-007")
def test_dipole_clips_and_flags_range() -> None:
    model = MagnetorquerModel(_spec(limit=1.2))
    flags = model.apply(5.0)
    assert flags & hal_pb2.VALIDITY_FLAG_RANGE
    assert model.applied_dipole_a_m2 == pytest.approx(1.2)
    flags = model.apply(-5.0)
    assert flags & hal_pb2.VALIDITY_FLAG_RANGE
    assert model.applied_dipole_a_m2 == pytest.approx(-1.2)


def test_in_envelope_command_applies_verbatim() -> None:
    model = MagnetorquerModel(_spec(limit=1.2))
    assert model.apply(0.7) == hal_pb2.VALIDITY_FLAG_VALID
    assert model.applied_dipole_a_m2 == pytest.approx(0.7)


@pytest.mark.verifies("FSW-ACT-007")
def test_spec_carries_no_torque_field() -> None:
    """The schema itself must not offer a torque envelope to fill.

    Writing max_torque into a magnetorquer spec is the documented
    failure mode (docs/ARCHITECTURE.md): it grants authority that does
    not exist. Pinning the field list keeps it from arriving quietly.
    """
    fields = {f.name for f in devices_pb2.MagnetorquerDevice.DESCRIPTOR.fields}
    assert fields == {"name", "max_dipole_a_m2"}


def test_state_message_reports_applied_dipole() -> None:
    model = MagnetorquerModel(_spec())
    model.apply(0.4)
    msg = model.state_message()
    assert msg.dipole_a_m2 == pytest.approx(0.4)
