"""A/B slots: the mechanism is uniform, the authority is gated."""

from pathlib import Path

import pytest

from flatsat.comms.slots import ActivationRefused, SlotManager
from flatsat.msgs import mode_pb2


def _staged(tmp_path: Path, version: str, content: bytes = b"weights") -> Path:
    path = tmp_path / "staging" / f"ml_policy@{version}"
    path.mkdir(parents=True, exist_ok=True)
    artifact = path / "artifact.bin"
    artifact.write_bytes(content)
    return artifact


def _manager(tmp_path: Path) -> SlotManager:
    return SlotManager(tmp_path / "slots")


@pytest.mark.verifies("FSW-UPL-003")
def test_activation_requires_ground_authority(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    with pytest.raises(ActivationRefused, match="ground authority"):
        manager.activate("ml_policy", "v1", _staged(tmp_path, "v1"), ground_authority=False)
    assert manager.state("ml_policy").active_version == ""
    assert manager.refused_activations == 1


@pytest.mark.verifies("FSW-UPL-004")
def test_activation_refused_in_safe(tmp_path: Path) -> None:
    """Safe mode's survival law is not displaced by an uplinked component."""
    manager = _manager(tmp_path)
    with pytest.raises(ActivationRefused, match="SAFE"):
        manager.activate(
            "ml_policy",
            "v1",
            _staged(tmp_path, "v1"),
            ground_authority=True,
            mode=mode_pb2.SYSTEM_MODE_SAFE,
        )
    assert manager.state("ml_policy").active_version == ""


@pytest.mark.verifies("FSW-UPL-005")
def test_unstaged_version_cannot_be_activated(tmp_path: Path) -> None:
    """Only bytes that arrived AND verified are eligible."""
    manager = _manager(tmp_path)
    with pytest.raises(ActivationRefused, match="not staged"):
        manager.activate("ml_policy", "v9", None, ground_authority=True)


@pytest.mark.verifies("FSW-UPL-003")
def test_authorized_activation_goes_live(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    slots = manager.activate(
        "ml_policy",
        "v1",
        _staged(tmp_path, "v1", b"first weights"),
        ground_authority=True,
        mode=mode_pb2.SYSTEM_MODE_NOMINAL,
    )
    assert slots.active_version == "v1"
    live = manager.active_path("ml_policy")
    assert live is not None and live.read_bytes() == b"first weights"


@pytest.mark.verifies("FSW-UPL-006")
def test_rollback_restores_the_previous_version(tmp_path: Path) -> None:
    """The C1 promise: a bad activation is one command from undone."""
    manager = _manager(tmp_path)
    manager.activate("ml_policy", "v1", _staged(tmp_path, "v1", b"good"), ground_authority=True)
    manager.activate(
        "ml_policy", "v2", _staged(tmp_path, "v2", b"regression"), ground_authority=True
    )
    live = manager.active_path("ml_policy")
    assert live is not None and live.read_bytes() == b"regression"

    slots = manager.rollback("ml_policy")
    assert slots.active_version == "v1"
    live = manager.active_path("ml_policy")
    assert live is not None and live.read_bytes() == b"good"


@pytest.mark.verifies("FSW-UPL-006")
def test_rollback_needs_no_authority_and_works_in_safe(tmp_path: Path) -> None:
    """Reverting to known-good is a toward-safety action."""
    manager = _manager(tmp_path)
    manager.activate("ml_policy", "v1", _staged(tmp_path, "v1"), ground_authority=True)
    manager.activate("ml_policy", "v2", _staged(tmp_path, "v2"), ground_authority=True)
    slots = manager.rollback("ml_policy")  # no authority argument exists at all
    assert slots.active_version == "v1"


def test_rollback_without_a_previous_version_fails_loud(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    manager.activate("ml_policy", "v1", _staged(tmp_path, "v1"), ground_authority=True)
    with pytest.raises(ActivationRefused, match="no previous version"):
        manager.rollback("ml_policy")


def test_slot_state_survives_a_restart(tmp_path: Path) -> None:
    """A reboot must know what is live — state is persisted, not in RAM."""
    manager = _manager(tmp_path)
    manager.activate("ml_policy", "v1", _staged(tmp_path, "v1"), ground_authority=True)
    manager.activate("ml_policy", "v2", _staged(tmp_path, "v2"), ground_authority=True)

    reborn = _manager(tmp_path)  # fresh process, same directory
    slots = reborn.state("ml_policy")
    assert slots.active_version == "v2"
    assert slots.previous_version == "v1"
    assert [state.name for state in reborn.all_states()] == ["ml_policy"]
