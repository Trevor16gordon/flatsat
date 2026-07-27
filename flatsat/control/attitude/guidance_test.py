"""Guidance: the reference source is independent of the control strategy."""

import pytest

from flatsat.control.attitude.guidance import ConstantRateReference


@pytest.mark.verifies("FSW-ADCS-008")
def test_guidance_supplies_the_reference() -> None:
    detumble = ConstantRateReference.from_config({})
    assert detumble.reference_at(0.0).body_rates_rad_s == (0.0, 0.0, 0.0)
    hold = ConstantRateReference.from_config({"target_rates_rad_s": [0.1, 0.0, -0.2]})
    assert hold.reference_at(123.0).body_rates_rad_s == (0.1, 0.0, -0.2)


def test_constant_reference_ignores_time() -> None:
    hold = ConstantRateReference.from_config({"target_rates_rad_s": [0.1, 0.2, 0.3]})
    assert hold.reference_at(0.0) == hold.reference_at(1e6)


def test_describe_names_the_objective() -> None:
    assert "detumble" in ConstantRateReference.from_config({}).describe()[0]
