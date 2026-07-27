"""Percentile summaries: bounded-memory windows for health telemetry."""

import pytest

from flatsat.core.health import health_topic, percentiles


def test_percentiles_summarize_a_window() -> None:
    summary = percentiles([1.0, 2.0, 3.0, 4.0, 100.0])
    assert summary.count == 5
    assert summary.max == pytest.approx(100.0)
    assert summary.p50 == pytest.approx(3.0)


def test_percentiles_of_empty_window_are_zero() -> None:
    summary = percentiles([])
    assert summary.count == 0 and summary.max == 0.0


def test_health_topic_prefixes_component() -> None:
    assert health_topic("adcs") == "health/adcs"
