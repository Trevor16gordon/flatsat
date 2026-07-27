"""Helpers for publishing process health telemetry.

Health messages are ordinary bus traffic: stamped by the same
:class:`~flatsat.core.bus.SamplePublisher` as sensor data, so a recorder,
FDIR limit checker, or regression tool consumes them with no special case.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from flatsat.msgs import health_pb2

HEALTH_TOPIC_PREFIX = "health"


def percentiles(samples: Sequence[float]) -> health_pb2.Percentiles:
    """Summarize a per-cycle measurement as a Percentiles message.

    Args:
        samples: Per-cycle values collected over one reporting window.

    Returns:
        The distribution summary; all zeros for an empty window.
    """
    out = health_pb2.Percentiles()
    if not samples:
        return out
    data = np.asarray(samples, dtype=np.float64)
    out.p50 = float(np.percentile(data, 50))
    out.p99 = float(np.percentile(data, 99))
    out.p99_9 = float(np.percentile(data, 99.9))
    out.max = float(data.max())
    out.count = int(data.size)
    return out


def health_topic(component: str) -> str:
    """Build the health topic for a component.

    Args:
        component: Component name, e.g. ``adcs`` or a sensor's name.

    Returns:
        The bus key expression to publish health on.
    """
    return f"{HEALTH_TOPIC_PREFIX}/{component}"
