"""Shared pytest configuration for the flight test suite."""

import pytest


def pytest_configure(config: pytest.Config) -> None:
    """Register the requirements-traceability marker.

    Args:
        config: Pytest configuration object.
    """
    config.addinivalue_line(
        "markers",
        "verifies(*requirement_ids): this test verifies the given requirement IDs "
        "(see requirements/ and tools/traceability.py)",
    )
