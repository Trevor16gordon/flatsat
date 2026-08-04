"""Load a saved orbit file into elements + epoch angles.

Split out of the scenario runner because the ORBIT is not a scenario
concern: the HIL bridge flies one, and the TRIAD estimator carries one
as its onboard model. Both need to resolve a ``config/orbits/*.txtpb``
without dragging in the mission machinery.
"""

from __future__ import annotations

import math
from pathlib import Path

from flatsat.core.config import load_textproto
from flatsat.sim import mission_pb2, orbit


def load_orbit(path: Path | str) -> tuple[orbit.OrbitalElements, float, float]:
    """Load a saved orbit, resolving intent into numbers.

    An unset inclination means "sun-synchronous at this altitude", so it
    is computed rather than copied. That keeps the property the mission
    actually wants — fixed local solar time — true by construction if
    the altitude ever changes.

    Args:
        path: Orbit file, e.g. ``config/orbits/spacex_rideshare_sso.txtpb``.

    Returns:
        Tuple of (elements, epoch GMST radians, epoch solar angle radians).
    """
    cfg = mission_pb2.OrbitConfig()
    load_textproto(path, cfg)
    inclination = (
        math.radians(cfg.inclination_deg)
        if cfg.HasField("inclination_deg")
        else orbit.sun_synchronous_inclination_rad(cfg.altitude_m, cfg.eccentricity)
    )
    elements = orbit.OrbitalElements(
        semi_major_axis_m=orbit.R_EARTH_M + cfg.altitude_m,
        eccentricity=cfg.eccentricity,
        inclination_rad=inclination,
        raan_rad=math.radians(cfg.raan_deg),
        arg_periapsis_rad=math.radians(cfg.arg_periapsis_deg),
        true_anomaly_rad=math.radians(cfg.true_anomaly_deg),
    )
    return elements, math.radians(cfg.epoch_gmst_deg), math.radians(cfg.epoch_solar_angle_deg)
