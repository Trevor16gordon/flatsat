"""Where the spacecraft is, and what the magnetic field is there.

Attitude dynamics alone are enough to detumble with reaction wheels,
which is why this did not exist before. Magnetic control is different:
a magnetorquer produces torque ``m x B``, so it needs a magnetic field,
which needs a position in an orbit, which needs an epoch. That chain is
what this module supplies.

Deliberately pure Python and NumPy, with no Basilisk. Basilisk is not
installed on the flight computer, and physics that can only be exercised
on one machine is physics that stops being tested. Basilisk remains the
high-fidelity plant; this is the model that runs everywhere, including
CI, and it is accurate enough for the questions a detumble controller
asks.

What is modelled, and what is not:

  * **Two-body Keplerian motion, with J2 nodal regression** applied to
    the ascending node. J2 is the dominant perturbation in LEO and the
    reason sun-synchronous orbits exist at all — leaving it out would
    make an SSO's whole defining property vanish. Drag, third bodies and
    higher harmonics are absent.
  * **A tilted dipole geomagnetic field.** The real field is lumpy —
    IGRF needs a hundred-odd coefficients and still misses the South
    Atlantic Anomaly's detail. A dipole gets the magnitude right to
    roughly 10-20% and, more importantly, gets the ROTATION of the field
    along the orbit right, which is the part B-dot detumble actually
    depends on. If a controller only works against a perfect dipole, it
    does not work.

Frames: ECI here means an Earth-centred inertial frame whose z is the
rotation axis and whose x is the ascending node reference. It is not
J2000 — there is no precession, nutation or polar motion. Everything is
self-consistent within this module, which is what the simulation needs;
it is not suitable for pointing a real antenna at a real satellite.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

MU_EARTH_M3_S2 = 3.986004418e14  # geocentric gravitational constant
R_EARTH_M = 6_378_137.0  # equatorial radius, WGS-84
J2 = 1.08262668e-3  # second zonal harmonic
EARTH_ROTATION_RAD_S = 7.2921159e-5  # sidereal, not solar
# Equatorial field magnitude at the Earth's surface. The dipole moment
# is this times R^3; expressing it this way makes the surface value
# checkable by inspection against the ~31 uT everyone quotes.
B0_TESLA = 3.12e-5
DIPOLE_TILT_RAD = math.radians(11.5)  # geomagnetic pole offset from the spin axis


@dataclass(frozen=True)
class OrbitalElements:
    """Classical elements at an epoch.

    Attributes:
        semi_major_axis_m: Orbit size, from the Earth's centre.
        eccentricity: 0 is circular; this module assumes small values.
        inclination_rad: Tilt from the equator.
        raan_rad: Right ascension of the ascending node, at epoch.
        arg_periapsis_rad: Argument of periapsis.
        true_anomaly_rad: Position along the orbit, at epoch.
    """

    semi_major_axis_m: float
    eccentricity: float
    inclination_rad: float
    raan_rad: float
    arg_periapsis_rad: float
    true_anomaly_rad: float

    @property
    def period_s(self) -> float:
        """Orbital period from Kepler's third law."""
        return 2.0 * math.pi * math.sqrt(self.semi_major_axis_m**3 / MU_EARTH_M3_S2)

    @property
    def altitude_m(self) -> float:
        """Altitude above the equatorial radius, for a circular orbit."""
        return self.semi_major_axis_m - R_EARTH_M


def circular(altitude_m: float, inclination_rad: float, raan_rad: float = 0.0) -> OrbitalElements:
    """Build a circular orbit at an altitude.

    Args:
        altitude_m: Height above the equatorial radius.
        inclination_rad: Orbit tilt.
        raan_rad: Ascending node at epoch.

    Returns:
        The elements.
    """
    return OrbitalElements(
        semi_major_axis_m=R_EARTH_M + altitude_m,
        eccentricity=0.0,
        inclination_rad=inclination_rad,
        raan_rad=raan_rad,
        arg_periapsis_rad=0.0,
        true_anomaly_rad=0.0,
    )


def sun_synchronous_inclination_rad(altitude_m: float, eccentricity: float = 0.0) -> float:
    """Inclination whose J2 nodal drift matches the Earth's orbit of the Sun.

    A sun-synchronous orbit is not a special altitude — it is the
    inclination at which the node precesses 360 degrees per year, so the
    local solar time of every pass stays fixed. That is why SSO
    inclinations are always just past 90 degrees: the drift has to be
    eastward, which needs a retrograde orbit.

    Args:
        altitude_m: Height above the equatorial radius.
        eccentricity: Orbit eccentricity.

    Returns:
        The required inclination in radians.

    Raises:
        ValueError: If no inclination can produce the required drift at
            this altitude, which happens well above LEO.
    """
    a = R_EARTH_M + altitude_m
    n = math.sqrt(MU_EARTH_M3_S2 / a**3)
    target_drift = 2.0 * math.pi / 365.2422 / 86400.0  # rad/s, one turn per year
    # RAAN_dot = -3/2 n J2 (Re/p)^2 cos(i)
    p = a * (1.0 - eccentricity**2)
    factor = -1.5 * n * J2 * (R_EARTH_M / p) ** 2
    cos_i = target_drift / factor
    if not -1.0 <= cos_i <= 1.0:
        raise ValueError(f"no sun-synchronous inclination at {altitude_m / 1e3:.0f} km")
    return math.acos(cos_i)


def raan_drift_rad_s(elements: OrbitalElements) -> float:
    """J2 nodal regression rate.

    Args:
        elements: The orbit.

    Returns:
        Rate of change of RAAN, radians per second. Negative (westward)
        for prograde orbits, positive for retrograde ones.
    """
    a = elements.semi_major_axis_m
    n = math.sqrt(MU_EARTH_M3_S2 / a**3)
    p = a * (1.0 - elements.eccentricity**2)
    return -1.5 * n * J2 * (R_EARTH_M / p) ** 2 * math.cos(elements.inclination_rad)


def _eccentric_from_mean(mean_anomaly: float, eccentricity: float) -> float:
    """Solve Kepler's equation by Newton iteration.

    Args:
        mean_anomaly: Mean anomaly, radians.
        eccentricity: Orbit eccentricity.

    Returns:
        Eccentric anomaly, radians.
    """
    e = eccentricity
    guess = mean_anomaly if e < 0.8 else math.pi
    for _ in range(24):
        error = guess - e * math.sin(guess) - mean_anomaly
        derivative = 1.0 - e * math.cos(guess)
        step = error / derivative
        guess -= step
        if abs(step) < 1e-13:
            break
    return guess


def propagate_eci(elements: OrbitalElements, t_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Position and velocity at a time after epoch.

    Args:
        elements: Elements at epoch.
        t_s: Seconds since epoch.

    Returns:
        Tuple of (position, velocity) in metres and metres per second,
        in the inertial frame described in the module docstring.
    """
    a = elements.semi_major_axis_m
    e = elements.eccentricity
    n = math.sqrt(MU_EARTH_M3_S2 / a**3)

    # Mean anomaly at epoch, advanced to now.
    nu0 = elements.true_anomaly_rad
    eccentric0 = 2.0 * math.atan2(
        math.sqrt(1.0 - e) * math.sin(nu0 / 2.0), math.sqrt(1.0 + e) * math.cos(nu0 / 2.0)
    )
    mean0 = eccentric0 - e * math.sin(eccentric0)
    eccentric = _eccentric_from_mean(mean0 + n * t_s, e)

    # In-plane position and velocity (perifocal frame).
    cos_ecc, sin_ecc = math.cos(eccentric), math.sin(eccentric)
    r = a * (1.0 - e * cos_ecc)
    pos_pf = np.array([a * (cos_ecc - e), a * math.sqrt(1.0 - e**2) * sin_ecc, 0.0])
    rate = math.sqrt(MU_EARTH_M3_S2 * a) / r
    vel_pf = np.array([-rate * sin_ecc, rate * math.sqrt(1.0 - e**2) * cos_ecc, 0.0])

    # The node has moved since epoch: J2 is what makes an SSO work, so
    # ignoring it here would quietly delete the property being simulated.
    raan = elements.raan_rad + raan_drift_rad_s(elements) * t_s
    rotation = _perifocal_to_inertial(raan, elements.inclination_rad, elements.arg_periapsis_rad)
    return rotation @ pos_pf, rotation @ vel_pf


def _perifocal_to_inertial(raan: float, inclination: float, arg_periapsis: float) -> np.ndarray:
    """Rotation from the orbital plane into the inertial frame.

    Args:
        raan: Right ascension of the ascending node.
        inclination: Orbit tilt.
        arg_periapsis: Argument of periapsis.

    Returns:
        A 3x3 rotation matrix.
    """
    cos_o, sin_o = math.cos(raan), math.sin(raan)
    cos_i, sin_i = math.cos(inclination), math.sin(inclination)
    cos_w, sin_w = math.cos(arg_periapsis), math.sin(arg_periapsis)
    return np.array(
        [
            [
                cos_o * cos_w - sin_o * sin_w * cos_i,
                -cos_o * sin_w - sin_o * cos_w * cos_i,
                sin_o * sin_i,
            ],
            [
                sin_o * cos_w + cos_o * sin_w * cos_i,
                -sin_o * sin_w + cos_o * cos_w * cos_i,
                -cos_o * sin_i,
            ],
            [sin_w * sin_i, cos_w * sin_i, cos_i],
        ]
    )


def dipole_axis_eci(t_s: float, epoch_gmst_rad: float = 0.0) -> np.ndarray:
    """Unit vector along the geomagnetic dipole, in the inertial frame.

    The dipole is tilted from the spin axis and rotates with the Earth,
    which is precisely why B-dot detumble works: an inertially fixed
    spacecraft still sees a field that turns.

    Args:
        t_s: Seconds since epoch.
        epoch_gmst_rad: Greenwich sidereal angle at epoch.

    Returns:
        Unit vector, inertial frame.
    """
    theta = epoch_gmst_rad + EARTH_ROTATION_RAD_S * t_s
    return np.array(
        [
            math.sin(DIPOLE_TILT_RAD) * math.cos(theta),
            math.sin(DIPOLE_TILT_RAD) * math.sin(theta),
            math.cos(DIPOLE_TILT_RAD),
        ]
    )


def magnetic_field_eci(
    position_eci_m: np.ndarray, t_s: float, epoch_gmst_rad: float = 0.0
) -> np.ndarray:
    """Geomagnetic field at a point, from a tilted dipole.

    ``B = B0 (Re/r)^3 [3 (m.r) r - m]`` with unit vectors, which gives
    B0 at the magnetic equator and 2*B0 over the poles — the standard
    factor-of-two that makes this model easy to sanity check.

    Args:
        position_eci_m: Position in the inertial frame.
        t_s: Seconds since epoch.
        epoch_gmst_rad: Greenwich sidereal angle at epoch.

    Returns:
        Field vector in tesla, inertial frame.
    """
    r = float(np.linalg.norm(position_eci_m))
    if r < 1.0:
        return np.zeros(3)
    r_hat = position_eci_m / r
    m_hat = dipole_axis_eci(t_s, epoch_gmst_rad)
    scale = B0_TESLA * (R_EARTH_M / r) ** 3
    return scale * (3.0 * float(np.dot(m_hat, r_hat)) * r_hat - m_hat)


def sun_direction_eci(t_s: float, epoch_solar_angle_rad: float = 0.0) -> np.ndarray:
    """Unit vector towards the Sun.

    Circular Earth orbit in the ecliptic, which is enough for the
    questions this answers: is a star tracker blinded, is the spacecraft
    in daylight. It is not an ephemeris.

    Args:
        t_s: Seconds since epoch.
        epoch_solar_angle_rad: Solar longitude at epoch.

    Returns:
        Unit vector, inertial frame.
    """
    obliquity = math.radians(23.44)
    angle = epoch_solar_angle_rad + 2.0 * math.pi * t_s / (365.2422 * 86400.0)
    return np.array(
        [
            math.cos(angle),
            math.sin(angle) * math.cos(obliquity),
            math.sin(angle) * math.sin(obliquity),
        ]
    )


def in_eclipse(position_eci_m: np.ndarray, sun_hat: np.ndarray) -> bool:
    """Whether the Earth blocks the Sun from this position.

    A cylindrical shadow: no penumbra, no atmospheric refraction. The
    error is a few seconds either side of a terminator crossing.

    Args:
        position_eci_m: Position in the inertial frame.
        sun_hat: Unit vector towards the Sun.

    Returns:
        True when the spacecraft is in shadow.
    """
    along = float(np.dot(position_eci_m, sun_hat))
    if along > 0.0:
        return False  # sunward of the Earth's centre: always lit
    perpendicular = position_eci_m - along * sun_hat
    return bool(np.linalg.norm(perpendicular) < R_EARTH_M)
