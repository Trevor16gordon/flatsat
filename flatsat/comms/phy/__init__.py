"""Concrete modems — one module per radio, real or simulated.

Registered in ``flatsat.core.registry`` and selected by the vehicle
file's comms block. ``loopback`` needs no hardware and can corrupt on
demand (CI, BER campaigns); ``pluto_gmsk`` is the proven Pluto baseline
and refuses to transmit without an explicit acknowledgement.
"""
