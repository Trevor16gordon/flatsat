"""Applications: thin process entry points that compose library pieces.

An app owns lifecycle, timing, and bus plumbing; it never contains device
knowledge or control math. Which driver or strategy it runs comes from the
vehicle composition file, resolved through ``flatsat.core.registry``.
"""
