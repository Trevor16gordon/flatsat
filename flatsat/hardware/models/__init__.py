"""Sensor models: physics truth -> what a specific device would report.

Kept apart from drivers on purpose: a model describes a device's error
behavior (used by simulation), a driver talks to one (used in flight).
Both read the same device spec, so they cannot describe different parts.
"""
