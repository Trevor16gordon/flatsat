"""flatsat: the flight-software package — the Tier-1 versionable artifact.

Everything that ships to a flight computer lives inside this package;
everything outside it is data (``config/``), contracts (``protos/``,
``requirements/``), or tooling (``tools/``). Where code RUNS is
deployment's decision (``deployment.toml`` + host profiles), never the
tree's.

Import rule: every domain may import ``flatsat.core``; ``core`` imports no
domain. Applications never import a concrete driver or controller — they
resolve one by name through ``flatsat.core.registry``.
"""
