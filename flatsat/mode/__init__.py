"""System mode: the mission state machine and its distribution.

Four flat states — Init → Nominal ⇄ (Safe → Recovery) — with the
asymmetry that IS the philosophy: toward safety is automatic and
software-triggerable from anywhere; away from safety is
ground-command-only. Small, boring, dependency-minimal (imports core
only): Safe's entry path must not depend on anything that can be the
reason for entering it.
"""
