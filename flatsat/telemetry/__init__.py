"""Telemetry: recording the bus, verbatim, with bounded disk.

The archive is the raw material every later consumer reads — regression
comparison, FDIR analysis, and the ML corpus. Payloads are stored
byte-exact and unparsed: what the bus carried IS the record.
"""
