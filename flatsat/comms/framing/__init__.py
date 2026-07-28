"""Framing: byte stream ⇄ delimited, integrity-checked packets.

Framing is the layer that survives a channel which delivers garbage,
partial packets, and bit flips. It is deliberately independent of the
PHY beneath it and the link above it, so every modem is compared under
IDENTICAL framing.
"""
