# radio/ — shared PHY (Pluto bring-up, flowgraphs, modem work)

## RF safety — read first

- **Never TX→RX without the 30 dB of SMA pads inline.** Pluto max TX ≈ +7 dBm;
  through 30 dB that arrives at ≈ −23 dBm, ~25 dB below the AD9364's ~+2.5 dBm
  RX damage threshold.
- Cabled only; nothing transmits over the air.
- Scripts that transmit must say so loudly and be separate from RX-only tools.

## Radios

| Role | Serial | Host |
|------|--------|------|
| Flat-sat radio | `104473b04a06000602001c00dd1f84cfaa` | Jetson |
| Ground radio | *(record when unboxed)* | Mac (future) |

Firmware v0.37 on the flat-sat unit; both units flash to a common version
(v0.39) before any comparative baseline (see PLAN.md decision log).

## Contents

| File | TX? | What |
|------|-----|------|
| `pluto_smoke_test.py` | **No — RX only** | P1 smoke test: URI resolve, RX flowgraph builds, IQ streams, front-end health metrics. Run with system `python3` (apt GNU Radio). |
