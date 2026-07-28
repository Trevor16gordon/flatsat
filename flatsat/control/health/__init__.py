"""FDIR: fault detection, isolation, and recovery — the system as plant.

The same sense-decide-act shape as attitude control: declarative limit
rules estimate health (sense), the arbiter decides, and the response
ladder actuates — process restart is systemd's tier, and "request Safe"
is FDIR acting as a CLIENT of mode authority, never its owner. After
anything unexpected the system rests in its safest configuration; only
deliberate human action makes it interesting again.
"""
