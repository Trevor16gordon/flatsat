"""Comms: the managed pipe to the ground, layer by swappable layer.

The space link is intermittent, asymmetric, high-BER, and mostly absent,
so the middleware is never tunneled over it (PLAN §6). Instead:

    bus topics -> link service -> segments -> framing -> MODEM -> RF
    RF -> MODEM -> framing -> segments -> reassembly -> ground bus

Each layer is a contract with named implementations selected by config,
exactly like sensors and controllers: a Pluto GMSK modem, a lossy
in-process loopback, and later a learned receiver are peers under
IDENTICAL framing — which is what makes the learned-vs-classical
comparison honest.

RF safety is structural: any modem that can transmit must refuse to do
so unless its configuration carries an explicit acknowledgement.
"""
