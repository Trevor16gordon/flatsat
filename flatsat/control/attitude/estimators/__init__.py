"""State estimators: measurements in, state estimate out.

An estimator sits between sensor topics and the controller. ``passthrough``
is today's behavior made explicit; a real filter (EKF) is a config swap.
"""
