"""The FDIR limit checker — pure rule evaluation, no bus, no clock owned.

Each monitor wraps one declarative rule from the vehicle file's fdir
block: it is fed message arrivals (time + validity flags) and answers
"is your condition holding right now?". Persistence (``min_trip_s``)
wraps every check so a single bad sample never trips a rule — the HIL
stall census showed sub-second staleness is NORMAL on this link.

Time comes in as arguments; the caller owns the clock, which is what
makes every rule unit-testable without sleeping.
"""

from __future__ import annotations

from collections import deque

from flatsat.control.health import fdir_config_pb2
from flatsat.core.config import which_impl


class FlagRatioMonitor:
    """Fraction of flagged samples over a sliding window."""

    def __init__(self, rule: fdir_config_pb2.FlagRatioRule) -> None:
        """Bind the check parameters.

        Args:
            rule: The declarative check.
        """
        self._mask = rule.flag_mask
        self._window_ns = int(rule.window_s * 1e9)
        self._max_ratio = rule.max_ratio
        self._samples: deque[tuple[int, bool]] = deque()

    def on_message(self, now_ns: int, validity: int) -> None:
        """Fold one message arrival into the window.

        Args:
            now_ns: Arrival time (monotonic).
            validity: The message header's validity flags.
        """
        self._samples.append((now_ns, bool(validity & self._mask)))

    def condition(self, now_ns: int) -> bool:
        """Whether the flagged ratio currently exceeds the bound.

        Args:
            now_ns: Current monotonic time.

        Returns:
            True when the window's flagged fraction exceeds max_ratio.
            An empty window is False — silence is SilenceRule's fault,
            not this one's.
        """
        horizon = now_ns - self._window_ns
        while self._samples and self._samples[0][0] < horizon:
            self._samples.popleft()
        if not self._samples:
            return False
        flagged = sum(1 for _, is_flagged in self._samples if is_flagged)
        return flagged / len(self._samples) > self._max_ratio


class SilenceMonitor:
    """Nothing on the topic for too long — the process-death signature."""

    def __init__(self, rule: fdir_config_pb2.SilenceRule, start_ns: int) -> None:
        """Arm from the start time — a topic that NEVER speaks is silent.

        Args:
            rule: The declarative check.
            start_ns: Monotonic time monitoring began.
        """
        self._timeout_ns = int(rule.timeout_s * 1e9)
        self._last_ns = start_ns

    def on_message(self, now_ns: int, validity: int) -> None:
        """Note one message arrival (flags irrelevant — sound is sound).

        Args:
            now_ns: Arrival time (monotonic).
            validity: Unused.
        """
        self._last_ns = now_ns

    def condition(self, now_ns: int) -> bool:
        """Whether the topic has been silent beyond the timeout.

        Args:
            now_ns: Current monotonic time.

        Returns:
            True when silent too long.
        """
        return now_ns - self._last_ns > self._timeout_ns


class RuleMonitor:
    """One named rule: a check wrapped in trip persistence."""

    def __init__(self, rule: fdir_config_pb2.FdirRule, start_ns: int) -> None:
        """Build the monitor for one declarative rule.

        Args:
            rule: The rule from the vehicle file.
            start_ns: Monotonic time monitoring began.

        Raises:
            ValueError: If the rule selects no check (fail loud at start).
        """
        self.name = rule.name
        kind = which_impl(rule, "check", f"fdir rule {rule.name!r}")
        if kind == "flag_ratio":
            self.topic: str = rule.flag_ratio.topic
            self._check: FlagRatioMonitor | SilenceMonitor = FlagRatioMonitor(rule.flag_ratio)
        else:
            self.topic = rule.silence.topic
            self._check = SilenceMonitor(rule.silence, start_ns)
        self._min_trip_ns = int((rule.min_trip_s if rule.HasField("min_trip_s") else 1.0) * 1e9)
        self._condition_since_ns: int | None = None

    def on_message(self, now_ns: int, validity: int) -> None:
        """Feed one arrival on this rule's topic.

        Args:
            now_ns: Arrival time (monotonic).
            validity: The message header's validity flags.
        """
        self._check.on_message(now_ns, validity)

    def tripped(self, now_ns: int) -> bool:
        """Whether the rule's condition has persisted past min_trip_s.

        Args:
            now_ns: Current monotonic time.

        Returns:
            True when tripped.
        """
        if self._check.condition(now_ns):
            if self._condition_since_ns is None:
                self._condition_since_ns = now_ns
            return now_ns - self._condition_since_ns >= self._min_trip_ns
        self._condition_since_ns = None
        return False


def build_monitors(config: fdir_config_pb2.FdirConfig, start_ns: int) -> list[RuleMonitor]:
    """Build a monitor for every rule in a vehicle's fdir block.

    Args:
        config: The declarative rulebook.
        start_ns: Monotonic time monitoring begins.

    Returns:
        One monitor per rule.
    """
    return [RuleMonitor(rule, start_ns) for rule in config.rules]
