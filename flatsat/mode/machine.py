"""The mode state machine — pure logic, no bus, no clock ownership.

Encodes the settled §7 rules:

  * Transition graph: Init → Nominal ⇄ (Safe → Recovery). Return from
    Safe goes THROUGH Recovery (checkout happens payload-off, one
    subsystem at a time); Safe → Nominal directly is not a transition
    that exists.
  * Authority asymmetry: toward SAFE needs no authority — FDIR trips,
    watchdogs, failed init checks, anyone. Away from safety is honored
    only with ground authority. Init → Nominal additionally accepts the
    boot source when the previous shutdown was clean.
  * Anti-flap: a minimum dwell time applies to AWAY-from-safety
    transitions only — safety must never wait out a timer. Entries into
    SAFE are counted for repeat-entry escalation.

Time comes in as arguments (``now_ns``); the caller owns the clock,
which is what makes every rule here unit-testable without sleeping.
"""

from __future__ import annotations

from dataclasses import dataclass

from flatsat.msgs import mode_pb2

BOOT_SOURCE = "boot"

# Explicit transition graph — a pair's absence means "does not exist".
_TOWARD_SAFETY = {
    (mode_pb2.SYSTEM_MODE_INIT, mode_pb2.SYSTEM_MODE_SAFE),
    (mode_pb2.SYSTEM_MODE_NOMINAL, mode_pb2.SYSTEM_MODE_SAFE),
    (mode_pb2.SYSTEM_MODE_RECOVERY, mode_pb2.SYSTEM_MODE_SAFE),
}
_AWAY_FROM_SAFETY = {
    (mode_pb2.SYSTEM_MODE_INIT, mode_pb2.SYSTEM_MODE_NOMINAL),
    (mode_pb2.SYSTEM_MODE_SAFE, mode_pb2.SYSTEM_MODE_RECOVERY),
    (mode_pb2.SYSTEM_MODE_RECOVERY, mode_pb2.SYSTEM_MODE_NOMINAL),
}


@dataclass(frozen=True)
class Decision:
    """Outcome of one mode request.

    Attributes:
        accepted: Whether the transition was taken.
        reason: Why — the request's reason when accepted, the refusal
            ground when not.
    """

    accepted: bool
    reason: str


class ModeStateMachine:
    """Holds the latched mode and decides every requested transition."""

    def __init__(self, min_dwell_ns: int, start_ns: int) -> None:
        """Start latched in INIT.

        Args:
            min_dwell_ns: Minimum time in a mode before an
                away-from-safety transition out of it is honored.
            start_ns: Current monotonic time.
        """
        self._min_dwell_ns = min_dwell_ns
        self.mode: mode_pb2.SystemMode.ValueType = mode_pb2.SYSTEM_MODE_INIT
        self.mode_seq = 0
        self.reason = "boot"
        # Dwell guards the gap BETWEEN transitions; no transition has
        # happened yet, so the first one must never be dwell-blocked.
        self.last_transition_ns = start_ns - min_dwell_ns
        self.transitions = 0
        self.rejected_requests = 0
        self.safe_entries = 0

    def decide(self, request: mode_pb2.ModeRequest, now_ns: int) -> Decision:
        """Apply one mode request against the rules.

        Args:
            request: The requested transition.
            now_ns: Current monotonic time (for dwell enforcement).

        Returns:
            The decision; on acceptance the machine's latched state has
            already advanced.
        """
        wanted = request.requested
        pair = (self.mode, wanted)

        if wanted == self.mode:
            return self._reject(f"already in {mode_pb2.SystemMode.Name(self.mode)}")

        if pair in _TOWARD_SAFETY:
            # Toward safety: no authority check, no dwell — never make
            # safety wait.
            return self._accept(request, now_ns)

        if pair in _AWAY_FROM_SAFETY:
            boot_nominal = (
                pair == (mode_pb2.SYSTEM_MODE_INIT, mode_pb2.SYSTEM_MODE_NOMINAL)
                and request.source == BOOT_SOURCE
            )
            if not request.ground_authority and not boot_nominal:
                return self._reject("away-from-safety transitions are ground-command-only")
            if now_ns - self.last_transition_ns < self._min_dwell_ns:
                return self._reject("minimum dwell time not met (anti-flap)")
            return self._accept(request, now_ns)

        return self._reject(
            f"no transition {mode_pb2.SystemMode.Name(self.mode)} -> "
            f"{mode_pb2.SystemMode.Name(wanted)}"
        )

    def _accept(self, request: mode_pb2.ModeRequest, now_ns: int) -> Decision:
        """Advance the latched state (internal).

        Args:
            request: The accepted request.
            now_ns: Current monotonic time.

        Returns:
            The affirmative decision.
        """
        self.mode = request.requested
        self.mode_seq += 1
        self.reason = f"{request.source}: {request.reason}"
        self.last_transition_ns = now_ns
        self.transitions += 1
        if request.requested == mode_pb2.SYSTEM_MODE_SAFE:
            self.safe_entries += 1
        return Decision(accepted=True, reason=self.reason)

    def _reject(self, why: str) -> Decision:
        """Refuse a request (internal).

        Args:
            why: Refusal ground.

        Returns:
            The negative decision.
        """
        self.rejected_requests += 1
        return Decision(accepted=False, reason=why)

    def state_message(self, transition_time_ns: int) -> mode_pb2.ModeState:
        """Render the latched state for the bus.

        Args:
            transition_time_ns: Flight-clock (wall) time of the last
                transition, stamped by the caller who owns the clock.

        Returns:
            The latched ModeState.
        """
        return mode_pb2.ModeState(
            mode=self.mode,
            mode_seq=self.mode_seq,
            reason=self.reason,
            transition_time_ns=transition_time_ns,
        )
