"""
UAV Flight Control Statechart — Executable SysML Behavioral Model
==================================================================

Implements the FlightComputerBlk statechart as an object-oriented state
machine aligned with IBM Rhapsody / SysML CONOPS phases:

    STANDBY → TAKEOFF → CRUISE → FAILSAFE_RTB → LANDING → STANDBY

Contingency telemetry (signal_loss, low_battery, geo_fence_breach) preempts
nominal flight and forces an immediate transition to FAILSAFE_RTB.

Event traces are persisted to ``output/flight_telemetry_trace.json`` with
microsecond-resolution timestamps for requirements verification evidence.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set


# ---------------------------------------------------------------------------
# Domain enumerations
# ---------------------------------------------------------------------------

class UAVState(str, Enum):
    """SysML state nodes owned by FlightComputerBlk."""

    STANDBY = "STANDBY"
    TAKEOFF = "TAKEOFF"
    CRUISE = "CRUISE"
    FAILSAFE_RTB = "FAILSAFE_RTB"
    LANDING = "LANDING"


class TelemetrySignal(str, Enum):
    """Simulated telemetry / operator events that drive transitions."""

    ARM_AND_LAUNCH = "arm_and_launch"
    ALTITUDE_REACHED = "altitude_reached"
    MISSION_COMPLETE = "mission_complete"
    SIGNAL_LOSS = "signal_loss"
    LOW_BATTERY = "low_battery"
    GEO_FENCE_BREACH = "geo_fence_breach"
    HOME_ACQUIRED = "home_acquired"
    TOUCHDOWN = "touchdown"


# Requirement IDs satisfied by specific transition triggers (RVM cross-ref).
TRIGGER_REQUIREMENT_MAP: Dict[TelemetrySignal, str] = {
    TelemetrySignal.ARM_AND_LAUNCH: "REQ-SYS-001",
    TelemetrySignal.ALTITUDE_REACHED: "REQ-SYS-002",
    TelemetrySignal.SIGNAL_LOSS: "REQ-SYS-003",
    TelemetrySignal.LOW_BATTERY: "REQ-SYS-005",
    TelemetrySignal.GEO_FENCE_BREACH: "REQ-SYS-006",
    TelemetrySignal.HOME_ACQUIRED: "REQ-SYS-007",
    TelemetrySignal.TOUCHDOWN: "REQ-SYS-007",
    TelemetrySignal.MISSION_COMPLETE: "REQ-SYS-007",
}

# Contingency signals that force FAILSAFE_RTB from airborne states.
FAILSAFE_TRIGGERS: Set[TelemetrySignal] = {
    TelemetrySignal.SIGNAL_LOSS,
    TelemetrySignal.LOW_BATTERY,
    TelemetrySignal.GEO_FENCE_BREACH,
}

AIRBORNE_STATES: Set[UAVState] = {UAVState.TAKEOFF, UAVState.CRUISE}


# ---------------------------------------------------------------------------
# Trace record
# ---------------------------------------------------------------------------

@dataclass
class TransitionRecord:
    """Single statechart transition with microsecond timing evidence."""

    timestamp_us: int
    event: str
    from_state: str
    to_state: str
    latency_us: int
    requirement_id: Optional[str] = None
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

@dataclass
class UAVStateMachine:
    """
    Object-oriented UAV flight-control statechart.

    Transition table mirrors a Rhapsody statechart owned by FlightComputerBlk.
    Telemetry signals are injected via :meth:`process_signal`; each accepted
    transition is appended to ``trace`` and optionally flushed to disk.
    """

    initial_state: UAVState = UAVState.STANDBY
    output_path: Path = field(
        default_factory=lambda: Path("output") / "flight_telemetry_trace.json"
    )
    auto_persist: bool = True

    # Populated in __post_init__
    state: UAVState = field(init=False)
    trace: List[TransitionRecord] = field(default_factory=list, init=False)
    _transitions: Dict[UAVState, Dict[TelemetrySignal, UAVState]] = field(
        default_factory=dict, init=False, repr=False
    )
    _listeners: List[Callable[[TransitionRecord], None]] = field(
        default_factory=list, init=False, repr=False
    )

    def __post_init__(self) -> None:
        self.state = self.initial_state
        self.output_path = Path(self.output_path)
        self._build_transition_table()
        self._record_bootstrap()

    # -- table construction -------------------------------------------------

    def _build_transition_table(self) -> None:
        """Define allowed (source, trigger) → target mappings."""
        table: Dict[UAVState, Dict[TelemetrySignal, UAVState]] = {
            UAVState.STANDBY: {
                TelemetrySignal.ARM_AND_LAUNCH: UAVState.TAKEOFF,
            },
            UAVState.TAKEOFF: {
                TelemetrySignal.ALTITUDE_REACHED: UAVState.CRUISE,
                TelemetrySignal.SIGNAL_LOSS: UAVState.FAILSAFE_RTB,
                TelemetrySignal.LOW_BATTERY: UAVState.FAILSAFE_RTB,
                TelemetrySignal.GEO_FENCE_BREACH: UAVState.FAILSAFE_RTB,
            },
            UAVState.CRUISE: {
                TelemetrySignal.MISSION_COMPLETE: UAVState.LANDING,
                TelemetrySignal.SIGNAL_LOSS: UAVState.FAILSAFE_RTB,
                TelemetrySignal.LOW_BATTERY: UAVState.FAILSAFE_RTB,
                TelemetrySignal.GEO_FENCE_BREACH: UAVState.FAILSAFE_RTB,
            },
            UAVState.FAILSAFE_RTB: {
                TelemetrySignal.HOME_ACQUIRED: UAVState.LANDING,
            },
            UAVState.LANDING: {
                TelemetrySignal.TOUCHDOWN: UAVState.STANDBY,
            },
        }
        self._transitions = table

    def _record_bootstrap(self) -> None:
        """Log initial state occupancy (no transition)."""
        record = TransitionRecord(
            timestamp_us=_now_us(),
            event="power_on",
            from_state=self.state.value,
            to_state=self.state.value,
            latency_us=0,
            notes="Initial state occupancy after BIT / power-on",
        )
        self.trace.append(record)

    # -- public API ---------------------------------------------------------

    def add_listener(self, callback: Callable[[TransitionRecord], None]) -> None:
        """Register a callback invoked after every accepted transition."""
        self._listeners.append(callback)

    def process_signal(self, signal: TelemetrySignal | str) -> TransitionRecord:
        """
        Inject a telemetry / operator signal and attempt a state transition.

        Parameters
        ----------
        signal:
            A :class:`TelemetrySignal` member or its string value.

        Returns
        -------
        TransitionRecord
            The resulting transition (or a rejected-event record if illegal).

        Raises
        ------
        ValueError
            If ``signal`` is not a recognized telemetry event.
        """
        if isinstance(signal, str):
            try:
                signal = TelemetrySignal(signal)
            except ValueError as exc:
                raise ValueError(f"Unknown telemetry signal: {signal!r}") from exc

        t0 = time.perf_counter_ns()
        source = self.state
        target = self._resolve_target(source, signal)

        if target is None:
            latency_us = (time.perf_counter_ns() - t0) // 1_000
            record = TransitionRecord(
                timestamp_us=_now_us(),
                event=signal.value,
                from_state=source.value,
                to_state=source.value,
                latency_us=latency_us,
                requirement_id=TRIGGER_REQUIREMENT_MAP.get(signal),
                notes="REJECTED: transition not legal in current state",
            )
            self.trace.append(record)
            self._notify(record)
            if self.auto_persist:
                self.persist_trace()
            return record

        self.state = target
        latency_us = (time.perf_counter_ns() - t0) // 1_000
        record = TransitionRecord(
            timestamp_us=_now_us(),
            event=signal.value,
            from_state=source.value,
            to_state=target.value,
            latency_us=latency_us,
            requirement_id=TRIGGER_REQUIREMENT_MAP.get(signal),
        )
        self.trace.append(record)
        self._notify(record)
        if self.auto_persist:
            self.persist_trace()
        return record

    def _resolve_target(
        self, source: UAVState, signal: TelemetrySignal
    ) -> Optional[UAVState]:
        """Look up the destination state for (source, signal), if any."""
        return self._transitions.get(source, {}).get(signal)

    def is_failsafe_active(self) -> bool:
        """Return True when the vehicle is executing Return-to-Base."""
        return self.state == UAVState.FAILSAFE_RTB

    def reset(self) -> None:
        """Return to STANDBY and clear the in-memory trace."""
        self.state = UAVState.STANDBY
        self.trace.clear()
        self._record_bootstrap()
        if self.auto_persist:
            self.persist_trace()

    def persist_trace(self) -> Path:
        """Write the full event trace to ``output_path`` as JSON."""
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "model": "UAV Flight Control CONOPS Statechart",
            "standard": "SysML / IBM Rhapsody behavioral simulation",
            "final_state": self.state.value,
            "transition_count": len(self.trace),
            "events": [r.to_dict() for r in self.trace],
        }
        self.output_path.write_text(
            json.dumps(payload, indent=2) + "\n", encoding="utf-8"
        )
        return self.output_path

    def _notify(self, record: TransitionRecord) -> None:
        for listener in self._listeners:
            listener(record)

    # -- convenience wrappers (Rhapsody-style event handlers) ---------------

    def on_arm_and_launch(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.ARM_AND_LAUNCH)

    def on_altitude_reached(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.ALTITUDE_REACHED)

    def on_mission_complete(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.MISSION_COMPLETE)

    def on_signal_loss(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.SIGNAL_LOSS)

    def on_low_battery(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.LOW_BATTERY)

    def on_geo_fence_breach(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.GEO_FENCE_BREACH)

    def on_home_acquired(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.HOME_ACQUIRED)

    def on_touchdown(self) -> TransitionRecord:
        return self.process_signal(TelemetrySignal.TOUCHDOWN)


# ---------------------------------------------------------------------------
# Scenario runners
# ---------------------------------------------------------------------------

SCENARIOS: Dict[str, List[TelemetrySignal]] = {
    "nominal": [
        TelemetrySignal.ARM_AND_LAUNCH,
        TelemetrySignal.ALTITUDE_REACHED,
        TelemetrySignal.MISSION_COMPLETE,
        TelemetrySignal.TOUCHDOWN,
    ],
    "signal_loss": [
        TelemetrySignal.ARM_AND_LAUNCH,
        TelemetrySignal.ALTITUDE_REACHED,
        TelemetrySignal.SIGNAL_LOSS,
        TelemetrySignal.HOME_ACQUIRED,
        TelemetrySignal.TOUCHDOWN,
    ],
    "low_battery": [
        TelemetrySignal.ARM_AND_LAUNCH,
        TelemetrySignal.ALTITUDE_REACHED,
        TelemetrySignal.LOW_BATTERY,
        TelemetrySignal.HOME_ACQUIRED,
        TelemetrySignal.TOUCHDOWN,
    ],
    "geo_fence": [
        TelemetrySignal.ARM_AND_LAUNCH,
        TelemetrySignal.GEO_FENCE_BREACH,
        TelemetrySignal.HOME_ACQUIRED,
        TelemetrySignal.TOUCHDOWN,
    ],
}


def run_scenario(
    name: str,
    output_path: Optional[Path] = None,
    verbose: bool = True,
) -> UAVStateMachine:
    """
    Execute a named mission scenario and persist the telemetry trace.

    Parameters
    ----------
    name:
        One of ``nominal``, ``signal_loss``, ``low_battery``, ``geo_fence``.
    output_path:
        Optional override for the JSON trace destination.
    verbose:
        When True, print each transition to stdout.
    """
    if name not in SCENARIOS:
        raise ValueError(
            f"Unknown scenario {name!r}. Choose from: {sorted(SCENARIOS)}"
        )

    kwargs: Dict[str, Any] = {}
    if output_path is not None:
        kwargs["output_path"] = Path(output_path)

    machine = UAVStateMachine(**kwargs)

    if verbose:
        print(f"[SIM] Scenario={name}  initial_state={machine.state.value}")

    for signal in SCENARIOS[name]:
        record = machine.process_signal(signal)
        if verbose:
            status = "OK" if "REJECTED" not in record.notes else "REJECTED"
            print(
                f"  [{status}] {record.from_state:14s} --{record.event}--> "
                f"{record.to_state:14s}  "
                f"latency={record.latency_us} µs  "
                f"req={record.requirement_id or '—'}"
            )

    path = machine.persist_trace()
    if verbose:
        print(f"[SIM] Final state={machine.state.value}")
        print(f"[SIM] Trace written → {path}")

    return machine


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _now_us() -> int:
    """Wall-clock timestamp in microseconds since the Unix epoch."""
    return time.time_ns() // 1_000


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "UAV Flight Control CONOPS statechart simulator "
            "(SysML / IBM Rhapsody behavioral model)."
        )
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS.keys()),
        default="nominal",
        help="Mission profile to execute (default: nominal).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output") / "flight_telemetry_trace.json",
        help="Path for the JSON event trace.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-transition console logging.",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        run_scenario(
            name=args.scenario,
            output_path=args.output,
            verbose=not args.quiet,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
