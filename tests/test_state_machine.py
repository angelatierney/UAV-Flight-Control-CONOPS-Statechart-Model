"""
Verification suite for the UAV Flight Control CONOPS statechart.

Validates SysML / Rhapsody-aligned behavioral requirements, including the
critical fail-safe latency budget (REQ-SYS-004: < 100 ms).
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from model.uav_state_machine import (
    FAILSAFE_TRIGGERS,
    TelemetrySignal,
    UAVState,
    UAVStateMachine,
    run_scenario,
)

# Fail-safe transition latency budget (REQ-SYS-004).
FAILSAFE_LATENCY_BUDGET_S = 0.100
FAILSAFE_LATENCY_BUDGET_MS = 100


@pytest.fixture
def machine(tmp_path: Path) -> UAVStateMachine:
    """Fresh state machine writing traces into a temporary directory."""
    return UAVStateMachine(
        output_path=tmp_path / "flight_telemetry_trace.json",
        auto_persist=False,
    )


@pytest.fixture
def airborne_cruise(machine: UAVStateMachine) -> UAVStateMachine:
    """Advance a machine through STANDBY → TAKEOFF → CRUISE."""
    machine.on_arm_and_launch()
    machine.on_altitude_reached()
    assert machine.state == UAVState.CRUISE
    return machine


# ---------------------------------------------------------------------------
# Nominal CONOPS transitions
# ---------------------------------------------------------------------------

def test_initial_state_is_standby(machine: UAVStateMachine) -> None:
    """REQ-SYS-001 pre-condition: power-on occupies STANDBY."""
    assert machine.state == UAVState.STANDBY


def test_arm_and_launch_transitions_to_takeoff(machine: UAVStateMachine) -> None:
    """REQ-SYS-001: STANDBY --arm_and_launch--> TAKEOFF."""
    record = machine.on_arm_and_launch()
    assert machine.state == UAVState.TAKEOFF
    assert record.from_state == UAVState.STANDBY.value
    assert record.to_state == UAVState.TAKEOFF.value
    assert record.requirement_id == "REQ-SYS-001"
    assert "REJECTED" not in record.notes


def test_altitude_reached_enters_cruise(machine: UAVStateMachine) -> None:
    """REQ-SYS-002: TAKEOFF --altitude_reached--> CRUISE."""
    machine.on_arm_and_launch()
    record = machine.on_altitude_reached()
    assert machine.state == UAVState.CRUISE
    assert record.requirement_id == "REQ-SYS-002"


def test_nominal_mission_returns_to_standby(machine: UAVStateMachine) -> None:
    """Full nominal path: STANDBY → … → LANDING → STANDBY."""
    machine.on_arm_and_launch()
    machine.on_altitude_reached()
    machine.on_mission_complete()
    assert machine.state == UAVState.LANDING
    machine.on_touchdown()
    assert machine.state == UAVState.STANDBY


# ---------------------------------------------------------------------------
# Fail-safe transitions (REQ-SYS-003 / 005 / 006 / 004)
# ---------------------------------------------------------------------------

def test_signal_loss_failsafe_under_100ms(
    airborne_cruise: UAVStateMachine,
) -> None:
    """
    REQ-SYS-003 + REQ-SYS-004:
    signal_loss forces CRUISE → FAILSAFE_RTB in under 100 ms.
    """
    t0 = time.perf_counter()
    record = airborne_cruise.on_signal_loss()
    elapsed_s = time.perf_counter() - t0

    assert airborne_cruise.state == UAVState.FAILSAFE_RTB
    assert airborne_cruise.is_failsafe_active()
    assert record.to_state == UAVState.FAILSAFE_RTB.value
    assert record.requirement_id == "REQ-SYS-003"
    assert elapsed_s < FAILSAFE_LATENCY_BUDGET_S, (
        f"Fail-safe latency {elapsed_s * 1000:.3f} ms exceeds "
        f"{FAILSAFE_LATENCY_BUDGET_MS} ms budget (REQ-SYS-004)"
    )
    assert record.latency_us < FAILSAFE_LATENCY_BUDGET_MS * 1000


def test_low_battery_forces_failsafe_rtb(
    airborne_cruise: UAVStateMachine,
) -> None:
    """REQ-SYS-005: low_battery forces CRUISE → FAILSAFE_RTB."""
    t0 = time.perf_counter()
    record = airborne_cruise.on_low_battery()
    elapsed_s = time.perf_counter() - t0

    assert airborne_cruise.state == UAVState.FAILSAFE_RTB
    assert record.requirement_id == "REQ-SYS-005"
    assert elapsed_s < FAILSAFE_LATENCY_BUDGET_S


def test_geo_fence_breach_forces_failsafe_rtb(machine: UAVStateMachine) -> None:
    """REQ-SYS-006: geo_fence_breach from TAKEOFF forces FAILSAFE_RTB."""
    machine.on_arm_and_launch()
    assert machine.state == UAVState.TAKEOFF

    t0 = time.perf_counter()
    record = machine.on_geo_fence_breach()
    elapsed_s = time.perf_counter() - t0

    assert machine.state == UAVState.FAILSAFE_RTB
    assert record.requirement_id == "REQ-SYS-006"
    assert elapsed_s < FAILSAFE_LATENCY_BUDGET_S


@pytest.mark.parametrize("trigger", sorted(FAILSAFE_TRIGGERS, key=lambda s: s.value))
def test_all_failsafe_triggers_from_cruise(
    airborne_cruise: UAVStateMachine,
    trigger: TelemetrySignal,
) -> None:
    """Every contingency signal from CRUISE must enter FAILSAFE_RTB < 100 ms."""
    t0 = time.perf_counter()
    record = airborne_cruise.process_signal(trigger)
    elapsed_s = time.perf_counter() - t0

    assert airborne_cruise.state == UAVState.FAILSAFE_RTB
    assert record.to_state == UAVState.FAILSAFE_RTB.value
    assert elapsed_s < FAILSAFE_LATENCY_BUDGET_S


def test_failsafe_rtb_completes_landing_to_standby(
    airborne_cruise: UAVStateMachine,
) -> None:
    """REQ-SYS-007: FAILSAFE_RTB → LANDING → STANDBY recovery path."""
    airborne_cruise.on_signal_loss()
    airborne_cruise.on_home_acquired()
    assert airborne_cruise.state == UAVState.LANDING
    airborne_cruise.on_touchdown()
    assert airborne_cruise.state == UAVState.STANDBY


# ---------------------------------------------------------------------------
# Guard / rejection behaviour
# ---------------------------------------------------------------------------

def test_signal_loss_rejected_in_standby(machine: UAVStateMachine) -> None:
    """Fail-safe triggers are not legal while grounded in STANDBY."""
    record = machine.on_signal_loss()
    assert machine.state == UAVState.STANDBY
    assert "REJECTED" in record.notes


def test_unknown_signal_raises(machine: UAVStateMachine) -> None:
    """Unrecognized telemetry strings raise ValueError."""
    with pytest.raises(ValueError, match="Unknown telemetry signal"):
        machine.process_signal("warp_drive_failure")


# ---------------------------------------------------------------------------
# Trace / telemetry evidence (REQ-SYS-008)
# ---------------------------------------------------------------------------

def test_trace_contains_microsecond_timestamps(
    airborne_cruise: UAVStateMachine,
) -> None:
    """REQ-SYS-008: every record carries a µs-resolution timestamp."""
    airborne_cruise.on_signal_loss()
    assert len(airborne_cruise.trace) >= 2
    for record in airborne_cruise.trace:
        assert isinstance(record.timestamp_us, int)
        assert record.timestamp_us > 0
        assert isinstance(record.latency_us, int)
        assert record.latency_us >= 0


def test_persist_trace_writes_json(
    machine: UAVStateMachine, tmp_path: Path
) -> None:
    """persist_trace emits a well-formed JSON artifact under output/."""
    machine.auto_persist = True
    machine.output_path = tmp_path / "trace.json"
    machine.on_arm_and_launch()

    assert machine.output_path.is_file()
    payload = json.loads(machine.output_path.read_text(encoding="utf-8"))
    assert payload["final_state"] == UAVState.TAKEOFF.value
    assert payload["transition_count"] == len(machine.trace)
    assert "events" in payload
    assert payload["events"][-1]["event"] == "arm_and_launch"


def test_run_scenario_signal_loss(tmp_path: Path) -> None:
    """End-to-end signal_loss scenario ends in STANDBY with a persisted trace."""
    out = tmp_path / "scenario_trace.json"
    result = run_scenario("signal_loss", output_path=out, verbose=False)
    assert result.state == UAVState.STANDBY
    assert out.is_file()
    events = {e["event"] for e in json.loads(out.read_text())["events"]}
    assert "signal_loss" in events
    assert any(
        e["to_state"] == UAVState.FAILSAFE_RTB.value
        for e in json.loads(out.read_text())["events"]
    )


def test_requirements_matrix_is_loadable() -> None:
    """Sanity-check that the RVM JSON is present and well-formed."""
    matrix_path = (
        Path(__file__).resolve().parents[1] / "model" / "requirements_matrix.json"
    )
    data = json.loads(matrix_path.read_text(encoding="utf-8"))
    assert data["summary"]["total_requirements"] == 8
    assert data["summary"]["compliant"] == 8
    ids = {req["id"] for req in data["requirements"]}
    assert "REQ-SYS-003" in ids
    assert "REQ-SYS-004" in ids
