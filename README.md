# UAV Flight Control CONOPS & Statechart Model

**Model-Based Systems Engineering (MBSE) | IBM Rhapsody / SysML**

Developed an MBSE flight control model for an Unmanned Aerial Vehicle using SysML and Rhapsody concepts. Mapped mission requirements to subsystem block architectures and designed statechart diagrams to simulate autonomous fail-safe transitions against operational constraints.

---

## Overview

This repository captures a lightweight, executable digital twin of a UAV flight-control Concept of Operations (CONOPS). Behavioral logic is expressed as a SysML-style statechart and realized in Python so that fail-safe transitions can be simulated, timed, and traced against allocated system requirements—mirroring the analysis workflow typically performed in IBM Rational Rhapsody.

| Artifact | Purpose |
|---|---|
| `model/uav_state_machine.py` | Executable statechart (STANDBY → TAKEOFF → CRUISE → FAILSAFE_RTB → LANDING) |
| `model/requirements_matrix.json` | Requirements Verification Matrix (RVM) |
| `tests/test_state_machine.py` | Automated verification of fail-safe timing & transition rules |
| `output/flight_telemetry_trace.json` | Microsecond-resolution event trace from simulation runs |

---

## System Architecture Breakdown

The architecture follows SysML **Block Definition Diagram (BDD)** composition. The top-level `UAV_System` block aggregates four primary subsystems, each owning ports, interfaces, and allocated requirements consistent with a Rhapsody IBD (Internal Block Diagram) view.

```
UAV_System
├── FlightComputerBlk      «block»  — mission manager, statechart host, fail-safe arbiter
├── TelemetryBlk           «block»  — uplink/downlink health, link-quality monitoring
├── NavigationBlk          «block»  — GNSS, IMU fusion, geo-fence enforcement
└── PropulsionBlk          «block»  — motor/ESC control, battery state-of-charge
```

### Subsystem Responsibilities

| Block | Key Interfaces | Allocated Concerns |
|---|---|---|
| **FlightComputerBlk** | `cmdIn`, `modeOut`, `failSafeTrigger` | State orchestration, mode arbitration, RTB command generation |
| **TelemetryBlk** | `linkStatus`, `heartbeat`, `signalQuality` | Detects `signal_loss`; reports C2 (command & control) continuity |
| **NavigationBlk** | `position`, `geoFenceStatus`, `homeWaypoint` | Detects `geo_fence_breach`; provides RTB navigation solution |
| **PropulsionBlk** | `throttleCmd`, `batterySoc`, `motorHealth` | Detects `low_battery`; executes thrust profiles per flight phase |

### Interface Context (IBD Concept)

```
  GCS ◄──TelemetryBlk──► FlightComputerBlk ◄──NavigationBlk──► GeoFence
                              │
                              ▼
                        PropulsionBlk
```

---

## Concept of Operations (CONOPS)

Mission phases map one-to-one onto statechart states. Transitions are guarded by telemetry predicates and operational constraints.

| Phase | State | Entry Criteria | Exit / Transition Trigger |
|---|---|---|---|
| **1. Pre-Flight** | `STANDBY` | Power-on BIT complete; GCS handshake OK | Operator `arm_and_launch` command |
| **2. Takeoff** | `TAKEOFF` | Cleared altitude climb profile active | `altitude_reached` (cruise altitude acquired) |
| **3. Autonomous Cruise** | `CRUISE` | Waypoint navigation engaged | `mission_complete` **or** fail-safe trigger |
| **4. Emergency Fail-Safe (RTB)** | `FAILSAFE_RTB` | `signal_loss` \| `low_battery` \| `geo_fence_breach` | `home_acquired` (home waypoint reached) |
| **5. Landing** | `LANDING` | Descent profile armed | `touchdown` → back to `STANDBY` |

### Nominal Mission Flow

```
STANDBY ──arm_and_launch──► TAKEOFF ──altitude_reached──► CRUISE
                                                              │
                                              mission_complete│
                                                              ▼
STANDBY ◄──touchdown── LANDING ◄──home_acquired── FAILSAFE_RTB
                              ▲                         ▲
                              │                         │
                              └──── (from CRUISE / TAKEOFF on contingency)
```

### Fail-Safe Philosophy

Contingency events (`signal_loss`, `low_battery`, `geo_fence_breach`) preempt nominal flight from **TAKEOFF** or **CRUISE** and force an immediate transition to `FAILSAFE_RTB`. The Flight Computer must complete the transition in **&lt; 100 ms** (REQ-SYS-004).

---

## Requirements Verification Matrix

| Req ID | Statement | Trigger / Evidence | Method | Allocation | Status |
|---|---|---|---|---|---|
| REQ-SYS-001 | System shall transition STANDBY → TAKEOFF on arm command | `arm_and_launch` | Test | FlightComputerBlk | Compliant |
| REQ-SYS-002 | System shall enter CRUISE when cruise altitude is reached | `altitude_reached` | Sim | FlightComputerBlk / NavigationBlk | Compliant |
| REQ-SYS-003 | System shall enter FAILSAFE_RTB on command-link loss | `signal_loss` | Test | TelemetryBlk / FlightComputerBlk | Compliant |
| REQ-SYS-004 | Fail-safe transition latency shall be &lt; 100 ms | timing assertion in pytest | Test | FlightComputerBlk | Compliant |
| REQ-SYS-005 | System shall enter FAILSAFE_RTB on low battery | `low_battery` | Test | PropulsionBlk / FlightComputerBlk | Compliant |
| REQ-SYS-006 | System shall enter FAILSAFE_RTB on geo-fence breach | `geo_fence_breach` | Test | NavigationBlk / FlightComputerBlk | Compliant |
| REQ-SYS-007 | System shall land and return to STANDBY after RTB | `home_acquired` → `touchdown` | Sim | FlightComputerBlk | Compliant |
| REQ-SYS-008 | Event traces shall be logged with µs timestamps | `flight_telemetry_trace.json` | Test | FlightComputerBlk | Compliant |

The authoritative machine-readable RVM lives in [`model/requirements_matrix.json`](model/requirements_matrix.json).

---

## Repository Structure

```
UAV-Flight-Control-CONOPS-Statechart-Model/
├── README.md
├── requirements.txt
├── .gitignore
├── model/
│   ├── __init__.py
│   ├── uav_state_machine.py      # Executable SysML statechart
│   └── requirements_matrix.json  # RVM
├── tests/
│   ├── __init__.py
│   └── test_state_machine.py     # Verification suite
└── output/
    └── .gitkeep                  # Trace JSON written here at runtime
```

---

## Quick Start

### 1. Prerequisites

- Python 3.9+
- `pip` package manager

### 2. Install Dependencies

```bash
cd UAV-Flight-Control-CONOPS-Statechart-Model
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Run the State Machine Simulation

Execute a full nominal + contingency mission profile. The simulator writes a microsecond-resolution event trace to `output/flight_telemetry_trace.json`.

```bash
# Nominal cruise then mission-complete landing
python -m model.uav_state_machine --scenario nominal

# Inject signal-loss fail-safe during cruise
python -m model.uav_state_machine --scenario signal_loss

# Inject low-battery fail-safe during cruise
python -m model.uav_state_machine --scenario low_battery

# Inject geo-fence breach during takeoff
python -m model.uav_state_machine --scenario geo_fence

# Custom output path
python -m model.uav_state_machine --scenario signal_loss --output output/custom_trace.json
```

### 4. Inspect the Telemetry Trace

```bash
cat output/flight_telemetry_trace.json | python -m json.tool
```

Example trace entry:

```json
{
  "timestamp_us": 1727370000123456,
  "event": "signal_loss",
  "from_state": "CRUISE",
  "to_state": "FAILSAFE_RTB",
  "latency_us": 42,
  "requirement_id": "REQ-SYS-003"
}
```

### 5. Run Verification Tests

```bash
# Full suite
pytest tests/ -v

# Fail-safe latency only (REQ-SYS-004)
pytest tests/test_state_machine.py::test_signal_loss_failsafe_under_100ms -v

# With coverage
pytest tests/ --cov=model --cov-report=term-missing
```

---

## Mapping to IBM Rhapsody / SysML Artifacts

| This Repo | Rhapsody / SysML Equivalent |
|---|---|
| `UAVStateMachine` class | Statechart diagram owned by `FlightComputerBlk` |
| Enumerated states | SysML `State` nodes |
| Trigger methods (`on_signal_loss`, …) | Transition triggers / signal events |
| Guard predicates | SysML transition guards |
| `flight_telemetry_trace.json` | Sequence / timing diagram evidence |
| `requirements_matrix.json` | Requirements diagram + satisfy/verify links |

---

## License

Distributed for educational and portfolio demonstration purposes.
