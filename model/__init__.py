"""UAV Flight Control CONOPS statechart package."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from model.uav_state_machine import (
        TelemetrySignal,
        UAVState,
        UAVStateMachine,
        run_scenario,
    )

__all__ = [
    "TelemetrySignal",
    "UAVState",
    "UAVStateMachine",
    "run_scenario",
]


def __getattr__(name: str):
    """Lazy-export public symbols to avoid import cycles with ``python -m``."""
    if name in __all__:
        from model import uav_state_machine as _sm

        return getattr(_sm, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
