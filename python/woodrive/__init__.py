"""Python bindings for the WooDrive BLDC motor controller SDK.

Thin wrapper around the compiled ``_woodrive`` extension (pybind11 bindings
over ``core/WooDriveSdk.h``). See ``python/README.md`` for build instructions
and ``python/examples/basic_check.py`` for a minimal usage example.
"""

from _woodrive import (
    Clock,
    DirectionPhase,
    MotorStatus,
    SerialTransport,
    WooDrive,
)

__all__ = [
    "Clock",
    "DirectionPhase",
    "MotorStatus",
    "SerialTransport",
    "WooDrive",
]
