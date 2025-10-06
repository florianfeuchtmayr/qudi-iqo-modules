# -*- coding: utf-8 -*-
"""
Vector Magnet Interfaces.

Defines abstract interfaces for:
 - Hardware layer controlling Cryomagnetics 4G supplies (dual X/Y + single Z).
 - (Optional) Logic layer for higher-level field / persistent mode control.

Add this file to: src/qudi/interface/

Usage in hardware:
    from qudi.interface.vector_magnet_interface import VectorMagnetHardwareInterface
    class VectorMagnetHardware(VectorMagnetHardwareInterface):
        ...

Usage in logic:
    hardware = Connector(name='hardware', interface='VectorMagnetHardwareInterface')

If you decide to use the logic interface, expose a connector elsewhere with:
    Connector(interface='VectorMagnetLogicInterface')
"""

from __future__ import annotations
from abc import abstractmethod
from typing import Dict, Optional
from PySide2 import QtCore
from qudi.core.module import Base


class VectorMagnetHardwareInterface(Base):
    """
    Abstract interface for 3‑axis vector magnet hardware based on Cryomagnetics 4G supplies.

    Axes:
        x, y on a dual supply (selected via CHAN)
        z on a single supply

    Heater groups:
        'xy' -> dual supply heater
        'z'  -> single supply heater

    Implementations must provide native sweep control (ULIM/LLIM + SWEEP) or emulate via software
    and emit the status signal at least at the configured poll interval.

    Signals expected (names must match for logic to connect):
        sigAxisStatus(dict):
            {
              'x': {'IOUT': float, 'IMAG': float, 'SWEEP': str, 'heater': bool, 'timestamp': float},
              'y': {...},
              'z': {...}
            }
        sigQuench(dict): e.g. {'x': True, 'z': True} when those axes report a quench
        sigCommunicationError(str): communication or parsing errors

    All currents in Ampere, fields (if ever exposed here) in Tesla.
    """

    # Re-declare signals as empty signals so the interface consumer can connect regardless
    sigAxisStatus = QtCore.Signal(dict)
    sigQuench = QtCore.Signal(dict)
    sigCommunicationError = QtCore.Signal(str)

    # ---------------- Abstract control methods ----------------
    @abstractmethod
    def start_axis_sweep(self, axis: str, target_A: float, fast: bool = False) -> None:
        """Begin a native sweep (UP/DOWN) toward target current for given axis.
           Must be non-blocking. 'fast' only if already permissible (no auto heater toggling)."""

    @abstractmethod
    def sweep_zero(self, fast: bool = False) -> None:
        """Issue zero sweep to all axes (or per-axis if implementation chooses)."""

    @abstractmethod
    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float) -> None:
        """Update stored ramp / sweep rate for an axis (implementation may store for later use)."""

    @abstractmethod
    def set_heater(self, group: str, on: bool) -> None:
        """Set heater ON/OFF for group ('xy' or 'z')."""

    @abstractmethod
    def query_heater(self, group: str) -> bool:
        """Return heater group state quickly (may use cached polling result)."""

    @abstractmethod
    def reset_quench(self) -> None:
        """Send QRESET (or equivalent) to clear quench state; non-blocking."""

    @abstractmethod
    def get_axis_current(self, axis: str, fresh: bool = False) -> float:
        """Return current supply output current for axis. 'fresh' forces device query if possible."""

    @abstractmethod
    def get_axis_magnet_current(self, axis: str, fresh: bool = True) -> float:
        """Return magnet current (IMAG) for axis (could differ if persistent mode)."""

    # Optional (implementation may just no-op if using native sweeps only)
    def enable_software_ramp_mode(self, enable: bool) -> None:
        """Optional method: request switching to software ramp fallback."""
        return

    # ---------------- Introspection helpers (not strictly required) ----------------
    def axis_limits_A(self) -> Dict[str, float]:
        """Return per-axis current limits if hardware stores them (optional convenience)."""
        return {}

    def ramp_rates_A_per_s(self) -> Dict[str, float]:
        """Return current per-axis ramp rates (optional convenience)."""
        return {}
