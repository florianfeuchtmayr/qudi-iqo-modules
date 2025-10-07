# -*- coding: utf-8 -*-
"""
Interface file for a vector magnet from.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

from __future__ import annotations
from abc import abstractmethod
from typing import Dict
from PySide2 import QtCore
from qudi.core.module import Base


class VectorMagnetHardwareInterface(Base):
    """Abstract base class for a 3‑axis vector magnet hardware layer.
    Vector Magnet Hardware Interface

    Defines the abstract base class that the concrete hardware driver must implement.
    This interface is intentionally handles:
      - Starting native sweeps toward a target current per axis
      - Zeroing all axes
      - Setting ramp rates
      - Heater (persistent switch) control
      - Quench reset
      - Current readout (supply output vs magnet current)

    Signal contract (Qt signals emitted by hardware implementation):
      sigAxisStatus(dict):
          {
            'x': {'IOUT': float|nan, 'IMAG': float|nan, 'SWEEP': str, 'heater': bool, 'timestamp': float},
            'y': {...},
            'z': {...}
          }
      sigQuench(dict):
          e.g. {'x': True, 'z': True} if quench detected on those axes
      sigCommunicationError(str):
          emitted on recoverable communication issues

    NOTE:
     - All currents in Ampere.
     - Fields are not part of the hardware interface (conversion handled in the logic layer).

    """

    # Signals (redeclared for interface introspection)
    sigAxisStatus = QtCore.Signal(dict)
    sigQuench = QtCore.Signal(dict)
    sigCommunicationError = QtCore.Signal(str)

    # ---------------- Abstract API ----------------

    @abstractmethod
    def start_axis_sweep(self, axis: str, target_A: float, fast: bool = False) -> None:
        """Non-blocking command initiating a native sweep (UP/DOWN) toward target_A on an axis."""

    @abstractmethod
    def sweep_zero(self, fast: bool = False) -> None:
        """Sweep all axes back to zero current natively."""

    @abstractmethod
    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float) -> None:
        """Store (and if applicable, forward) desired ramp rate for an axis."""

    @abstractmethod
    def set_heater(self, group: str, on: bool) -> None:
        """Turn heater group ON/OFF (group can be 'xy' or 'z')."""

    @abstractmethod
    def query_heater(self, group: str) -> bool:
        """Return current heater group state (may use cached poll values)."""

    @abstractmethod
    def reset_quench(self) -> None:
        """Send a non-blocking quench reset command."""

    @abstractmethod
    def get_axis_current(self, axis: str, fresh: bool = False) -> float:
        """Return supply output current (IOUT) for axis. If fresh=True, force device query."""

    @abstractmethod
    def get_axis_magnet_current(self, axis: str, fresh: bool = True) -> float:
        """Return magnet current (IMAG) for axis. If fresh=False may return cached value."""

    # ---------------- Optional Introspection ----------------

    def axis_limits_A(self) -> Dict[str, float]:
        """Optional convenience: per-axis current limits."""
        return {}

    def ramp_rates_A_per_s(self) -> Dict[str, float]:
        """Optional convenience: current stored ramp rates."""
        return {}