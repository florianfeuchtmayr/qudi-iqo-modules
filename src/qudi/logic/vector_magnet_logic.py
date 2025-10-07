# -*- coding: utf-8 -*-
"""
A module for controlling vector magnet hardware.

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
import math
import os
import time
from typing import Dict, Optional, List
from PySide2 import QtCore
from qudi.core.module import Base
from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector


class VectorMagnetLogic(Base):
    """Logic layer bridging GUI and hardware; enforces limits and higher-level behavior.
    Vector Magnet Logic Layer (clean version)

    Responsibilities:
     - Field ↔ current conversion (diagonal calibration)
     - Validation of requested setpoints vs vector & per-axis limits
     - Native sweep orchestration (fast option pass-through)
     - Automatic heater management:
           * Turn ON heaters for axes that ramp
           * If persistent mode enabled: turn OFF heaters after ramp completion (simulate lock-in)
           * Otherwise leave heaters ON
     - Ramp progress tracking with tolerance and SWEEP? status assist
     - Zero-all convenience operation
     - Quench handling: lock out further ramps until reset
     - Logging to file and GUI

    Signals:
      sigFieldReadback(dict)    -> Bx,By,Bz,Bmag (in configured units, default mT)
      sigCurrentReadback(dict)  -> Ix,Iy,Iz (A)
      sigSetpointAccepted(dict)
      sigSetpointRejected(str)
      sigRampProgress(float 0..1)
      sigModeUpdate(dict)       -> persistent state updates
      sigQuenchState(dict)      -> {'quench': bool, 'axes': list[str]}
      sigLogEvent(str)
      sigHeaterState(dict)      -> {'x': bool, 'y': bool, 'z': bool}
      sigStatusText

    Example Config:

    vector_magnet_logic:
        module.Class: 'vector_magnet_logic.VectorMagnetLogic'
        connect:
            hardware: vector_magnet
        options:
            field_units: 'mT'
            reject_new_setpoint_while_ramping: true
            allow_option_b_hold_leads: true
            log_to_file: true
            log_file_basename: 'vector_magnet_log.txt'
            ramp_progress_update_ms: 300

    """

    hardware = Connector(name='hardware', interface='VectorMagnetHardwareInterface')

    # Configuration
    _field_units: str = ConfigOption('field_units', default='mT', missing='nothing')
    _reject_mid_ramp: bool = ConfigOption('reject_new_setpoint_while_ramping', default=True, missing='nothing')
    _log_to_file: bool = ConfigOption('log_to_file', default=True, missing='nothing')
    _log_file_basename: str = ConfigOption('log_file_basename', default='vector_magnet_log.txt', missing='nothing')
    _ramp_progress_update_ms: int = ConfigOption('ramp_progress_update_ms', default=300, missing='nothing')

    # Signals
    sigFieldReadback = QtCore.Signal(dict)
    sigCurrentReadback = QtCore.Signal(dict)
    sigSetpointAccepted = QtCore.Signal(dict)
    sigSetpointRejected = QtCore.Signal(str)
    sigRampProgress = QtCore.Signal(float)
    sigModeUpdate = QtCore.Signal(dict)
    sigQuenchState = QtCore.Signal(dict)
    sigLogEvent = QtCore.Signal(str)
    sigHeaterState = QtCore.Signal(dict)
    sigStatusText = QtCore.Signal(str)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Calibration & limits
        self._M_diag: Dict[str, float] = {}
        self._Minv_diag: Dict[str, float] = {}
        self._max_currents: Dict[str, float] = {}
        self._vector_limit_T: float = 0.5
        self._component_limit_T: float = 0.5
        self._current_tolerance_A: float = 0.01

        # Persistent mode
        self._persistent_enabled: bool = False
        self._persistent_idle_behavior: str = 'zero_leads'  # or 'hold_leads'

        # State
        self._heater_states = {'x': False, 'y': False, 'z': False}
        self._quench_active: bool = False
        self._ramping_axes: set[str] = set()
        self._target_currents: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._ramp_start_currents: Dict[str, float] = {}
        self._last_sweep_states: Dict[str, str] = {'x': '', 'y': '', 'z': ''}

        # Timers / logging
        self._ramp_progress_timer: Optional[QtCore.QTimer] = None
        self._log_file_handle = None

    # ---------------- Lifecycle ----------------

    def on_activate(self):
        hw = self.hardware()
        hw.sigCommunicationError.connect(lambda msg: self._log("HW_COMM " + msg))

        # Calibration & limits
        self._M_diag = dict(hw._cal_diag)
        self._Minv_diag = {ax: 1.0 / v for ax, v in self._M_diag.items()}
        self._max_currents = dict(hw._max_currents)
        self._vector_limit_T = hw._vector_field_limit_T
        self._component_limit_T = hw._max_field_T
        self._current_tolerance_A = hw._current_tolerance_A
        self._persistent_enabled = hw._default_persistent
        self._persistent_idle_behavior = hw._persistent_idle_behavior

        # Logging
        if self._log_to_file:
            log_dir = hw._log_directory or os.getcwd()
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, self._log_file_basename)
            self._log_file_handle = open(path, 'a', buffering=1)

        # Hardware status signals
        hw.sigAxisStatus.connect(self._on_axis_status)
        hw.sigQuench.connect(self._on_quench_detected)

        # Ramp progress timer
        self._ramp_progress_timer = QtCore.QTimer(self)
        self._ramp_progress_timer.setInterval(self._ramp_progress_update_ms)
        self._ramp_progress_timer.timeout.connect(self._update_ramp_progress)

        # Initial mode broadcast
        self.sigModeUpdate.emit({
            'persistent_enabled': self._persistent_enabled,
            'persistent_idle_behavior': self._persistent_idle_behavior
        })

    def on_deactivate(self):
        if self._log_file_handle:
            try:
                self._log_file_handle.close()
            except Exception:
                pass
            self._log_file_handle = None

    # ---------------- Logging ----------------

    def _log(self, msg: str):
        line = f"{time.strftime('%Y-%m-%dT%H:%M:%S')}  {msg}"
        if self._log_file_handle:
            self._log_file_handle.write(line + '\n')
        self.sigLogEvent.emit(line)

    # ---------------- Public API (GUI) ----------------

    def set_persistent_mode(self, enable: bool):
        if self._quench_active:
            self.sigSetpointRejected.emit('Quench active – cannot toggle persistent mode.')
            return
        self._persistent_enabled = bool(enable)
        self._log(f'PERSISTENT_MODE_SET {self._persistent_enabled}')
        self.sigModeUpdate.emit({'persistent_enabled': self._persistent_enabled})
        self.sigStatusText.emit(f"Persistent mode {'enabled' if enable else 'disabled'}.")

    def set_persistent_idle_behavior(self, behavior: str):
        if behavior not in ('zero_leads', 'hold_leads'):
            return
        self._persistent_idle_behavior = behavior
        self._log(f'PERSISTENT_IDLE_BEHAVIOR {behavior}')
        self.sigModeUpdate.emit({'persistent_idle_behavior': behavior})
        self.sigStatusText.emit(f"Idle behavior set: {behavior.replace('_', ' ')}")

    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float):
        self.hardware().set_axis_ramp_rate(axis, rate_A_per_s)
        self._log(f'RAMP_RATE_SET axis={axis} rate={rate_A_per_s}')

    def zero_all(self, fast: bool = False):
        """Sweep all axes to zero and treat as a ramp for progress tracking."""
        self._log('ZERO_ALL_REQUEST')
        self.hardware().sweep_zero(fast=fast)
        for ax in self._target_currents:
            self._target_currents[ax] = 0.0
        self._ramping_axes = {'x', 'y', 'z'}
        self._ramp_start_currents.clear()
        if self._ramp_progress_timer and not self._ramp_progress_timer.isActive():
            self._ramp_progress_timer.start()
        # Ensure heaters ON (some controllers require heater on while sweeping)
        for ax in ('x', 'y', 'z'):
            self.hardware().set_axis_heater(ax, True)
        self.sigStatusText.emit("Zero-all sweep started.")

    def request_set_field_cartesian(self, Bx_mT: float, By_mT: float, Bz_mT: float, fast: bool = False):
        self._set_field_internal(Bx_mT / 1000.0, By_mT / 1000.0, Bz_mT / 1000.0, fast=fast)

    def request_set_field_spherical(self, Bmag_mT: float, theta_deg_user: float, phi_deg: float, fast: bool = False):
        B = Bmag_mT / 1000.0
        th_user = math.radians(theta_deg_user)
        phi = math.radians(phi_deg)
        theta_conv = math.pi - th_user
        Bx = B * math.sin(theta_conv) * math.cos(phi)
        By = B * math.sin(theta_conv) * math.sin(phi)
        Bz = B * math.cos(theta_conv)
        self._set_field_internal(Bx, By, Bz, fast=fast)

    def emergency_stop(self):
        """Immediately sweep zero (non-fast) and stop progress tracking."""
        self._log('EMERGENCY_STOP_REQUEST')
        self.hardware().sweep_zero(fast=False)
        self.sigStatusText.emit("Emergency stop: sweeping to zero.")

    def reset_quench(self):
        if not self._quench_active:
            return
        self.hardware().reset_quench()
        self._quench_active = False
        self._log('QUENCH_RESET')
        self.sigQuenchState.emit({'quench': False, 'axes': []})
        self.sigStatusText.emit("Quench reset.")

    # ---------------- Internal Field Setting & Ramping ----------------

    def _set_field_internal(self, Bx_T: float, By_T: float, Bz_T: float, fast: bool):
        if self._quench_active:
            self.sigSetpointRejected.emit('Quench active – reset first.')
            return
        if self._reject_mid_ramp and self._ramping_axes:
            self.sigSetpointRejected.emit('Ramp in progress – request ignored.')
            return

        # Magnitude constraint
        Bmag = math.sqrt(Bx_T ** 2 + By_T ** 2 + Bz_T ** 2)
        if Bmag > self._vector_limit_T + 1e-12:
            self.sigSetpointRejected.emit(f'|B| exceeds limit {self._vector_limit_T} T')
            return
        # Per-component constraint
        for comp, val in zip(('Bx', 'By', 'Bz'), (Bx_T, By_T, Bz_T)):
            if abs(val) > self._component_limit_T + 1e-12:
                self.sigSetpointRejected.emit(f'{comp} exceeds per-axis limit {self._component_limit_T} T')
                return

        # Convert to currents (diagonal calibration)
        targets_A = {
            'x': Bx_T * self._Minv_diag['x'],
            'y': By_T * self._Minv_diag['y'],
            'z': Bz_T * self._Minv_diag['z'],
        }
        for ax, val in targets_A.items():
            if abs(val) > self._max_currents[ax] + 1e-12:
                self.sigSetpointRejected.emit(f'Axis {ax} current {val:.3f}A exceeds limit {self._max_currents[ax]}A')
                return

        # Store
        self._target_currents.update(targets_A)
        self._log(
            f'SETPOINT_ACCEPTED B=({Bx_T:.5f},{By_T:.5f},{Bz_T:.5f})T '
            f'I=({targets_A["x"]:.4f},{targets_A["y"]:.4f},{targets_A["z"]:.4f})A'
        )
        self.sigSetpointAccepted.emit({
            'Bx_T': Bx_T, 'By_T': By_T, 'Bz_T': Bz_T,
            'Ix_A': targets_A['x'], 'Iy_A': targets_A['y'], 'Iz_A': targets_A['z']
        })
        self.sigStatusText.emit("Ramp started.")
        self._begin_ramps(fast=fast)

    def _begin_ramps(self, fast: bool):
        """Issue native sweeps for axes that require movement and start progress tracking."""
        hw = self.hardware()
        self._ramping_axes.clear()
        self._ramp_start_currents.clear()

        # Determine which axes need ramping
        needing = []
        for ax in ('x', 'y', 'z'):
            curr = hw.get_axis_current(ax, fresh=False)
            if math.isnan(curr):
                continue
            tgt = self._target_currents[ax]
            if abs(tgt - curr) > self._current_tolerance_A:
                needing.append(ax)

        # Turn heaters ON for axes that will ramp
        for ax in needing:
            hw.set_axis_heater(ax, True)

        # Start native sweeps
        if hw._use_native_sweep:
            for ax in needing:
                curr = hw.get_axis_current(ax, fresh=False)
                if math.isnan(curr):
                    continue
                self._ramping_axes.add(ax)
                self._ramp_start_currents[ax] = curr
                hw.start_axis_sweep(ax, self._target_currents[ax], fast=fast)

            if not self._ramping_axes:
                # If data incomplete, retry shortly, else finalize
                if any(math.isnan(hw.get_axis_current(a, fresh=False)) for a in ('x', 'y', 'z')):
                    QtCore.QTimer.singleShot(250, lambda: self._begin_ramps(fast=fast))
                    return
                self._post_ramp_finalize()
                return

            if self._ramp_progress_timer and not self._ramp_progress_timer.isActive():
                self._ramp_progress_timer.start()

    # ---------------- Ramp Progress ----------------

    def _update_ramp_progress(self):
        """Compute ramp progress as max fraction among ramping axes."""
        hw = self.hardware()
        if not self._ramping_axes:
            self.sigRampProgress.emit(1.0)
            if self._ramp_progress_timer:
                self._ramp_progress_timer.stop()
            return

        max_fraction = 0.0
        completed: List[str] = []

        for ax in list(self._ramping_axes):
            curr = hw.get_axis_magnet_current(ax, fresh=True)
            if math.isnan(curr):
                curr = hw.get_axis_current(ax, fresh=True)
            tgt = self._target_currents[ax]
            start = self._ramp_start_currents.get(ax, tgt if tgt != 0 else 1.0)

            # Normalize span: use start if target near zero to avoid noise plateau
            if abs(tgt) < 5 * self._current_tolerance_A:
                span = max(abs(start), 5 * self._current_tolerance_A)
            else:
                span = max(abs(tgt), self._current_tolerance_A)

            fraction = 1.0 - min(1.0, abs(tgt - curr) / span)

            # If hardware already reports standby, snap to completion if close
            sweep_state = self._last_sweep_states.get(ax, '')
            if sweep_state.lower().startswith('standby') and fraction > 0.95:
                fraction = 1.0

            max_fraction = max(max_fraction, fraction)

            if abs(tgt - curr) <= self._current_tolerance_A or fraction >= 0.999:
                completed.append(ax)

        for ax in completed:
            self._ramping_axes.discard(ax)

        self.sigRampProgress.emit(max_fraction if self._ramping_axes else 1.0)

        if not self._ramping_axes:
            if self._ramp_progress_timer:
                self._ramp_progress_timer.stop()
            self._log('RAMP_COMPLETE')
            self.sigStatusText.emit("Ramp complete.")
            self._post_ramp_finalize()

    # ---------------- Post-Ramp Handling ----------------

    def _post_ramp_finalize(self):
        """Handle heater state after ramp depending on persistent mode."""
        hw = self.hardware()
        if self._persistent_enabled:
            # Turn heaters OFF (simulate locking flux)
            for ax in ('x', 'y', 'z'):
                hw.set_axis_heater(ax, False)
            self._log('PERSISTENT_FINALIZATION (heaters off)')
            self.sigStatusText.emit("Persistent lock: heaters off.")
        else:
            self._log('NO_PERSISTENT_FINALIZATION (heaters left on)')
            self.sigStatusText.emit("At setpoint (heaters on).")

    # ---------------- Hardware Status Event Handlers ----------------

    @QtCore.Slot(dict)
    def _on_axis_status(self, status: Dict[str, dict]):
        """Handle polled axis status: update field and current readbacks, heater states, sweep states."""
        hw = self.hardware()

        # Convert magnet currents to fields via diagonal calibration
        Bx = status['x']['IMAG'] * hw._cal_diag['x']
        By = status['y']['IMAG'] * hw._cal_diag['y']
        Bz = status['z']['IMAG'] * hw._cal_diag['z']
        Bmag = math.sqrt(Bx ** 2 + By ** 2 + Bz ** 2)

        scale = 1000.0 if self._field_units.lower() == 'mt' else 1.0
        self.sigFieldReadback.emit({
            'Bx_mT': Bx * scale,
            'By_mT': By * scale,
            'Bz_mT': Bz * scale,
            'Bmag_mT': Bmag * scale
        })

        self.sigCurrentReadback.emit({
            'Ix': status['x']['IOUT'],
            'Iy': status['y']['IOUT'],
            'Iz': status['z']['IOUT']
        })

        # Update sweep & heater states
        for ax in ('x', 'y', 'z'):
            self._last_sweep_states[ax] = status[ax].get('SWEEP', '')
            self._heater_states[ax] = status[ax]['heater']

        self.sigHeaterState.emit(dict(self._heater_states))

    @QtCore.Slot(dict)
    def _on_quench_detected(self, axes: Dict[str, bool]):
        if not self._quench_active:
            self._quench_active = True
            self._log(f'QUENCH_DETECTED axes={list(axes.keys())}')
            # Stop ramp tracking
            if self._ramp_progress_timer and self._ramp_progress_timer.isActive():
                self._ramp_progress_timer.stop()
            self._ramping_axes.clear()
            self.sigQuenchState.emit({'quench': True, 'axes': list(axes.keys())})
            self.sigStatusText.emit("QUENCH detected – reset required.")