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
    _field_use_imag_when_heater_off: bool = ConfigOption(
        'field_use_imag_when_heater_off', default=False, missing='nothing'
    )
    _field_zero_epsilon_A: float = ConfigOption(
        'field_zero_epsilon_A', default=1e-4, missing='nothing'
    )

    # Signals
    sigFieldReadback = QtCore.Signal(dict)
    sigCurrentReadback = QtCore.Signal(dict)
    sigSetpointAccepted = QtCore.Signal(dict)
    sigSetpointRejected = QtCore.Signal(str)
    sigModeUpdate = QtCore.Signal(dict)
    sigQuenchState = QtCore.Signal(dict)
    sigLogEvent = QtCore.Signal(str)
    sigHeaterState = QtCore.Signal(dict)
    sigStatusText = QtCore.Signal(str)
    sigRampActiveState = QtCore.Signal(bool)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Calibration & limits
        self._M_diag: Dict[str, float] = {}
        self._Minv_diag: Dict[str, float] = {}
        self._max_currents: Dict[str, float] = {}
        self._vector_limit_T: float = 0.5
        self._component_limit_T: float = 0.5
        self._current_tolerance_A: float = 0.01
        self._heater_warmup_s: float = 5.0  # default if not supplied by hardware

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
        self._paused_axes: set[str] = set()

        # Timers / logging
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
        try:
            self._heater_warmup_s = float(hw._heater_warmup_s)
        except Exception:
            self._heater_warmup_s = 5.0
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

        # Initial mode broadcast
        self.sigModeUpdate.emit({
            'persistent_enabled': self._persistent_enabled,
            'persistent_idle_behavior': self._persistent_idle_behavior
        })
        self.sigRampActiveState.emit(False)

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

    def set_axis_heater(self, axis: str, on: bool):
        """Manually set a single heater ON/OFF from the GUI."""
        if axis not in ('x', 'y', 'z'):
            return
        self.hardware().set_axis_heater(axis, bool(on))
        self._log(f'HEATER_SET axis={axis} on={bool(on)}')
        self.sigStatusText.emit(f"Heater {axis.upper()} {'ON' if on else 'OFF'} request sent.")

    def toggle_axis_heater(self, axis: str):
        """Toggle heater state for one axis."""
        current = self._heater_states.get(axis, False)
        self.set_axis_heater(axis, not current)

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

    # ... inside class VectorMagnetLogic ...

    def zero_all(self, fast: bool = False):
        """Sweep all axes to zero with proper heater gating."""
        self._log('ZERO_ALL_REQUEST')

        # Turn heaters ON for all axes and warm up, then issue zero sweep
        for ax in ('x', 'y', 'z'):
            self.hardware().set_axis_heater(ax, True)

        def _do_zero():
            self.hardware().sweep_zero(fast=fast)
            self._target_currents.update({'x': 0.0, 'y': 0.0, 'z': 0.0})
            self._ramping_axes = {'x', 'y', 'z'}
            self._paused_axes.clear()
            self.sigStatusText.emit("Zero-all sweep started.")
            self.sigRampActiveState.emit(True)

        QtCore.QTimer.singleShot(max(0, int(round(self._heater_warmup_s * 1000.0))), _do_zero)

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
        # Treat as aborted ramp
        self._ramping_axes.clear()
        self._paused_axes.clear()
        self._post_ramp_finalize(aborted=True)
        self.sigStatusText.emit("Emergency stop: sweeping to zero.")

    def reset_quench(self):
        if not self._quench_active:
            return
        self.hardware().reset_quench()
        self._quench_active = False
        self._log('QUENCH_RESET')
        self.sigQuenchState.emit({'quench': False, 'axes': []})
        self.sigStatusText.emit("Quench reset.")

    def stop_ramp(self):
        """Pause ongoing sweeps (SWEEP PAUSE) and allow new setpoints without emergency zero."""
        if not self._ramping_axes:
            return
        hw = self.hardware()
        for ax in list(self._ramping_axes):
            self._pause_axis(ax, hw)
        self._paused_axes.update(self._ramping_axes)
        self._ramping_axes.clear()
        self._log('RAMP_PAUSED')
        self.sigStatusText.emit("Ramp paused.")
        # Heaters remain ON; user may change setpoint now.
        self.sigRampActiveState.emit(False)

    def _pause_axis(self, axis: str, hw):
        # Send SWEEP PAUSE for axis
        try:
            if axis in ('x', 'y'):
                hw._write(hw._dual_port, f'CHAN {hw._axis_chan[axis]}')
                time.sleep(0.05)
                hw._collect_bytes(hw._dual_port, 0.15, 0.04)
                hw._write(hw._dual_port, 'SWEEP PAUSE')
            else:
                hw._write(hw._single_port, 'SWEEP PAUSE')
        except Exception:
            pass

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

    # ... inside class VectorMagnetLogic ...

    def _begin_ramps(self, fast: bool):
        hw = self.hardware()
        self._ramping_axes.clear()
        self._ramp_start_currents.clear()
        self._paused_axes.clear()

        needing: list[str] = []
        for ax in ('x', 'y', 'z'):
            curr = hw.get_axis_magnet_current(ax, fresh=True)
            if math.isnan(curr):
                curr = hw.get_axis_current(ax, fresh=True)
            if math.isnan(curr):
                continue
            tgt = self._target_currents[ax]
            if abs(tgt - curr) > self._current_tolerance_A:
                needing.append(ax)

        if not needing:
            self._post_ramp_finalize()
            return

        if self._persistent_enabled:
            # Persistent: handle each axis separately depending on heater state
            for ax in needing:
                if not self._heater_states.get(ax, False):
                    # Heater OFF: we must first match IOUT to IMAG
                    self._log(f'PERSISTENT_PREP axis={ax}')
                    self._ramp_axis_persistent_sequence(ax, fast)
                else:
                    # Heater already ON: just do a warm-up delay then final sweep
                    hw.set_axis_heater(ax, True)
                    QtCore.QTimer.singleShot(
                        int(round(self._heater_warmup_s * 1000.0)),
                        lambda a=ax: self._start_axis_final_sweep(a, fast)
                    )
        else:
            # Non-persistent: turn all heaters ON and ramp after warm-up collectively
            for ax in needing:
                hw.set_axis_heater(ax, True)
            QtCore.QTimer.singleShot(
                int(round(self._heater_warmup_s * 1000.0)),
                lambda: [self._start_axis_final_sweep(a, fast) for a in needing]
            )

    def _ramp_axis_persistent_sequence(self, axis: str, fast: bool):
        """
        Persistent mode sequence:
          1. Ramp supply output (IOUT) to stored magnet current IMAG with heater OFF.
          2. When |IOUT - IMAG| <= tolerance, turn heater ON.
          3. After warm-up, perform final sweep to target current.
        """
        hw = self.hardware()
        imag = hw.get_axis_magnet_current(axis, fresh=True)
        if math.isnan(imag):
            imag = hw.get_axis_current(axis, fresh=True)
        if math.isnan(imag):
            QtCore.QTimer.singleShot(250, lambda: self._ramp_axis_persistent_sequence(axis, fast))
            return

        # Step 1: ramp supply to IMAG (using existing start_axis_sweep abstraction)
        hw.start_axis_sweep(axis, imag, fast=False)
        self._log(f'PERSISTENT_MATCH axis={axis} imag={imag:.5f}A')

        def _check_match():
            iout = hw.get_axis_current(axis, fresh=True)
            im2 = hw.get_axis_magnet_current(axis, fresh=True)
            if math.isnan(iout) or math.isnan(im2):
                QtCore.QTimer.singleShot(300, _check_match)
                return
            if abs(iout - im2) <= self._current_tolerance_A:
                # Step 2: heater ON
                hw.set_axis_heater(axis, True)
                self._log(f'PERSISTENT_HEATER_ON axis={axis}')
                # Warm-up delay then final sweep
                QtCore.QTimer.singleShot(
                    int(round(self._heater_warmup_s * 1000.0)),
                    lambda: self._start_axis_final_sweep(axis, fast)
                )
            else:
                QtCore.QTimer.singleShot(300, _check_match)

        _check_match()

    def _start_axis_final_sweep(self, axis: str, fast: bool):
        """Issue final sweep to target current after heater warm-up."""
        hw = self.hardware()
        tgt = self._target_currents[axis]
        curr = hw.get_axis_magnet_current(axis, fresh=True)
        if math.isnan(curr):
            curr = hw.get_axis_current(axis, fresh=True)
        if math.isnan(curr) or abs(tgt - curr) <= self._current_tolerance_A:
            return
        hw.start_axis_sweep(axis, tgt, fast=fast)
        self._ramping_axes.add(axis)
        self._ramp_start_currents[axis] = curr
        self._log(f'FINAL_SWEEP axis={axis} target={tgt:.5f}A')
        self.sigRampActiveState.emit(True)

    # ---------------- Post-Ramp Handling ----------------

    def _post_ramp_finalize(self, aborted: bool = False):
        """Handle heater state after ramp depending on persistent mode."""
        hw = self.hardware()
        if aborted:
            self.sigStatusText.emit("Ramp aborted.")
        else:
            if self._persistent_enabled:
                for ax in ('x', 'y', 'z'):
                    hw.set_axis_heater(ax, False)
                self._log('PERSISTENT_FINALIZATION (heaters off)')
                self.sigStatusText.emit("Persistent lock: heaters off.")
            else:
                self._log('NO_PERSISTENT_FINALIZATION (heaters left on)')
                self.sigStatusText.emit("At setpoint (heaters on).")
        self.sigRampActiveState.emit(False)

    # ---------------- Hardware Status Event Handlers ----------------

    def _current_for_field(self, st: dict) -> float:
        """
        Choose the current used to compute the field for this axis.

        Default (recommended when IMAG is stale on idle axes):
        - Prefer IOUT (supply output) always.
        - Fall back to IMAG only if IOUT is NaN/unavailable.
        - Clamp very small magnitudes to 0 using _field_zero_epsilon_A.

        Optional (enable via field_use_imag_when_heater_off=True):
        - If heater is OFF (persistent switch closed), prefer IMAG; else prefer IOUT.
        """
        iout = st.get('IOUT')
        imag = st.get('IMAG')
        heater_on = bool(st.get('heater', False))
        eps = float(self._field_zero_epsilon_A)

        def isnum(x):
            try:
                return (x is not None) and (not math.isnan(x))
            except Exception:
                return False

        if self._field_use_imag_when_heater_off and (not heater_on):
            val = imag if isnum(imag) else (iout if isnum(iout) else 0.0)
        else:
            val = iout if isnum(iout) else (imag if isnum(imag) else 0.0)
        if abs(val) < eps:
            return 0.0
        return val

    @QtCore.Slot(dict)
    def _on_axis_status(self, status: Dict[str, dict]):
        """Update field and current readbacks, heater states, sweep states."""
        hw = self.hardware()

        # Effective coil currents for field computation (policy above)
        ix_eff = self._current_for_field(status['x'])
        iy_eff = self._current_for_field(status['y'])
        iz_eff = self._current_for_field(status['z'])

        # Field from diagonal calibration
        Bx = ix_eff * hw._cal_diag['x']
        By = iy_eff * hw._cal_diag['y']
        Bz = iz_eff * hw._cal_diag['z']
        Bmag = math.sqrt(Bx * Bx + By * By + Bz * Bz)

        scale = 1000.0 if self._field_units.lower() == 'mt' else 1.0
        self.sigFieldReadback.emit({
            'Bx_mT': Bx * scale,
            'By_mT': By * scale,
            'Bz_mT': Bz * scale,
            'Bmag_mT': Bmag * scale
        })

        # Keep showing supply outputs for current readback (unchanged)
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

        # Completion evaluation (without progress timer):
        if self._ramping_axes:
            finished: List[str] = []
            tol = float(self._current_tolerance_A)
            for ax in list(self._ramping_axes):
                # Use freshest magnet current if available, otherwise supply current
                curr = hw.get_axis_magnet_current(ax, fresh=True)
                if math.isnan(curr):
                    curr = hw.get_axis_current(ax, fresh=True)

                tgt = self._target_currents.get(ax, curr)
                if (not math.isnan(curr)) and (abs(tgt - curr) <= tol):
                    # Axis reached target: pause native sweep to stop motion, mark finished
                    self._pause_axis(ax, hw)
                    finished.append(ax)

            for ax in finished:
                self._ramping_axes.discard(ax)

            if not self._ramping_axes:
                # All axes finished: finalize heaters depending on persistent mode
                self._log('RAMP_COMPLETE')
                self._post_ramp_finalize(aborted=False)

    @QtCore.Slot(dict)
    def _on_quench_detected(self, axes: Dict[str, bool]):
        if not self._quench_active:
            self._quench_active = True
            self._log(f'QUENCH_DETECTED axes={list(axes.keys())}')
            self._ramping_axes.clear()
            self._paused_axes.clear()
            self.sigQuenchState.emit({'quench': True, 'axes': list(axes.keys())})
            self.sigStatusText.emit("QUENCH detected – reset required.")
            self.sigRampActiveState.emit(False)