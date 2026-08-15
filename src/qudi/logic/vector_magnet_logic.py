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
from PySide6 import QtCore
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
    _field_use_imag_when_heater_off: bool = ConfigOption(
        'field_use_imag_when_heater_off', default=True, missing='nothing'
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
        self._heater_cooldown_s: float = 10.0

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
        self._consec_in_tol_counts: Dict[str, int] = {'x': 0, 'y': 0, 'z': 0}

        # Timers / logging
        self._log_file_handle = None

    # ---------------- Lifecycle ----------------

    def on_activate(self):
        hw = self.hardware()
        hw.sigCommunicationError.connect(lambda msg: self._log("HW_COMM " + msg))

        # Calibration & limits (unchanged)
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

        # Read defaults from hardware config and publish to GUI
        self._persistent_enabled = bool(hw._default_persistent)
        self._persistent_idle_behavior = str(hw._persistent_idle_behavior or 'hold_leads')

        # Hardware status signals
        hw.sigAxisStatus.connect(self._on_axis_status)
        hw.sigQuench.connect(self._on_quench_detected)

        # Initial mode broadcast to synchronize GUI
        self.sigModeUpdate.emit({
            'persistent_enabled': self._persistent_enabled,
            'persistent_idle_behavior': self._persistent_idle_behavior
        })
        self.sigRampActiveState.emit(False)

    def on_deactivate(self):
        # Disconnect hardware signals to avoid late calls during shutdown
        try:
            self.hardware().sigAxisStatus.disconnect(self._on_axis_status)
        except Exception:
            pass
        try:
            self.hardware().sigQuench.disconnect(self._on_quench_detected)
        except Exception:
            pass

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
        """Enable/disable persistent mode. If disabling, force idle behavior to hold_leads."""
        if self._quench_active:
            self.sigSetpointRejected.emit('Quench active – cannot toggle persistent mode.')
            return
        self._persistent_enabled = bool(enable)

        # When persistent is disabled, zero_leads is not meaningful; force hold_leads.
        if not self._persistent_enabled and (self._persistent_idle_behavior != 'hold_leads'):
            self._persistent_idle_behavior = 'hold_leads'
            self._log('PERSISTENT_IDLE_FORCED_TO_HOLD_LEADS')

        self._log(f'PERSISTENT_MODE_SET {self._persistent_enabled}')
        self.sigModeUpdate.emit({
            'persistent_enabled': self._persistent_enabled,
            'persistent_idle_behavior': self._persistent_idle_behavior
        })
        self.sigStatusText.emit(f"Persistent mode {'enabled' if enable else 'disabled'}.")

    def set_persistent_idle_behavior(self, behavior: str):
        """Set idle behavior. If selecting zero_leads while not persistent, auto-enable persistent."""
        if behavior not in ('zero_leads', 'hold_leads'):
            return
        # If user requests zero_leads and persistent is OFF, turn persistent ON automatically.
        if behavior == 'zero_leads' and (not self._persistent_enabled):
            self._persistent_enabled = True
            self._log('PERSISTENT_AUTO_ENABLED_FOR_ZERO_LEADS')

        self._persistent_idle_behavior = behavior
        self._log(f'PERSISTENT_IDLE_BEHAVIOR {behavior}')
        self.sigModeUpdate.emit({
            'persistent_enabled': self._persistent_enabled,
            'persistent_idle_behavior': self._persistent_idle_behavior
        })
        self.sigStatusText.emit(f"Idle behavior set: {behavior.replace('_', ' ')}")

    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float):
        self.hardware().set_axis_ramp_rate(axis, rate_A_per_s)
        self._log(f'RAMP_RATE_SET axis={axis} rate={rate_A_per_s}')

    # ... inside class VectorMagnetLogic ...

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
            self.sigRampActiveState.emit(False)
            return

        if self._persistent_enabled:
            # Turn heaters OFF (persistent lock); magnet stays at its stored current.
            for ax in ('x', 'y', 'z'):
                try:
                    hw.set_axis_heater(ax, False)
                except Exception:
                    pass

            if self._persistent_idle_behavior == 'zero_leads':
                # Wait for persistent switch to open after heater OFF, then sweep leads to zero.
                cooldown_ms = int(round(self._heater_cooldown_s * 1000.0))

                def _sweep_leads_zero_after_cooldown():
                    try:
                        hw.start_axis_sweep('x', 0.0, fast=False)
                        hw.start_axis_sweep('y', 0.0, fast=False)
                        hw.start_axis_sweep('z', 0.0, fast=False)
                        self._log(
                            f'PERSISTENT_FINALIZATION (heaters off, waited {self._heater_cooldown_s:.1f}s, leads swept to zero)')
                        self.sigStatusText.emit(
                            f"Persistent lock: heaters off; waited {self._heater_cooldown_s:.0f}s for switch opening; leads swept to zero."
                        )
                    except Exception:
                        self._log('PERSISTENT_FINALIZATION error sweeping leads to zero')
                        self.sigStatusText.emit("Persistent lock: heaters off; sweep-to-zero failed.")

                self._log(f'PERSISTENT_COOLDOWN_WAIT {self._heater_cooldown_s:.1f}s before lead zero')
                self.sigStatusText.emit(
                    f"Persistent lock: heaters off; waiting {self._heater_cooldown_s:.0f}s for switch opening..."
                )
                QtCore.QTimer.singleShot(cooldown_ms, _sweep_leads_zero_after_cooldown)
            else:
                # hold_leads: do nothing to the supply outputs
                self._log('PERSISTENT_FINALIZATION (heaters off; leads held)')
                self.sigStatusText.emit("Persistent lock: heaters off; leads held.")

        else:
            # Non-persistent: leave heaters ON at setpoint
            self._log('NO_PERSISTENT_FINALIZATION (heaters left on)')
            self.sigStatusText.emit("At setpoint (heaters on).")

        self.sigRampActiveState.emit(False)

    # ---------------- Hardware Status Event Handlers ----------------

    def _current_for_field(self, st: dict) -> float:
        """
        Choose the current used to compute the field for this axis.

        Default (recommended when IMAG is stale on idle axes):
        - Prefer IMAG (supply output) always.
        - Fall back to IOUT only if IMAG is NaN/unavailable.
        - Clamp very small magnitudes to 0 using _field_zero_epsilon_A.

        Optional (enable via field_use_imag_when_heater_off=True):
        - If heater is OFF (persistent switch closed), prefer IMAG; else prefer IOUT.
        """
        iout = st.get('IOUT')
        imag = st.get('IMAG')
        eps = float(self._field_zero_epsilon_A)

        def isnum(x):
            try:
                return (x is not None) and (not math.isnan(x))
            except Exception:
                return False

        # Prefer IMAG regardless of heater state; fall back to IOUT
        val = imag if isnum(imag) else (iout if isnum(iout) else 0.0)
        if abs(val) < eps:
            return 0.0
        return val

    @QtCore.Slot(dict)
    def _on_axis_status(self, status: Dict[str, dict]):
        """Update readbacks and perform ramp completion tracking without extra hardware queries."""
        try:
            # Effective currents for field computation
            ix_eff = self._current_for_field(status['x'])
            iy_eff = self._current_for_field(status['y'])
            iz_eff = self._current_for_field(status['z'])

            # Field from diagonal calibration (cached)
            Bx = ix_eff * self._M_diag.get('x', 0.0)
            By = iy_eff * self._M_diag.get('y', 0.0)
            Bz = iz_eff * self._M_diag.get('z', 0.0)
            Bmag = math.sqrt(Bx * Bx + By * By + Bz * Bz)

            scale = 1000.0 if self._field_units.lower() == 'mt' else 1.0
            self.sigFieldReadback.emit({
                'Bx_mT': Bx * scale,
                'By_mT': By * scale,
                'Bz_mT': Bz * scale,
                'Bmag_mT': Bmag * scale
            })

            # Emit both supply and magnet currents, plus effective
            self.sigCurrentReadback.emit({
                'Ix_out': status['x'].get('IOUT', float('nan')),
                'Iy_out': status['y'].get('IOUT', float('nan')),
                'Iz_out': status['z'].get('IOUT', float('nan')),
                'Ix_mag': status['x'].get('IMAG', float('nan')),
                'Iy_mag': status['y'].get('IMAG', float('nan')),
                'Iz_mag': status['z'].get('IMAG', float('nan')),
                'Ix_eff': ix_eff,
                'Iy_eff': iy_eff,
                'Iz_eff': iz_eff,
            })

            # Update sweep & heater states
            for ax in ('x', 'y', 'z'):
                self._last_sweep_states[ax] = status[ax].get('SWEEP', '')
                self._heater_states[ax] = status[ax].get('heater', False)
            self.sigHeaterState.emit(dict(self._heater_states))

            # Completion evaluation using already computed effective currents
            if self._ramping_axes:
                finished: List[str] = []
                tol = float(self._current_tolerance_A)
                eff_map = {'x': ix_eff, 'y': iy_eff, 'z': iz_eff}
                for ax in list(self._ramping_axes):
                    curr_eff = eff_map[ax]
                    tgt = self._target_currents.get(ax, curr_eff)
                    if (not math.isnan(curr_eff)) and (abs(tgt - curr_eff) <= tol):
                        # Track consecutive in-tolerance polls
                        self._consec_in_tol_counts[ax] = self._consec_in_tol_counts.get(ax, 0) + 1
                        if self._consec_in_tol_counts[ax] >= 3:
                            # Pause native sweep to stop motion, mark finished
                            try:
                                hw = self.hardware()
                                self._pause_axis(ax, hw)
                            except Exception:
                                # If hardware already disconnected, skip pausing gracefully
                                pass
                            finished.append(ax)
                    else:
                        # Reset consecutive count if out of tolerance
                        self._consec_in_tol_counts[ax] = 0

                for ax in finished:
                    self._ramping_axes.discard(ax)
                    self._consec_in_tol_counts[ax] = 0

                if not self._ramping_axes:
                    self._log('RAMP_COMPLETE')
                    self._post_ramp_finalize(aborted=False)

        except Exception as exc:
            # Be defensive in shutdown: do not propagate fatal errors
            self._log(f'LOGIC_STATUS_ERROR {exc}')

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