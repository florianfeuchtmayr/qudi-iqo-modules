# -*- coding: utf-8 -*-
"""
Logic layer orchestrating:
 - Field→current conversion
 - Request validation (limits, vector magnitude)
 - Native sweep coordination with the hardware
 - Persistent mode state machine (heater warm/cool)
 - Ramp progress monitoring
 - Quench lockout & reset
 - Emergency stop
 - Logging

Angle convention:
  User θ: 0° = -Z, 180° = +Z (we map internally: θ_conv = π - θ_user_rad)
  φ: azimuth 0–360°, from +X toward +Y.

Units exposed to GUI: mT (internal SI: Tesla).
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
    """
    Connectors:
      hardware -> VectorMagnetHardware

    Signals:
      sigFieldReadback(dict)
      sigCurrentReadback(dict)
      sigSetpointAccepted(dict)
      sigSetpointRejected(str)
      sigRampProgress(float 0..1)
      sigModeUpdate(dict)
      sigQuenchState(dict)
      sigLogEvent(str)
      sigHeaterState(dict)  # {'xy': bool, 'z': bool}
    """

    hardware = Connector(name='hardware', interface='VectorMagnetHardwareInterface')

    # Config options
    _field_units: str = ConfigOption('field_units', default='mT', missing='nothing')
    _reject_mid_ramp: bool = ConfigOption('reject_new_setpoint_while_ramping', default=True, missing='nothing')
    _allow_hold_leads: bool = ConfigOption('allow_option_b_hold_leads', default=True, missing='nothing')
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

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._M_diag: Dict[str, float] = {}
        self._Minv_diag: Dict[str, float] = {}
        self._max_currents: Dict[str, float] = {}
        self._vector_limit_T: float = 0.5
        self._component_limit_T: float = 0.5
        self._current_tolerance_A: float = 0.01

        self._persistent_enabled: bool = False
        self._persistent_idle_behavior: str = 'zero_leads'  # 'hold_leads'
        self._heater_timers: Dict[str, QtCore.QTimer] = {}
        self._heater_states = {'xy': False, 'z': False}

        self._quench_active: bool = False
        self._ramping_axes: set[str] = set()
        self._target_currents: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._ramp_progress_timer: Optional[QtCore.QTimer] = None

        self._log_file_handle = None
        self._software_ramp_mode = False  # fallback activation
        self._software_ramp_step_A = 0.05  # per tick per axis (if fallback)
        self._software_ramp_timer: Optional[QtCore.QTimer] = None

    # -------------- Lifecycle --------------
    def on_activate(self):
        hw = self.hardware()
        # Pull calibration & constraints
        self._M_diag = dict(hw._cal_diag)
        self._Minv_diag = {ax: 1.0 / v for ax, v in self._M_diag.items()}
        self._max_currents = dict(hw._max_currents)
        self._vector_limit_T = hw._vector_field_limit_T
        self._component_limit_T = hw._max_field_T
        self._current_tolerance_A = hw._current_tolerance_A
        self._persistent_enabled = hw._default_persistent
        self._persistent_idle_behavior = hw._persistent_idle_behavior

        if self._log_to_file:
            log_dir = hw._log_directory or os.getcwd()
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, self._log_file_basename)
            self._log_file_handle = open(path, 'a', buffering=1)

        # Connect signals from hardware
        hw.sigAxisStatus.connect(self._on_axis_status)
        hw.sigQuench.connect(self._on_quench_detected)

        # Setup ramp progress timer
        self._ramp_progress_timer = QtCore.QTimer(self)
        self._ramp_progress_timer.setInterval(self._ramp_progress_update_ms)
        self._ramp_progress_timer.timeout.connect(self._update_ramp_progress)

        # Software ramp timer
        self._software_ramp_timer = QtCore.QTimer(self)
        self._software_ramp_timer.setInterval(200)
        self._software_ramp_timer.timeout.connect(self._software_ramp_step)

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

    # -------------- Logging --------------
    def _log(self, msg: str):
        line = f"{time.strftime('%Y-%m-%dT%H:%M:%S')}  {msg}"
        if self._log_file_handle:
            self._log_file_handle.write(line + '\n')
        self.sigLogEvent.emit(line)

    # -------------- Public API (GUI) --------------
    def set_persistent_mode(self, enable: bool):
        if self._quench_active:
            self.sigSetpointRejected.emit('Quench active – cannot toggle persistent mode.')
            return
        self._persistent_enabled = bool(enable)
        self._log(f'PERSISTENT_MODE_SET {self._persistent_enabled}')
        self.sigModeUpdate.emit({'persistent_enabled': self._persistent_enabled})

    def set_persistent_idle_behavior(self, behavior: str):
        if behavior not in ('zero_leads', 'hold_leads'):
            return
        self._persistent_idle_behavior = behavior
        self._log(f'PERSISTENT_IDLE_BEHAVIOR {behavior}')
        self.sigModeUpdate.emit({'persistent_idle_behavior': behavior})

    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float):
        # The hardware stores them; just forward
        self.hardware().set_axis_ramp_rate(axis, rate_A_per_s)
        self._log(f'RAMP_RATE_SET axis={axis} rate={rate_A_per_s}')

    def request_set_field_cartesian(self, Bx_mT: float, By_mT: float, Bz_mT: float, fast: bool = False):
        self._set_field_internal(Bx_mT/1000.0, By_mT/1000.0, Bz_mT/1000.0, fast=fast)

    def request_set_field_spherical(self, Bmag_mT: float, theta_deg_user: float, phi_deg: float, fast: bool = False):
        B = Bmag_mT / 1000.0
        th_user = math.radians(theta_deg_user)
        phi = math.radians(phi_deg)
        # Convert: θ_conv = π - θ_user
        theta_conv = math.pi - th_user
        Bx = B * math.sin(theta_conv) * math.cos(phi)
        By = B * math.sin(theta_conv) * math.sin(phi)
        Bz = B * math.cos(theta_conv)
        self._set_field_internal(Bx, By, Bz, fast=fast)

    def emergency_stop(self):
        self._log('EMERGENCY_STOP_REQUEST')
        # Cancel software ramp if active
        if self._software_ramp_timer and self._software_ramp_timer.isActive():
            self._software_ramp_timer.stop()
        self.hardware().sweep_zero(fast=False)

    def reset_quench(self):
        if not self._quench_active:
            return
        self.hardware().reset_quench()
        self._quench_active = False
        self._log('QUENCH_RESET')
        self.sigQuenchState.emit({'quench': False, 'axes': []})

    # -------------- Core Field Setting Logic --------------
    def _set_field_internal(self, Bx_T: float, By_T: float, Bz_T: float, fast: bool):
        if self._quench_active:
            self.sigSetpointRejected.emit('Quench active – reset first.')
            return
        if self._reject_mid_ramp and self._ramping_axes:
            self.sigSetpointRejected.emit('Ramp in progress – request ignored.')
            return

        # Validate magnitude
        Bmag = math.sqrt(Bx_T**2 + By_T**2 + Bz_T**2)
        if Bmag >= self._vector_limit_T - 1e-12:
            self.sigSetpointRejected.emit(f'|B| exceeds limit {self._vector_limit_T} T')
            return
        # Per component
        for comp, val in zip(('Bx', 'By', 'Bz'), (Bx_T, By_T, Bz_T)):
            if abs(val) > self._component_limit_T + 1e-12:
                self.sigSetpointRejected.emit(f'{comp} exceeds per-axis limit {self._component_limit_T} T')
                return

        # Field->current diag
        targets_A = {
            'x': Bx_T * self._Minv_diag['x'],
            'y': By_T * self._Minv_diag['y'],
            'z': Bz_T * self._Minv_diag['z'],
        }
        for ax, val in targets_A.items():
            if abs(val) > self._max_currents[ax] + 1e-12:
                self.sigSetpointRejected.emit(f'Axis {ax} current {val:.3f}A exceeds limit {self._max_currents[ax]}A')
                return

        self._target_currents.update(targets_A)
        self._log(f'SETPOINT_ACCEPTED B=({Bx_T:.5f},{By_T:.5f},{Bz_T:.5f})T I=({targets_A["x"]:.4f},{targets_A["y"]:.4f},{targets_A["z"]:.4f})A')
        self.sigSetpointAccepted.emit({
            'Bx_T': Bx_T, 'By_T': By_T, 'Bz_T': Bz_T,
            'Ix_A': targets_A['x'], 'Iy_A': targets_A['y'], 'Iz_A': targets_A['z']
        })

        self._begin_ramps(fast=fast)

    def _begin_ramps(self, fast: bool):
        hw = self.hardware()
        self._ramping_axes.clear()
        # Provide ramp commands (native) or fallback
        if hw._use_native_sweep:
            for ax in ('x','y','z'):
                current_now = hw.get_axis_current(ax, fresh=True)
                tgt = self._target_currents[ax]
                if abs(tgt - current_now) > self._current_tolerance_A:
                    self._ramping_axes.add(ax)
                    hw.start_axis_sweep(ax, tgt, fast=fast)
            if self._ramping_axes:
                self._ramp_progress_timer.start()
            else:
                # Already effectively at setpoint
                self._post_ramp_finalize()
        else:
            # Software fallback mode
            self._software_ramp_mode = True
            self._ramping_axes = {ax for ax in ('x','y','z')
                                  if abs(self._target_currents[ax] - hw.get_axis_current(ax, fresh=True)) > self._current_tolerance_A}
            if self._ramping_axes:
                self._software_ramp_timer.start()
                self._ramp_progress_timer.start()
            else:
                self._post_ramp_finalize()

    # -------------- Ramp Progress & Completion --------------
    def _update_ramp_progress(self):
        hw = self.hardware()
        if not self._ramping_axes:
            self.sigRampProgress.emit(1.0)
            self._ramp_progress_timer.stop()
            return

        max_fraction = 0.0
        completed: List[str] = []
        for ax in list(self._ramping_axes):
            curr = hw.get_axis_current(ax, fresh=True)
            tgt = self._target_currents[ax]
            span = max(abs(tgt), 1e-9)
            fraction = 1.0 - min(1.0, abs(tgt - curr) / span)
            max_fraction = max(max_fraction, fraction)
            if abs(tgt - curr) <= self._current_tolerance_A:
                completed.append(ax)
        for ax in completed:
            self._ramping_axes.discard(ax)

        self.sigRampProgress.emit(max_fraction)
        if not self._ramping_axes:
            self._ramp_progress_timer.stop()
            self._log('RAMP_COMPLETE')
            # End of ramp, handle persistent
            self._post_ramp_finalize()

    def _software_ramp_step(self):
        if not self._software_ramp_mode:
            return
        hw = self.hardware()
        still_axes = []
        for ax in list(self._ramping_axes):
            current = hw.get_axis_current(ax, fresh=True)
            target = self._target_currents[ax]
            delta = target - current
            if abs(delta) <= self._current_tolerance_A:
                continue
            step = self._software_ramp_step_A
            if abs(delta) < step:
                step = abs(delta)
            new_value = current + math.copysign(step, delta)
            # Issue next partial sweep via IMAG path is not ideal; we mimic native by adjusting ULIM/LLIM relative
            # For safety just set direct final limit approach (coarse).
            hw.start_axis_sweep(ax, new_value, fast=False)
            still_axes.append(ax)
        self._ramping_axes = set(still_axes)
        if not self._ramping_axes:
            self._software_ramp_timer.stop()
            self._software_ramp_mode = False
            self._log('SOFTWARE_RAMP_COMPLETE')
            self._post_ramp_finalize()

    def _post_ramp_finalize(self):
        # Handle persistent mode transitions
        if self._persistent_enabled:
            self._transition_to_persistent()
        else:
            self._log('NO_PERSISTENT_FINALIZATION')

    # -------------- Persistent Mode Handling --------------
    def _transition_to_persistent(self):
        # Steps:
        # 1. Ensure heaters ON during ramp if needed (already ON externally)
        # 2. Turn heaters OFF to lock (XY, then Z if non-zero target)
        # 3. Wait cooldown (heater_cooldown_s)
        # 4. If 'zero_leads' behavior: sweep zero leads
        hw = self.hardware()
        any_xy = any(abs(self._target_currents[a]) > 1e-9 for a in ('x','y'))
        any_z = abs(self._target_currents['z']) > 1e-9

        # We don't implement asynchronous warm/cool in deep detail; simple timers:
        def finalize():
            if self._persistent_idle_behavior == 'zero_leads':
                hw.sweep_zero(fast=False)
                self._log('PERSISTENT_LOCKED leads_zeroed')
            else:
                self._log('PERSISTENT_LOCKED leads_held')
            self.sigModeUpdate.emit({
                'persistent_locked': True,
                'persistent_idle_behavior': self._persistent_idle_behavior
            })

        # Turn off heaters if current non-zero
        if any_xy:
            hw.set_heater('xy', True)  # Ensure ON
            hw.set_heater('xy', False)
            self._heater_states['xy'] = False
        if any_z:
            hw.set_heater('z', True)
            hw.set_heater('z', False)
            self._heater_states['z'] = False

        self.sigHeaterState.emit(dict(self._heater_states))

        # Cooldown timer
        cooldown = QtCore.QTimer(self)
        cooldown.setSingleShot(True)
        cooldown.timeout.connect(finalize)
        cooldown.start(int(self.hardware()._heater_cooldown_s * 1000))

    # -------------- Event Handlers from Hardware --------------
    @QtCore.Slot(dict)
    def _on_axis_status(self, status: Dict[str, dict]):
        # Generate readbacks for GUI
        # Magnet current * diag factor ⇒ field component
        Bx = status['x']['IMAG'] * self._M_diag['x']
        By = status['y']['IMAG'] * self._M_diag['y']
        Bz = status['z']['IMAG'] * self._M_diag['z']
        Bmag = math.sqrt(Bx**2 + By**2 + Bz**2)
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
        # Heater states combined (store last)
        # For dual supply we treat the single PSHTR for XY:
        self._heater_states['xy'] = status['x']['heater'] or status['y']['heater']
        self._heater_states['z'] = status['z']['heater']
        self.sigHeaterState.emit(dict(self._heater_states))

    @QtCore.Slot(dict)
    def _on_quench_detected(self, axes: Dict[str, bool]):
        if not self._quench_active:
            self._quench_active = True
            self._log(f'QUENCH_DETECTED axes={list(axes.keys())}')
            # Stop any ongoing ramp timers
            if self._ramp_progress_timer and self._ramp_progress_timer.isActive():
                self._ramp_progress_timer.stop()
            if self._software_ramp_timer and self._software_ramp_timer.isActive():
                self._software_ramp_timer.stop()
            self._ramping_axes.clear()
            self.sigQuenchState.emit({'quench': True, 'axes': list(axes.keys())})
        # Do not send any sweep or heater commands (non-interference policy)
