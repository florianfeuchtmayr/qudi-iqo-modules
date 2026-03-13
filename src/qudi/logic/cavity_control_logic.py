# -*- coding: utf-8 -*-
"""
Single logic module for:
- Full scan orchestration (step AO2, ramp AO1, acquire osc, stitch V_eff vs AI1)
- Apply constant coarse voltage
- Drift tracker (keep AO1 centered by slow AO2 adjustments)

Logic connects ONLY to the cavity control interfuse (CavityControlInterface).
"""

import time
from typing import List, Tuple

import numpy as np
from PySide2 import QtCore

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
from qudi.util.mutex import Mutex
from qudi.core.module import LogicBase

from qudi.interface.cavity_control_interface import CavityControlInterface


class CavityControlLogic(LogicBase):
    cavity = Connector(name='cavity', interface=CavityControlInterface)

    # Persist UI parameters
    alpha: float = StatusVar('alpha', default=10.0)
    ao1_low_v: float = StatusVar('ao1_low_v', default=0.0)
    ao1_high_v: float = StatusVar('ao1_high_v', default=1.0)
    ao1_speed_vps: float = StatusVar('ao1_speed_vps', default=1.0)

    # Osc
    osc_decimation: int = StatusVar('osc_decimation', default=128)
    osc_trigger_source: int = StatusVar('osc_trigger_source', default=2)  # Scan floor

    # Drift tracker params
    drift_enable: bool = StatusVar('drift_enable', default=False)
    drift_center_pct: float = StatusVar('drift_center_pct', default=50.0)
    drift_speed_vps: float = StatusVar('drift_speed_vps', default=0.02)  # slow
    drift_deadband_uv: float = StatusVar('drift_deadband_uv', default=10.0)  # microvolts at AO1
    drift_bounds_v: Tuple[float, float] = StatusVar('drift_bounds_v', default=(None, None))

    # Signals
    sigFullScanProgress = QtCore.Signal(int, int)  # current_step, total_steps
    sigFullScanChunk = QtCore.Signal(list, list)   # V_eff, AI1
    sigFullScanDone = QtCore.Signal()

    sigCoarseVoltageSet = QtCore.Signal(float)
    sigAOPositions = QtCore.Signal(float, float)   # AO1, AO2

    sigDriftStatus = QtCore.Signal(bool, float)    # enabled, current error

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._drift_timer = QtCore.QTimer(self)
        self._drift_timer.setSingleShot(False)
        self._drift_timer.setInterval(200)  # ms
        self._drift_timer.timeout.connect(self._drift_tick, QtCore.Qt.QueuedConnection)

    def on_activate(self) -> None:
        # Apply initial params to interfuse
        self.cavity().set_alpha(float(self.alpha))
        # Start drift if enabled
        if bool(self.drift_enable):
            QtCore.QMetaObject.invokeMethod(self, '_start_drift', QtCore.Qt.QueuedConnection)

    def on_deactivate(self) -> None:
        QtCore.QMetaObject.invokeMethod(self, '_stop_drift', QtCore.Qt.QueuedConnection)

    # Public API for GUI
    @QtCore.Slot(float)
    def set_alpha(self, alpha: float) -> None:
        self.alpha = float(alpha)
        try:
            self.cavity().set_alpha(float(alpha))
        except Exception:
            pass

    @QtCore.Slot(float, float, float)
    def set_ao1_params(self, low_v: float, high_v: float, speed_vps: float) -> None:
        self.ao1_low_v = float(low_v)
        self.ao1_high_v = float(high_v)
        self.ao1_speed_vps = float(speed_vps)
        try:
            self.cavity().set_ao1_ramp_params(float(low_v), float(high_v), float(speed_vps), True, False)
        except Exception:
            pass

    @QtCore.Slot(int, int)
    def set_osc_params(self, decimation: int, trigger_source: int) -> None:
        self.osc_decimation = int(decimation)
        self.osc_trigger_source = int(trigger_source)
        try:
            self.cavity().set_osc_params(int(decimation), int(trigger_source))
        except Exception:
            pass

    @QtCore.Slot(float, float)
    def full_scan(self, coarse_start_v: float, coarse_end_v: float) -> None:
        """
        Perform full scan by stepping AO2 by ΔV_AO2 = AO1_range / alpha between ramp acquisitions.
        """
        with self._lock:
            alpha = max(1e-9, float(self.alpha))
            span_ao1 = float(self.ao1_high_v) - float(self.ao1_low_v)
            delta_ao2 = float(span_ao1) / alpha  # one AO1 window per coarse step
            start = float(coarse_start_v)
            end = float(coarse_end_v)
            if delta_ao2 <= 0.0:
                self.log.error('Invalid AO1 span or alpha; aborting full scan.')
                return
            # Determine steps
            n_steps = int(max(1, round(abs(end - start) / delta_ao2)))
            direction = 1 if end >= start else -1
            v = start
            self.sigFullScanProgress.emit(0, n_steps)
            for k in range(n_steps):
                try:
                    self.cavity().set_coarse_voltage(v)
                    v_eff, ai1 = self.cavity().single_ramp_acquire()
                    self.sigFullScanChunk.emit(v_eff, ai1)
                except Exception:
                    self.log.exception('Full scan step failed')
                    break
                self.sigFullScanProgress.emit(k + 1, n_steps)
                v += direction * delta_ao2
            self.sigFullScanDone.emit()

    @QtCore.Slot(float)
    def apply_coarse_voltage(self, value: float) -> None:
        try:
            self.cavity().set_coarse_voltage(float(value))
            ao2 = float(self.cavity().get_coarse_voltage())
            ao1 = float(self.cavity().get_ao1_position())
            self.sigCoarseVoltageSet.emit(ao2)
            self.sigAOPositions.emit(ao1, ao2)
        except Exception:
            self.log.exception('Apply coarse voltage failed')

    # Drift tracker
    @QtCore.Slot(bool)
    def enable_drift(self, enable: bool) -> None:
        self.drift_enable = bool(enable)
        if enable:
            QtCore.QMetaObject.invokeMethod(self, '_start_drift', QtCore.Qt.QueuedConnection)
        else:
            QtCore.QMetaObject.invokeMethod(self, '_stop_drift', QtCore.Qt.QueuedConnection)

    @QtCore.Slot()
    def _start_drift(self) -> None:
        try:
            self._drift_timer.start()
        except Exception:
            pass

    @QtCore.Slot()
    def _stop_drift(self) -> None:
        try:
            self._drift_timer.stop()
        except Exception:
            pass

    @QtCore.Slot()
    def _drift_tick(self) -> None:
        """
        Slow recentering: compute error at AO1, convert to AO2 correction by ΔV2 = e/alpha,
        rate-limit by drift_speed_vps and apply via cavity.set_coarse_voltage.
        """
        try:
            if not bool(self.drift_enable):
                return
            ao1 = float(self.cavity().get_ao1_position())
            # Center target
            center = float(self.ao1_low_v) + 0.5 * (float(self.ao1_high_v) - float(self.ao1_low_v))
            e_v = center - ao1  # volts at AO1
            # Deadband
            if abs(e_v) * 1e6 < float(self.drift_deadband_uv):
                self.sigDriftStatus.emit(True, 0.0)
                return
            alpha = max(1e-9, float(self.alpha))
            dv2_need = e_v / alpha  # volts at AO2
            # Rate limit per tick
            dt = float(self._drift_timer.interval()) / 1000.0
            max_step = float(self.drift_speed_vps) * dt
            step = max(-max_step, min(max_step, dv2_need))
            # Apply bounds if provided
            ao2_now = float(self.cavity().get_coarse_voltage())
            lo, hi = self.drift_bounds_v if isinstance(self.drift_bounds_v, tuple) else (None, None)
            ao2_new = ao2_now + step
            if lo is not None:
                ao2_new = max(float(lo), ao2_new)
            if hi is not None:
                ao2_new = min(float(hi), ao2_new)
            self.cavity().set_coarse_voltage(ao2_new)
            self.sigAOPositions.emit(ao1, ao2_new)
            self.sigDriftStatus.emit(True, e_v)
        except Exception:
            self.log.exception('Drift tracker tick failed')
            self.sigDriftStatus.emit(False, 0.0)