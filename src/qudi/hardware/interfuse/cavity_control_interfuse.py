# -*- coding: utf-8 -*-
"""
Cavity control interfuse:
- Connects RedPitayaLockInterface (fine AO1 ramp + osc) and CoarseActuatorInterface (AO2 or external)
- Exposes unified CavityControlInterface to logic:
  - Set alpha and ramp/osc params
  - Set/get coarse voltage
  - Perform single ramp acquisition and return correlated (V_eff, AI1)
"""

import math
from typing import Tuple, Sequence, Dict, Optional

import numpy as np
from PySide2 import QtCore

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.module import Base
from qudi.core.statusvariable import StatusVar
from qudi.util.mutex import Mutex

from qudi.interface.cavity_control_interface import RedPitayaLockInterface, CoarseActuatorInterface, CavityControlInterface


class CavityControlInterfuse(Base, CavityControlInterface):
    # Connections
    _rp = Connector(name='rp', interface=RedPitayaLockInterface)
    _coarse = Connector(name='coarse', interface=CoarseActuatorInterface)

    # Persisted parameters
    _alpha: float = StatusVar('alpha', default=10.0)  # AO1 attenuated by 1/alpha relative to AO2
    _ao1_low_v: float = StatusVar('ao1_low_v', default=0.0)
    _ao1_high_v: float = StatusVar('ao1_high_v', default=1.0)
    _ao1_speed_vps: float = StatusVar('ao1_speed_vps', default=1.0)
    _ao1_direction_up: bool = StatusVar('ao1_direction_up', default=True)
    _ao1_sawtooth: bool = StatusVar('ao1_sawtooth', default=False)

    _osc_decimation: int = StatusVar('osc_decimation', default=128)
    _osc_trigger_source: int = StatusVar('osc_trigger_source', default=2)  # 2=Scan floor
    _osc_trig_pos: int = StatusVar('osc_trig_pos', default=8191)
    _osc_hysteresis: int = StatusVar('osc_hysteresis', default=1)
    _osc_threshold: int = StatusVar('osc_threshold', default=0)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._ao2_last_v = 0.0

    def on_activate(self) -> None:
        # Ensure RP connected
        try:
            self._rp().connect()
        except Exception:
            pass

    def on_deactivate(self) -> None:
        try:
            self._coarse().deactivate()
        except Exception:
            pass
        try:
            self._rp().disconnect()
        except Exception:
            pass

    # CavityControlInterface impl
    def set_alpha(self, alpha: float) -> None:
        with self._lock:
            self._alpha = float(alpha)

    def set_ao1_ramp_params(self,
                            low_lim_v: float,
                            high_lim_v: float,
                            speed_v_per_s: float,
                            direction_up: bool,
                            sawtooth: bool = False) -> None:
        with self._lock:
            self._ao1_low_v = float(low_lim_v)
            self._ao1_high_v = float(high_lim_v)
            self._ao1_speed_vps = float(speed_v_per_s)
            self._ao1_direction_up = bool(direction_up)
            self._ao1_sawtooth = bool(sawtooth)
            # Convert to 14-bit limits and step_ticks (empirical mapping: speed -> ticks)
            low_raw = int(round(self._ao1_low_v * 8192.0 / 1.0))
            high_raw = int(round(self._ao1_high_v * 8192.0 / 1.0))
            step_ticks = self._estimate_step_ticks(self._ao1_speed_vps, self._ao1_low_v, self._ao1_high_v)
            self._rp().configure_ramp(low_lim=low_raw,
                                      high_lim=high_raw,
                                      step_ticks=step_ticks,
                                      direction_up=self._ao1_direction_up,
                                      sawtooth=self._ao1_sawtooth)

    def set_osc_params(self,
                       decimation: int,
                       trigger_source: int,
                       trig_pos: int = 8191,
                       hysteresis: int = 1,
                       threshold: int = 0) -> None:
        with self._lock:
            self._osc_decimation = int(decimation)
            self._osc_trigger_source = int(trigger_source)
            self._osc_trig_pos = int(trig_pos)
            self._osc_hysteresis = int(hysteresis)
            self._osc_threshold = int(threshold)
            self._apply_osc_params()

    def _apply_osc_params(self) -> None:
        try:
            self._rp().osc_config(decimation=int(self._osc_decimation),
                                  trigger_source=int(self._osc_trigger_source),
                                  trig_pos=int(self._osc_trig_pos),
                                  hysteresis=int(self._osc_hysteresis),
                                  threshold=int(self._osc_threshold))
        except Exception:
            pass

    def set_coarse_voltage(self, value: float) -> None:
        with self._lock:
            self._coarse().set_voltage(float(value))
            self._ao2_last_v = float(value)

    def get_coarse_voltage(self) -> float:
        with self._lock:
            try:
                self._ao2_last_v = float(self._coarse().get_voltage())
            except Exception:
                pass
            return float(self._ao2_last_v)

    def single_ramp_acquire(self) -> Tuple[Sequence[float], Sequence[float]]:
        """
        Route oscA to Ramp A, oscB to IN1 (photodiode), run single-shot acquisition.
        Compute V_eff = V_AO2 + RampA/alpha and return (V_eff, AI1).
        """
        with self._lock:
            # Route outputs/osc sources: oscA_sw should show Ramp A, oscB_sw IN1 (mapping uses the app defaults)
            # Here we assume osc.curv returns ch1=oscA selected signal and ch2=oscB selected signal.
            try:
                # Ensure ramp enabled
                self._rp().enable_ramp(True)
                # Trigger acquisition
                self._rp().osc_acquire(wait=True)
                t, chA, chB = self._rp().osc_curves(raw=False)
            except Exception as e:
                self.log.exception('Osc acquisition failed')
                raise

            # chA is Ramp A in volts; coarse AO2 in volts
            v2 = float(self.get_coarse_voltage())
            alpha = float(self._alpha) if self._alpha != 0 else 1.0
            rampA = np.array(chA, dtype=float)
            ai1 = np.array(chB, dtype=float)
            v_eff = v2 + (rampA / alpha)
            return v_eff.tolist(), ai1.tolist()

    def get_ao1_position(self) -> float:
        try:
            v = self._rp().get_ao1_position()
            return float(v if v is not None else 0.0)
        except Exception:
            return 0.0

    def get_constraints(self) -> Dict[str, Dict[str, float]]:
        # AO1 constraints: from StatusVar limits; AO2 from coarse constraints
        ao2c = {}
        try:
            ao2c = self._coarse().constraints()
        except Exception:
            ao2c = {'min': -10.0, 'max': 10.0, 'unit': 'V'}
        return {
            'ao1': {'min': float(self._ao1_low_v), 'max': float(self._ao1_high_v), 'unit': 'V'},
            'ao2': ao2c
        }

    # Helpers
    def _estimate_step_ticks(self, speed_vps: float, low_v: float, high_v: float) -> int:
        """
        Estimate ramp_step ticks for the desired speed (V/s).
        The FPGA ramp moves one 14-bit LSB per 'ramp_step' ticks; this mapping is device-dependent.
        We approximate: ticks ≈ K * span_volts / speed, where K is a constant derived empirically.
        Here we use a conservative default K=125e3 to match typical 125 MHz clock scaling to user units.

        You can tune this empirically later in logic.
        """
        span = max(1e-9, float(high_v) - float(low_v))
        K = 125000.0  # heuristic scaling factor
        ticks = int(max(1, round(K * span / max(1e-6, float(speed_vps)))))
        return ticks