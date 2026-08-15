# -*- coding: utf-8 -*-
"""
Coarse actuator implemented via Red Pitaya lock app:
- Uses aux_A (or aux_B) register mapped to OUT2 via output mux.
- Implement CoarseActuatorInterface with simple set/get operations.
"""

from typing import Dict

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.module import Base
from qudi.util.mutex import Mutex

from qudi.interface.cavity_control_interface import CoarseActuatorInterface, RedPitayaLockInterface


class CoarseActuatorRP(Base, CoarseActuatorInterface):
    _rp = Connector(name='rp', interface=RedPitayaLockInterface)
    _use_aux: str = ConfigOption('use_aux', default='aux_A', missing='warn')  # 'aux_A' or 'aux_B'
    _out_sel_value: int = ConfigOption('out_sel_value', default=14, missing='warn')  # out*_sw value for aux_*

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._active = False
        self._last_v = 0.0
        self._routed = False

    def on_activate(self) -> None:
        pass

    def on_deactivate(self) -> None:
        try:
            if self._active:
                self.deactivate()
        except Exception:
            pass

    def activate(self) -> None:
        # IMPORTANT: do not use self._lock here; set flag directly to avoid deadlock when called from set_voltage
        if not self._active:
            self._active = True

    def deactivate(self) -> None:
        # Likewise, no lock needed for a simple flag
        self._active = False

    def set_voltage(self, value: float) -> None:
        # Determine if we need to activate without holding the lock (to avoid re-entrancy)
        need_activate = False
        with self._lock:
            need_activate = not self._active
        if need_activate:
            self.activate()

        # Route OUT2 on first use (no lock held during remote call)
        need_route = False
        with self._lock:
            need_route = not self._routed
        if need_route:
            try:
                self._rp().set_out_mux(out2_sel=int(self._out_sel_value))
                with self._lock:
                    self._routed = True
            except Exception:
                self.log.exception('Failed to set OUT2 mux on RP')

        # Clip to RP fast DAC range (±1 V) and clip raw to 14-bit safe range
        v = max(-1.0, min(1.0, float(value)))
        raw = int(round(v * 8192.0 / 1.0))
        raw = max(-8191, min(8191, raw))
        try:
            self._rp().write_reg(self._use_aux, raw)
            with self._lock:
                self._last_v = float(v)
        except Exception:
            self.log.exception('Failed to write RP aux register')
            # swallow exception to keep GUI responsive

    def get_voltage(self) -> float:
        try:
            raw = self._rp().read_reg(self._use_aux)
            v = float(raw) * (1.0 / 8192.0)
            with self._lock:
                self._last_v = v
            return v
        except Exception:
            with self._lock:
                return float(self._last_v)

    def constraints(self) -> Dict[str, float]:
        # Reflect RP fast DAC range
        return {'min': -1.0, 'max': 1.0, 'unit': 'V'}