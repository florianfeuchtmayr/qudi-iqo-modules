# -*- coding: utf-8 -*-
"""
Coarse actuator wrapper around NIXSeriesAnalogOutput (ProcessSetpointInterface).
Implements CoarseActuatorInterface by delegating to a configured NI AO channel.
"""

from typing import Dict

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.module import Base
from qudi.util.mutex import Mutex

from qudi.interface.cavity_control_interface import CoarseActuatorInterface
from qudi.interface.process_control_interface import ProcessSetpointInterface


class CoarseActuatorNI(Base, CoarseActuatorInterface):
    _backend = Connector(name='backend', interface=ProcessSetpointInterface)
    _channel: str = ConfigOption('channel', default='ao0', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Mutex()
        self._active = False
        self._last_v = 0.0

    def on_activate(self) -> None:
        pass

    def on_deactivate(self) -> None:
        try:
            if self._active:
                self.deactivate()
        except Exception:
            pass

    def activate(self) -> None:
        with self._lock:
            if not self._active:
                self._backend().set_activity_state(self._channel, True)
                self._active = True

    def deactivate(self) -> None:
        with self._lock:
            if self._active:
                try:
                    self._backend().set_activity_state(self._channel, False)
                finally:
                    self._active = False

    def set_voltage(self, value: float) -> None:
        with self._lock:
            if not self._active:
                self.activate()
            self._backend().set_setpoint(self._channel, float(value))
            self._last_v = float(value)

    def get_voltage(self) -> float:
        with self._lock:
            if self._active:
                try:
                    self._last_v = float(self._backend().get_setpoint(self._channel))
                except Exception:
                    pass
            return float(self._last_v)

    def constraints(self) -> Dict[str, float]:
        cons = self._backend().constraints
        try:
            lims = cons.limits[self._channel]
            return {'min': float(lims[0]), 'max': float(lims[1]), 'unit': 'V'}
        except Exception:
            return {'min': -10.0, 'max': 10.0, 'unit': 'V'}