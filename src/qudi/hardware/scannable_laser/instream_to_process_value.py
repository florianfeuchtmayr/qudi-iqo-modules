# -*- coding: utf-8 -*-
"""
Expose one DataInStreamInterface channel as a ProcessValueInterface for PID.

This interfuse turns an in-streaming hardware (e.g. wavemeter buffer) into a single-channel
ProcessValueInterface that SoftPIDController can consume.

Example config:

wavemeter_process_value:
    module.Class: 'interfuse.instream_to_process_value.InStreamProcessValueInterfuse'
    connect:
        streamer: 'wavemeter_buffer'
    options:
        channel_name: 'dye_laser'     # choose which wavemeter channel drives the PID
        autostart_stream: True        # start/stop stream around reads if idle
        keep_running: False           # keep stream on while PID enabled (optional)
        unit: 'nm'                    # optional override if needed
"""

import time
import numpy as np
from typing import Dict, Optional

from PySide2 import QtCore
from qudi.core.module import Base
from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.interface.process_control_interface import ProcessValueInterface
from qudi.interface.data_instream_interface import DataInStreamInterface, StreamingMode
from qudi.util.network import netobtain  # <-- ensure we import this

class InStreamProcessValueInterfuse(ProcessValueInterface):

    streamer = Connector(name='streamer', interface='DataInStreamInterface')

    _channel_name: str = ConfigOption(name='channel_name', default='default_channel', missing='error')
    _autostart_stream: bool = ConfigOption(name='autostart_stream', default=True, missing='warn')
    _keep_running: bool = ConfigOption(name='keep_running', default=False, missing='warn')
    _unit_override: Optional[str] = ConfigOption(name='unit', default=None, missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._constraints = type('PVConstraints', (), {})()
        self._active: bool = False
        self._configured: bool = False

    def on_activate(self):
        st = self.streamer()

        # Safely fetch remote constraints by value
        try:
            cons = netobtain(st.constraints)
            units = dict(getattr(cons, 'channel_units', {}))
        except Exception:
            units = {}

        unit = units.get(self._channel_name, None)
        if self._unit_override:
            unit = self._unit_override
        if unit is None:
            unit = 'arb.'

        # shape a simple constraints object the PID expects
        limits = (-np.inf, np.inf)
        self._constraints.channel_units = {self._channel_name: unit}
        self._constraints.channel_limits = {self._channel_name: limits}
        '''
        # Try to configure the streamer with primitives only (no Enum objects across RPyC)
        try:
            # Read these by value to avoid passing netrefs back in
            cbs = float(netobtain(st.channel_buffer_size)) if hasattr(st, 'channel_buffer_size') else None
            sr = float(netobtain(st.sample_rate)) if hasattr(st, 'sample_rate') else None
            st.configure(
                active_channels=[self._channel_name],
                streaming_mode=int(0),  # pass primitive
                channel_buffer_size=cbs,
                sample_rate=sr,
            )
            self._configured = True
        except Exception as e:
            print(e)
            # If configuration is blocked (buffer shared) we still can read the latest sample
            self._configured = False'''

    def on_deactivate(self):
        try:
            if self.streamer().module_state() == 'locked':
                self.streamer().stop_stream()
        except Exception:
            pass
        self._active = False

    @property
    def constraints(self):
        return self._constraints

    def set_activity_state(self, channel: str, active: bool) -> None:
        if channel != self._channel_name:
            return
        self._active = bool(active)
        st = self.streamer()
        try:
            if self._active:
                if st.module_state() == 'idle' and self._autostart_stream:
                    st.start_stream()
            else:
                if st.module_state() == 'locked' and (self._autostart_stream and not self._keep_running):
                    st.stop_stream()
        except Exception:
            pass

    def get_activity_state(self, channel: str) -> bool:
        return bool(self._active) if channel == self._channel_name else False

    def get_process_value(self, channel: Optional[str] = None) -> float:
        if channel is None:
            channel = self._channel_name
        if channel != self._channel_name:
            raise ValueError(f'Unknown channel "{channel}"')

        st = self.streamer()
        started_here = False

        if self._autostart_stream and (st.module_state() == 'idle'):
            try:
                st.start_stream()
                started_here = True
            except Exception:
                pass

        try:
            # Allow short wait for at least one sample
            t0 = time.time()
            while netobtain(st.available_samples) < 1 and (time.time() - t0) < 0.5:
                time.sleep(0.01)

            data, _ = st.read_single_point()
            data = netobtain(data)  # ensure local ndarray
            # choose index robustly
            try:
                active = netobtain(st.active_channels)
            except Exception:
                active = [channel]
            idx = 0 if (len(active) == 1) else (active.index(self._channel_name) if self._channel_name in active else 0)

            arr = np.atleast_1d(data)
            val = float(arr[idx]) if len(arr) > idx else float('nan')
            return val
        finally:
            if started_here and not self._keep_running:
                try:
                    st.stop_stream()
                except Exception:
                    pass