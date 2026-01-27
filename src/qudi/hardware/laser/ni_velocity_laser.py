# -*- coding: utf-8 -*-
"""
Velocity laser control via NI USB analog output implementing ScannableLaserInterface.


Copyright (c) 2024, the qudi developers. See the AUTHORS.md file at the top-level directory of this
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

import time
from typing import Optional, Tuple

import numpy as np
from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.core.module import Base
from qudi.interface.scannable_laser_interface import (
    ScannableLaserInterface,
    ScannableLaserConstraints,
    ScannableLaserSettings,
    LaserScanMode,
    LaserScanDirection,
)
from qudi.util.constraints import ScalarConstraint

# Import nidaqmx lazily guarded
try:
    import nidaqmx as ni
    from nidaqmx.constants import AcquisitionType, TerminalConfiguration
    from nidaqmx.stream_writers import AnalogMultiChannelWriter
except Exception:
    ni = None


class NiVelocityLaser(ScannableLaserInterface):
    """
    This is the Hardware class for the control of sirah matisse laser.

    Example config:

    ni_velocity_laser:
        module.Class: 'laser.ni_velocity_laser.NiVelocityLaser'
        allow_remote: True
        options:
            device_name: 'Dev3'
            limits: [-10.0, 10.0]
            sense_channel: 'ai0'         # optional, readback for get_setpoint (via AI)
            position_bounds: [-10, 10]
            speed_bounds: [0.0, 1.0]
            speed_default: 0.1
            poll_interval_ms: 200
            sample_rate_hz: 5000
    """

    # NI device options
    _device_name: str = ConfigOption('device_name', default='Dev1', missing='error')
    _sense_channel: Optional[str] = ConfigOption('sense_channel', default='ai0', missing='nothing')

    # Voltage limits and speeds (V / V/s)
    _value_bounds: Tuple[float, float] = ConfigOption('position_bounds', default=(-10.0, 10.0), missing='warn')
    _speed_bounds: Tuple[float, float] = ConfigOption('speed_bounds', default=(0.001, 5.0), missing='warn')
    _speed_default: float = ConfigOption('speed_default', default=0.5, missing='warn')

    # Scan supervision poll interval (ms)
    _poll_interval_ms: int = ConfigOption('poll_interval_ms', default=150, missing='warn')

    # Internal sample rate for AO waveform generation (Hz)
    _sample_rate_hz: float = ConfigOption('sample_rate_hz', default=5000.0, missing='warn')

    sigPositionChanged = QtCore.Signal(float)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._constraints: Optional[ScannableLaserConstraints] = None
        self._settings: Optional[ScannableLaserSettings] = None

        self._ao_task = None
        self._ai_task = None
        self._writer = None

        self._current_direction: LaserScanDirection = LaserScanDirection.UP
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(False)
        self._timer.setInterval(int(self._poll_interval_ms))
        self._timer.timeout.connect(self._supervise, QtCore.Qt.QueuedConnection)

        # Cache for restart (bounds and speed)
        self._last_bounds: Optional[Tuple[float, float]] = None
        self._last_speed: Optional[float] = None
        self._last_output: Optional[float] = None

    # ---------- lifecycle ----------
    def on_activate(self):
        if ni is None:
            raise RuntimeError('nidaqmx not available. Please install the NI-DAQmx Python package.')

        # Build constraints
        lo, hi = tuple(self._value_bounds)
        center = 0.5 * (lo + hi)
        self._last_output = center
        value_c = ScalarConstraint(default=center, bounds=(lo, hi), increment=1e-4)
        speed_c = ScalarConstraint(default=self._speed_default, bounds=tuple(self._speed_bounds), increment=1e-3)
        reps_c = ScalarConstraint(default=1, bounds=(1, 10000), increment=1, enforce_int=True)

        self._constraints = ScannableLaserConstraints(
            value=value_c,
            unit='V',
            speed=speed_c,
            repetitions=reps_c,
            initial_directions=(LaserScanDirection.UP, LaserScanDirection.DOWN),
            modes=(LaserScanMode.CONTINUOUS, LaserScanMode.REPETITIONS),
        )

        # Default settings (no move)
        self._settings = ScannableLaserSettings(
            bounds=(lo, hi),
            speed=float(self._speed_default),
            mode=LaserScanMode.CONTINUOUS,
            repetitions=0,
            initial_direction=LaserScanDirection.UP
        )

        if self._sense_channel:
            self._create_ai_task()

    def on_deactivate(self):
        try:
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            if self.module_state() == 'locked':
                self.stop_scan()
        finally:
            self._terminate_ao_task()
            self._terminate_ai_task()

    # ---------- ScannableLaserInterface ----------
    @property
    def constraints(self) -> ScannableLaserConstraints:
        return self._constraints

    @property
    def scan_settings(self) -> ScannableLaserSettings:
        return self._settings

    def configure_scan(self,
                       bounds: Tuple[float, float],
                       speed: float,
                       mode: LaserScanMode,
                       repetitions: Optional[int] = 0,
                       initial_direction: Optional[LaserScanDirection] = LaserScanDirection.UNDEFINED) -> None:
        if self.module_state() != 'idle':
            raise RuntimeError('Cannot configure scan while scan is running.')

        lo, hi = float(bounds[0]), float(bounds[1])
        lo = max(self._value_bounds[0], min(lo, self._value_bounds[1]))
        hi = max(self._value_bounds[0], min(hi, self._value_bounds[1]))
        if lo >= hi:
            raise ValueError('bounds min must be < max')

        spd = float(speed)
        self._constraints.speed.check(spd)

        mode_enum = mode if isinstance(mode, LaserScanMode) else LaserScanMode(int(mode))
        reps = int(repetitions or 0)
        if mode_enum == LaserScanMode.REPETITIONS and reps < 1:
            raise ValueError('repetitions must be >= 1 for REPETITIONS mode')

        init_dir = (initial_direction if isinstance(initial_direction, LaserScanDirection)
                    else LaserScanDirection.UP if initial_direction == LaserScanDirection.UNDEFINED
                    else LaserScanDirection(int(initial_direction)))

        # Apply cached for restarts
        self._last_bounds = (lo, hi)
        self._last_speed = spd

        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=spd,
                                                mode=mode_enum,
                                                repetitions=reps,
                                                initial_direction=init_dir)

    def start_scan(self) -> None:
        if self.module_state() == 'locked':
            return

        # Restart AO task with a continuous triangle waveform
        self._current_direction = self._settings.initial_direction
        self._start_continuous_triangle(bounds=self._settings.bounds,
                                        speed=self._settings.speed,
                                        direction=self._current_direction)
        self.module_state.lock()
        QtCore.QMetaObject.invokeMethod(self, '_timer_start', QtCore.Qt.QueuedConnection)

    def stop_scan(self) -> None:
        try:
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            self._terminate_ao_task()
        finally:
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def scan_to(self, value: float, blocking: Optional[bool] = False) -> None:
        if self.module_state() != 'idle':
            raise RuntimeError('Cannot move while scan is running. Stop the scan first.')

        lo, hi = self._settings.bounds
        target = max(lo, min(float(value), hi))
        start = self._last_output if (self._last_output is not None) else target
        if start == target:
            self.sigPositionChanged.emit(target)
            self._last_output = target
            return

        # Generate finite ramp to target
        sr = float(self._sample_rate_hz)
        speed = float(self._settings.speed)
        duration_s = abs(target - start) / max(speed, 1e-9)
        n = max(2, int(duration_s * sr))
        wave = np.linspace(start, target, n, dtype=float)

        task = None
        try:
            task = ni.Task('Velocity AO Finite')
            task.ao_channels.add_ao_voltage_chan(f'/{self._device_name}/ao0:1',
                                                 min_val=self._value_bounds[0],
                                                 max_val=self._value_bounds[1])
            task.timing.cfg_samp_clk_timing(sr, sample_mode=AcquisitionType.FINITE, samps_per_chan=n)
            writer = AnalogMultiChannelWriter(task.out_stream)
            frame = np.vstack([wave, wave])  # same waveform to ao0 and ao1
            writer.write_many_sample(frame)
            task.start()
            if blocking:
                # Wait until estimated duration has passed
                time.sleep(n / sr)
        finally:
            try:
                if task is not None:
                    if not task.is_task_done():
                        task.stop()
                    task.close()
            except Exception:
                pass

        self.sigPositionChanged.emit(target)
        self._last_output = target

    # ---------- public helpers ----------
    def set_scan_speeds(self, rising_speed: float, falling_speed: Optional[float] = None) -> None:
        """
        For NI AO triangle scan, a single “speed” (V/s) is used to derive the sample count.
        This setter stores the speed in settings (falling_speed ignored; we use symmetric speed).
        """
        spd = float(rising_speed)
        self._constraints.speed.check(spd)
        self._settings = ScannableLaserSettings(bounds=self._settings.bounds,
                                                speed=spd,
                                                mode=self._settings.mode,
                                                repetitions=self._settings.repetitions,
                                                initial_direction=self._settings.initial_direction)
        self._last_speed = spd

    def set_scan_bounds(self, lower: float, upper: float) -> None:
        lo = float(lower)
        hi = float(upper)
        if lo >= hi:
            raise ValueError('lower must be < upper')
        lo = max(self._value_bounds[0], min(lo, self._value_bounds[1]))
        hi = max(self._value_bounds[0], min(hi, self._value_bounds[1]))
        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=self._settings.speed,
                                                mode=self._settings.mode,
                                                repetitions=self._settings.repetitions,
                                                initial_direction=self._settings.initial_direction)
        self._last_bounds = (lo, hi)

    def set_scan_direction(self, direction: LaserScanDirection) -> None:
        """
        Change scan direction while running: restart the continuous AO task with the new initial half-cycle.
        """
        if not isinstance(direction, LaserScanDirection):
            direction = LaserScanDirection(int(direction))
        self._current_direction = direction

        if self.module_state() == 'locked':
            # Restart with current cached params
            bounds = self._last_bounds or self._settings.bounds
            speed = float(self._last_speed or self._settings.speed)
            self._start_continuous_triangle(bounds=bounds, speed=speed, direction=direction)

    # ---------- supervision ----------
    @QtCore.Slot()
    def _supervise(self) -> None:
        """
        Minimal supervision: reads AI position (if available) to keep the GUI responsive and
        optionally re-emit position (no automatic flipping is needed because the waveform is a triangle).
        """
        try:
            if self.module_state() != 'locked':
                return
            pos = self._read_position()
            if pos is not None:
                self.sigPositionChanged.emit(pos)
                self._last_output = pos
        except Exception:
            self.log.exception('NI AO supervision failed, stopping scan.')
            self.stop_scan()

    @QtCore.Slot()
    def _timer_start(self) -> None:
        try:
            self._timer.start()
        except Exception:
            pass

    @QtCore.Slot()
    def _timer_stop(self) -> None:
        try:
            self._timer.stop()
        except Exception:
            pass

    # ---------- NI-DAQ helpers ----------
    def _start_continuous_triangle(self,
                                   bounds: Tuple[float, float],
                                   speed: float,
                                   direction: LaserScanDirection) -> None:
        """
        Start (or restart) a continuous triangle scan between bounds at given speed.
        Waveform is rotated so the first sample matches the current output (minimizes jump).
        """
        lo, hi = bounds
        sr = float(self._sample_rate_hz)
        span = float(hi - lo)
        duration_one_way = span / max(float(speed), 1e-9)
        n_one = max(2, int(duration_one_way * sr))

        up = np.linspace(lo, hi, n_one, dtype=float)
        down = up[::-1]
        base = np.concatenate([up, down]) if direction == LaserScanDirection.UP else np.concatenate([down, up])

        # Prefer AI readback; else last_output; else center
        start_val = self._read_position()
        if start_val is None:
            start_val = self._last_output if (self._last_output is not None) else 0.5 * (lo + hi)
        start_val = float(max(lo, min(start_val, hi)))

        nearest_idx = int(np.argmin(np.abs(base - start_val)))
        wave = np.concatenate([base[nearest_idx:], base[:nearest_idx]])
        wave[0] = start_val

        try:
            self._ao_task = ni.Task('Velocity AO Continuous')
            self._ao_task.ao_channels.add_ao_voltage_chan(f'/{self._device_name}/ao0:1',
                                                          min_val=self._value_bounds[0],
                                                          max_val=self._value_bounds[1])
            self._ao_task.timing.cfg_samp_clk_timing(sr,
                                                     sample_mode=AcquisitionType.CONTINUOUS,
                                                     samps_per_chan=wave.size)
            self._writer = AnalogMultiChannelWriter(self._ao_task.out_stream)
            frame = np.vstack([wave, wave])
            self._writer.write_many_sample(frame)
            self._ao_task.start()
            self._last_output = start_val
        except Exception:
            self._terminate_ao_task()
            raise

    def _terminate_ao_task(self):
        if self._ao_task is not None:
            try:
                if not self._ao_task.is_task_done():
                    self._ao_task.stop()
                self._ao_task.close()
            except Exception:
                pass
            self._ao_task = None
            self._writer = None

    def _create_ai_task(self):
        if self._ai_task is not None:
            return
        try:
            self._ai_task = ni.Task('Velocity AI Readback')
            self._ai_task.ai_channels.add_ai_voltage_chan(
                f'/{self._device_name}/{self._sense_channel}',
                min_val=self._value_bounds[0],
                max_val=self._value_bounds[1],
                terminal_config=TerminalConfiguration.RSE
            )
        except Exception:
            self._terminate_ai_task()
            raise

    def _terminate_ai_task(self):
        if self._ai_task is not None:
            try:
                if not self._ai_task.is_task_done():
                    self._ai_task.stop()
                self._ai_task.close()
            except Exception:
                pass
            self._ai_task = None

    def _read_position(self) -> Optional[float]:
        if self._ai_task is None:
            return None
        try:
            # average a few samples for a stable value
            return float(np.mean(self._ai_task.read(100)))
        except Exception:
            return None