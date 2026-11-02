# -*- coding: utf-8 -*-
"""
Velocity laser piezo control via NI USB analog output (e.g. NI USB-6002) implementing
ScannableLaserInterface. Intended to run on the PC the NI card is attached to and to be connected
remotely to the laser scanning logic.

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

from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
from qudi.util.mutex import RecursiveMutex
from qudi.util.constraints import ScalarConstraint
from qudi.interface.scannable_laser_interface import (
    ScannableLaserInterface,
    ScannableLaserConstraints,
    ScannableLaserSettings,
    LaserScanMode,
    LaserScanDirection,
)

try:
    import nidaqmx as ni
    from nidaqmx.constants import AcquisitionType, TerminalConfiguration
    from nidaqmx.stream_writers import AnalogMultiChannelWriter
except ImportError:
    ni = None


def _to_enum(enum_cls, value):
    """ Coerce enum members, remote proxies of enum members, names or values into enum_cls """
    if isinstance(value, enum_cls):
        return value
    name = getattr(value, 'name', None)
    if isinstance(name, str) and name in enum_cls.__members__:
        return enum_cls[name]
    if isinstance(value, str):
        return enum_cls[value.strip().upper()]
    return enum_cls(int(value))


class _Waveform:
    """ AO waveform currently loaded into a running NI task, used to estimate output value and
    scan direction from the time elapsed since the task was started. The NI sample counter is not
    used, since on USB devices it counts samples transferred to the onboard FIFO, which runs far
    ahead of the actual output.
    """
    def __init__(self, values: np.ndarray, directions: np.ndarray, sample_rate: float,
                 continuous: bool):
        self.values = values
        self.directions = directions  # +1 for rising, -1 for falling
        self.sample_rate = float(sample_rate)
        self.continuous = continuous
        self.t_start = time.perf_counter()

    def index(self) -> int:
        elapsed = int((time.perf_counter() - self.t_start) * self.sample_rate)
        if self.continuous:
            return elapsed % self.values.size
        return min(elapsed, self.values.size - 1)

    def value(self) -> float:
        return float(self.values[self.index()])

    def direction(self) -> LaserScanDirection:
        return LaserScanDirection.UP if self.directions[self.index()] > 0 else LaserScanDirection.DOWN


class NiVelocityLaser(ScannableLaserInterface):
    """ Scans the piezo voltage of a Velocity laser with the analog outputs of a NI card. The same
    voltage is written to all configured AO channels. An optional AI channel reads the output back.

    Scan direction supervision (flipping at device or wavelength bounds) is done by the laser
    scanning logic. Without supervision the hardware scans a triangle within the configured bounds.

    Example config for copy-paste (on the PC with the NI card):

    ni_velocity_laser:
        module.Class: 'scannable_laser.ni_velocity_laser.NiVelocityLaser'
        allow_remote: True
        options:
            device_name: 'Dev1'
            ao_channels: 'ao0:1'            # optional
            sense_channel: 'ai0'            # optional, AO readback via AI (RSE). null to disable
            position_bounds: [-0.5, 0.5]    # V
            speed_bounds: [0.0001, 1.0]     # V/s
            speed_default: 0.01             # V/s
            scan_sample_rate_hz: 30         # optional, AO rate of scan waveforms
            move_sample_rate_hz: 5000       # optional, AO rate of scan_to ramps
            readback_samples: 100           # optional, AI samples averaged per readback
            max_samples: 10000000           # optional, max. AO waveform length
    """

    _device_name: str = ConfigOption('device_name', default='Dev1', missing='error')
    _ao_channels: str = ConfigOption('ao_channels', default='ao0:1', missing='nothing')
    _sense_channel: Optional[str] = ConfigOption('sense_channel', default='ai0', missing='nothing')

    _value_bounds: Tuple[float, float] = ConfigOption('position_bounds',
                                                      default=(-0.5, 0.5),
                                                      missing='warn',
                                                      constructor=lambda x: tuple(sorted(x)))
    _speed_bounds: Tuple[float, float] = ConfigOption('speed_bounds',
                                                      default=(1e-4, 1.0),
                                                      missing='warn',
                                                      constructor=lambda x: tuple(sorted(x)))
    _speed_default: float = ConfigOption('speed_default', default=0.01, missing='warn')

    _scan_sample_rate: float = ConfigOption('scan_sample_rate_hz', default=30.0, missing='nothing')
    _move_sample_rate: float = ConfigOption('move_sample_rate_hz', default=5000.0, missing='nothing')
    _readback_samples: int = ConfigOption('readback_samples', default=100, missing='nothing')
    _max_samples: int = ConfigOption('max_samples', default=int(1e7), missing='nothing')

    _last_output: Optional[float] = StatusVar(name='last_output', default=None)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = RecursiveMutex()
        self._constraints: Optional[ScannableLaserConstraints] = None
        self._settings: Optional[ScannableLaserSettings] = None
        self._current_direction = LaserScanDirection.UP

        self._ai_task = None
        self._scan_task = None
        self._scan_waveform: Optional[_Waveform] = None
        self._move_task = None
        self._move_waveform: Optional[_Waveform] = None

    # ---------- lifecycle ----------
    def on_activate(self):
        if ni is None:
            raise RuntimeError('nidaqmx not available. Please install the nidaqmx Python package.')

        lo, hi = self._value_bounds
        if lo >= hi:
            raise ValueError(f'Invalid position_bounds {self._value_bounds}')
        self._constraints = ScannableLaserConstraints(
            value=ScalarConstraint(default=0.5 * (lo + hi), bounds=(lo, hi), increment=1e-4),
            unit='V',
            speed=ScalarConstraint(default=self._speed_default,
                                   bounds=self._speed_bounds,
                                   increment=1e-4),
            repetitions=ScalarConstraint(default=1, bounds=(1, 10000), increment=1,
                                         enforce_int=True),
            initial_directions=(LaserScanDirection.UP, LaserScanDirection.DOWN),
            modes=(LaserScanMode.CONTINUOUS, LaserScanMode.REPETITIONS),
        )
        self._current_direction = LaserScanDirection.UP
        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=float(self._speed_default),
                                                mode=LaserScanMode.CONTINUOUS,
                                                repetitions=0,
                                                initial_direction=self._current_direction)

        if self._sense_channel:
            self._create_ai_task()
        position = self._read_ai()
        if position is not None:
            # Do not move the laser on activation if the actual output is known
            self._last_output = position
        else:
            if self._last_output is None:
                self._last_output = 0.5 * (lo + hi)
            self._last_output = self._clip(self._last_output)
            self._write_static(self._last_output)

    def on_deactivate(self):
        try:
            with self._lock:
                self._close_move_task()
                self._close_scan_task()
                if self.module_state() == 'locked':
                    self.module_state.unlock()
        finally:
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
                       initial_direction: Optional[LaserScanDirection] = LaserScanDirection.UNDEFINED
                       ) -> None:
        with self._lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Cannot configure scan while scan is running.')

            lo, hi = sorted(self._clip(float(b)) for b in bounds)
            if lo >= hi:
                raise ValueError(f'Scan bounds {bounds} do not span a range within '
                                 f'{self._value_bounds}')
            speed = float(speed)
            self._constraints.speed.check(speed)
            mode = _to_enum(LaserScanMode, mode)
            repetitions = int(repetitions or 0)
            if mode == LaserScanMode.REPETITIONS and repetitions < 1:
                raise ValueError('repetitions must be >= 1 for REPETITIONS mode')
            direction = _to_enum(LaserScanDirection, initial_direction)
            if direction == LaserScanDirection.UNDEFINED:
                direction = self._current_direction

            self._current_direction = direction
            self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                    speed=speed,
                                                    mode=mode,
                                                    repetitions=repetitions,
                                                    initial_direction=direction)

    def start_scan(self) -> None:
        with self._lock:
            if self.module_state() == 'locked':
                return
            self._close_move_task()

            lo, hi = self._settings.bounds
            start = self._actual_output()
            step = self._settings.speed / self._scan_sample_rate
            if (start < lo - step) or (start > hi + step):
                target = lo if start < lo else hi
                self.log.info(f'Output {start:.4f} V outside of scan bounds. '
                              f'Moving to {target:.4f} V before starting the scan.')
                self._ramp(target, blocking=True)
                start = target

            self._current_direction = self._settings.initial_direction
            self._start_scan_waveform(self._current_direction, start)
            self.module_state.lock()

    def stop_scan(self) -> None:
        with self._lock:
            try:
                self._close_scan_task()
                self._close_move_task()
            finally:
                if self.module_state() == 'locked':
                    self.module_state.unlock()

    def scan_to(self, value: float, blocking: Optional[bool] = False) -> None:
        with self._lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Cannot move while scan is running. Stop the scan first.')
            self._constraints.value.check(float(value))
            self._ramp(float(value), blocking=bool(blocking))

    # ---------- methods used by the laser scanning logic ----------
    def is_running(self) -> bool:
        with self._lock:
            if self._scan_task is None:
                return False
            try:
                done = self._scan_task.is_task_done()
            except Exception:
                self.log.exception('Error while checking NI scan task state.')
                done = True
            if done:
                # finite (REPETITIONS) scan finished or task failed
                self._close_scan_task()
                if self.module_state() == 'locked':
                    self.module_state.unlock()
                return False
            return True

    def get_scan_position(self) -> float:
        """ Current output voltage from the AI readback, estimated if no sense channel is set """
        with self._lock:
            return self._actual_output()

    def get_scan_direction(self) -> LaserScanDirection:
        """ Current scan direction. The waveform reverses on its own at the scan bounds, so the
        logic needs to poll this to follow the hardware.
        """
        with self._lock:
            if self._scan_waveform is not None:
                self._current_direction = self._scan_waveform.direction()
            return self._current_direction

    def set_scan_direction(self, direction: LaserScanDirection) -> None:
        with self._lock:
            direction = _to_enum(LaserScanDirection, direction)
            if direction == LaserScanDirection.UNDEFINED:
                return
            self._current_direction = direction
            if self._scan_task is None:
                self._settings = ScannableLaserSettings(bounds=self._settings.bounds,
                                                        speed=self._settings.speed,
                                                        mode=self._settings.mode,
                                                        repetitions=self._settings.repetitions,
                                                        initial_direction=direction)
                return
            # Waveform may already run in the requested direction (self-reversed at the bounds)
            if self._scan_waveform is not None and self._scan_waveform.direction() == direction:
                return
            estimated = self._estimated_output()
            self._close_scan_task()  # output is frozen now, _last_output holds the readback
            start = self._last_output
            self.log.debug(f'Direction change to {direction.name}: estimated output '
                           f'{estimated:.4f} V, readback {start:.4f} V')
            self._start_scan_waveform(direction, start)

    # ---------- helpers ----------
    def _clip(self, value: float) -> float:
        lo, hi = self._value_bounds
        return float(min(max(value, lo), hi))

    def _check_sample_count(self, n: int) -> None:
        if n > self._max_samples:
            raise ValueError(f'AO waveform needs {n:d} samples, more than max_samples '
                             f'({self._max_samples:d}). Increase speed or reduce sample rate.')
        if n > self._max_samples / 10:
            self.log.warning(f'AO waveform getting pretty big ({n:d} samples)')

    def _estimated_output(self) -> float:
        """ Commanded AO value estimated from the running waveform, else the last known value """
        for waveform in (self._scan_waveform, self._move_waveform):
            if waveform is not None:
                return waveform.value()
        if self._last_output is None:
            self._last_output = 0.5 * sum(self._value_bounds)
        return float(self._last_output)

    def _actual_output(self) -> float:
        """ Actual AO value from the AI readback (see NiAo.get_process_value), else estimated """
        position = self._read_ai()
        return self._estimated_output() if position is None else position

    def _scan_cycle(self, start: float, direction: LaserScanDirection
                    ) -> Tuple[np.ndarray, np.ndarray]:
        """ One full back-and-forth triangle cycle within the scan bounds, beginning at start and
        moving in direction first (see NiAo._initialize_continous_scan_data).
        """
        lo, hi = self._settings.bounds
        n_one = max(2, int(round((hi - lo) / self._settings.speed * self._scan_sample_rate)))
        rising = np.linspace(lo, hi, n_one)
        # The cycle begins exactly at start and continues with the first grid point beyond it, so
        # no step is larger than one grid step (no jump on start or direction change).
        if direction == LaserScanDirection.DOWN:
            falling = rising[::-1]
            idx = int(np.searchsorted(-falling, -start, side='right'))
            values = np.concatenate(([start], falling[idx:], rising, falling[:idx]))
            directions = np.concatenate((-np.ones(1 + n_one - idx), np.ones(n_one), -np.ones(idx)))
        else:
            idx = int(np.searchsorted(rising, start, side='right'))
            values = np.concatenate(([start], rising[idx:], rising[::-1], rising[:idx]))
            directions = np.concatenate((np.ones(1 + n_one - idx), -np.ones(n_one), np.ones(idx)))
        return values, directions

    def _start_scan_waveform(self, direction: LaserScanDirection, start: float) -> None:
        lo, hi = self._settings.bounds
        start = float(min(max(start, lo), hi))
        values, directions = self._scan_cycle(start, direction)
        continuous = self._settings.mode == LaserScanMode.CONTINUOUS
        if not continuous:
            values = np.tile(values, self._settings.repetitions)
            directions = np.tile(directions, self._settings.repetitions)
        self._check_sample_count(values.size)
        self.log.debug(f'Starting {"continuous" if continuous else "finite"} scan in '
                       f'{self._settings.bounds} V at {self._settings.speed} V/s from {start:.4f} V '
                       f'({values.size:d} samples at {self._scan_sample_rate} Hz)')
        self._scan_task = self._start_waveform_task(values, self._scan_sample_rate, continuous)
        self._scan_waveform = _Waveform(values, directions, self._scan_sample_rate, continuous)

    def _ramp(self, target: float, blocking: bool) -> None:
        """ Linear ramp from current output to target at the configured scan speed """
        self._close_move_task()
        target = self._clip(target)
        start = self._actual_output()
        n = int(round(abs(target - start) / self._settings.speed * self._move_sample_rate))
        if n < 2:
            self._write_static(target)
            return
        self._check_sample_count(n)
        values = np.linspace(start, target, n)
        directions = np.full(n, 1.0 if target > start else -1.0)
        self._move_task = self._start_waveform_task(values, self._move_sample_rate, False)
        self._move_waveform = _Waveform(values, directions, self._move_sample_rate, False)
        if blocking:
            self._move_task.wait_until_done(timeout=n / self._move_sample_rate + 10)
            self._close_move_task()

    # ---------- NI-DAQ ----------
    def _add_ao_channels(self, task) -> None:
        lo, hi = self._value_bounds
        task.ao_channels.add_ao_voltage_chan(f'/{self._device_name}/{self._ao_channels}',
                                             min_val=lo,
                                             max_val=hi)

    def _write_static(self, value: float) -> None:
        """ On-demand write of a single value to all AO channels (see NiAo.set_control_value) """
        value = self._clip(value)
        with ni.Task() as task:
            self._add_ao_channels(task)
            n_ch = task.number_of_channels
            data = value if n_ch == 1 else (np.ones((n_ch, 1)) * value)
            task.write(data, auto_start=True)
        self._last_output = value

    def _start_waveform_task(self, values: np.ndarray, sample_rate: float, continuous: bool):
        task = ni.Task()
        try:
            self._add_ao_channels(task)
            task.timing.cfg_samp_clk_timing(
                sample_rate,
                sample_mode=AcquisitionType.CONTINUOUS if continuous else AcquisitionType.FINITE,
                samps_per_chan=values.size
            )
            writer = AnalogMultiChannelWriter(task.out_stream)
            writer.write_many_sample(np.tile(values, (task.number_of_channels, 1)))
            task.start()
        except Exception:
            try:
                task.close()
            except Exception:
                pass
            raise
        return task

    def _close_scan_task(self) -> None:
        if self._scan_task is not None:
            estimated = self._scan_waveform.value() if self._scan_waveform is not None else None
            self._close_task(self._scan_task)
            self._update_last_output(estimated)
        self._scan_task = None
        self._scan_waveform = None

    def _close_move_task(self) -> None:
        if self._move_task is not None:
            estimated = self._move_waveform.value() if self._move_waveform is not None else None
            self._close_task(self._move_task)
            self._update_last_output(estimated)
        self._move_task = None
        self._move_waveform = None

    def _update_last_output(self, estimated: Optional[float]) -> None:
        """ After stopping a task the AO holds its value: read it back, else use the estimate """
        position = self._read_ai()
        if position is not None:
            self._last_output = position
        elif estimated is not None:
            self._last_output = estimated

    def _close_task(self, task) -> None:
        try:
            if not task.is_task_done():
                task.stop()
            task.close()
        except Exception:
            self.log.exception('Error while terminating NI analog output task.')

    def _create_ai_task(self) -> None:
        if self._ai_task is not None:
            return
        lo, hi = self._value_bounds
        task = ni.Task()
        try:
            task.ai_channels.add_ai_voltage_chan(f'/{self._device_name}/{self._sense_channel}',
                                                 min_val=lo,
                                                 max_val=hi,
                                                 terminal_config=TerminalConfiguration.RSE)
        except Exception:
            task.close()
            raise
        self._ai_task = task

    def _terminate_ai_task(self) -> None:
        if self._ai_task is not None:
            self._close_task(self._ai_task)
        self._ai_task = None

    def _read_ai(self) -> Optional[float]:
        """ Mean of readback samples, clipped to bounds (see NiAo.get_process_value) """
        if self._ai_task is None:
            return None
        try:
            with self._lock:
                data = self._ai_task.read(number_of_samples_per_channel=int(self._readback_samples))
            return self._clip(float(np.mean(data)))
        except Exception:
            self.log.exception('Error while reading AO readback via AI.')
            return None
