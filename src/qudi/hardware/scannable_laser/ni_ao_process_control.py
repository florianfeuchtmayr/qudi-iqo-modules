# -*- coding: utf-8 -*-

"""
This file contains the qudi hardware module to use a National Instruments X-series card as mixed
signal input data streamer.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import time

from qudi.interface.process_control_interface import ProcessControlInterface
from qudi.core.module import Base
from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar

from qudi.util.mutex import Mutex
from qudi.util.helpers import in_range

import nidaqmx as ni
from nidaqmx.constants import TerminalConfiguration, AcquisitionType, WAIT_INFINITELY
from nidaqmx.stream_writers import AnalogMultiChannelWriter

import numpy as np
from PySide2 import QtCore


class NiAo(ProcessControlInterface):
    """ Nuk created this file to control the Velocity laser. Scanning AO things and tracking on an AI channel
    with the Ni USB-6002 Hardware.
    Parts copied from new qudi core, but also non interface things happening here ...
    TODO: DO NOT PUSH THIS ANYWHERE!!
    """
    _device_name = ConfigOption(name='device_name',
                                default='Dev1',
                                missing='error')

    _sense_channel = ConfigOption(name='sense_channel', default='ai0')
    _limits = ConfigOption(name='limits', default=(-0.5, 0.5), missing='warn')  # , constructor=lambda _, x: tuple(x))??
    _limit_epsilon = ConfigOption(name='limit_epsilon', default=0.01, missing='warn')

    _setpoint = StatusVar(name='current_setpoint', default=737.0)

    _default_scan_velocity = ConfigOption(name='default_scan_velocity', default=1)

    sigScanStateChanged = QtCore.Signal(bool, str)  # running, direction: 'up'|'down'|'none'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = Mutex()

        self._ao_task_handle = None
        self._ai_task_handle = None

        self._scan_ao_task_handle = None
        self._writer = None
        self._constraints = type('PVConstraints', (), {})()
        self._active: bool = False

        self._current_direction = None  # 'up' or 'down'
        self._last_pv = None
        self._scan_watchdog = None
        self._last_scan_range = None
        self._last_scan_velocity = None

    def on_activate(self):
        """ Starts up the NI-card and performs sanity checks """
        self._limits = tuple(sorted(self._limits))
        assert len(self._limits) == 2, 'Incorrect limits specified'
        self._create_ao_task()
        self._create_ai_task()

        # Sanitize status variables
        self._sanitize_setpoint_status()
        self.set_control_value(self._setpoint)

        self._scan_watchdog = QtCore.QTimer()
        self._scan_watchdog.setSingleShot(False)
        self._scan_watchdog.setInterval(200)  # ms
        self._scan_watchdog.timeout.connect(self._update_scan_state)

    def on_deactivate(self):
        self.stop_scan()
        self._terminate_ao_task()
        self._terminate_ai_task()
        self._active = False

    def is_scanning(self) -> bool:
        """Return True if a scan task is running."""
        return (self._scan_ao_task_handle is not None) and (not self._scan_ao_task_handle.is_task_done())

    def _update_scan_state(self):
        """Poll readback to infer scan direction and emit updates."""
        running = self.is_scanning()
        if not running:
            if self._current_direction is not None:
                self._current_direction = None
                self._last_pv = None
                self.sigScanStateChanged.emit(False, 'none')
            return

        try:
            pv = self.get_process_value()
        except Exception:
            return

        # Infer direction from delta and clamp at limits
        dirn = self._current_direction or 'up'
        if self._last_pv is not None:
            if pv > self._last_pv + 1e-9:
                dirn = 'up'
            elif pv < self._last_pv - 1e-9:
                dirn = 'down'

        lo, hi = self._limits
        eps = float(self._limit_epsilon)
        if pv >= hi - eps:
            dirn = 'down'
        elif pv <= lo + eps:
            dirn = 'up'

        if dirn != self._current_direction:
            self._current_direction = dirn
            self.sigScanStateChanged.emit(True, dirn)

        self._last_pv = pv

    def change_scan_direction(self, direction: str):
        """Change scan direction while scanning by restarting continuous scan with same params."""
        direction = 'down' if str(direction).lower().startswith('d') else 'up'
        self._current_direction = direction
        # Use last known parameters or fall back to limits/default velocity
        rng = self._last_scan_range if self._last_scan_range else self._limits
        vel = self._last_scan_velocity if self._last_scan_velocity else self._default_scan_velocity
        # restart scan
        try:
            self.start_scan(scan_velocity=vel, scan_range=rng, scan_direction=direction)
        except Exception:
            pass

    def set_control_value(self, value, channel=None):
        """ Set the value of the controlled process variable

        @param (float) value: The value to set
        @param (int) channel: (Optional) The number of the channel

        """

        with self._thread_lock:
            in_range_bool, in_range_val = in_range(value, *self._limits)
            min_val, max_val = self._limits
            if (value > max_val) | (value < min_val):
                self.log.warning(f'Value {value} is out of range for '
                                 f'limits {self._limits} and will be clipped to {in_range_val}')
            self._ao_task_handle.write(np.ones((2, 1)) * in_range_val, auto_start=True)
            self._setpoint = in_range_val

    def get_control_value(self, channel=None):
        """ Get the value of the controlled process variable

        @param (int) channel: (Optional) The number of the channel

        @return (float): The current control value
        """
        return self._setpoint

    def get_control_unit(self, channel=None):
        """ Return the unit that the value is set in as a tuple of ('abbreviation', 'full unit name')

        @param (int) channel: (Optional) The number of the channel

        @return: The unit as a tuple of ('abbreviation', 'full unit name')
        """
        return ('V', 'Voltage')

    def get_control_limit(self, channel=None):
        """ Return limits within which the controlled value can be set as a tuple of (low limit, high limit)

        @param (int) channel: (Optional) The number of the channel

        @return (tuple): The limits as (low limit, high limit)
        """
        return self._limits

    def _create_ao_task(self) -> None:
        if self._ao_task_handle:
            raise ValueError(f'AO task already present.')
        try:
            ao_task = ni.Task('USB AO')
        except ni.DaqError as err:
            raise RuntimeError(f'Unable to create NI task USB AO') from err
        try:
            min_val, max_val = self._limits
            ao_task.ao_channels.add_ao_voltage_chan(physical_channel=f'/{self._device_name}/ao0:1',  # ao_phys_ch, #TODO Careful hardcoded
                                                    min_val=min_val,
                                                    max_val=max_val)
        except Exception as err:
            try:
                ao_task.close()
            except ni.DaqError:
                pass
            raise RuntimeError('Error while configuring NI analog out task') from err
        self._ao_task_handle = ao_task

    def _terminate_ao_task(self):
        if self._ao_task_handle is not None:
            try:
                if not self._ao_task_handle.is_task_done():
                    self._ao_task_handle.stop()
                self._ao_task_handle.close()
            except ni.DaqError:
                self.log.exception('Error while trying to terminate analog input task.')
            self._ao_task_handle = None

    def _sanitize_setpoint_status(self) -> None:
        # TODO: Should we use the AI here? Does this make sense?
        min_val, max_val = self._limits
        if self._setpoint is None:
            self._setpoint = max_val / 2 + min_val / 2
            self.log.info(f'Setpoint defaulted to {self._setpoint}, since StatusVar was defaulted')

        if not (min_val < self._setpoint) & (max_val > self._setpoint):
            self._setpoint = max_val / 2 + min_val / 2
            self.log.info(f'Setpoint defaulted to {self._setpoint}, since StatusVar value out of limits {self._limits}')

    #  -----------------------------------------------------------------------------------------------------------------
    # ProcessInterface methods to read voltage
    #  -----------------------------------------------------------------------------------------------------------------

    def get_process_value(self, channel=None):
        """ Return a measured value

        @param (int) channel: (Optional) The number of the channel
        @return (float): The measured process value
        """
        return np.clip(np.mean(self._ai_task_handle.read(100)), *self._limits)
        # get mean of 100 points of auxiliary AO voltage through AI, and clip to limits (due to fluct. )

    def get_process_unit(self, channel=None):
        """ Return the unit that the value is set in as a tuple of ('abbreviation', 'full unit name')

        @param (int) channel: (Optional) The number of the channel

        @return: The unit as a tuple of ('abbreviation', 'full unit name')
        """
        return ('V', 'Voltage')

    def get_setpoint(self, channel=None):
        """ Return the current setpoint"""
        return self._setpoint

    def set_setpoint(self, setpoint):
        self._setpoint = setpoint

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

    def _create_ai_task(self):
        if self._ai_task_handle:
            raise ValueError(f'AI task already present.')
        try:
            ai_task = ni.Task('USB AI')
        except ni.DaqError as err:
            raise RuntimeError(f'Unable to create NI task USB AI') from err
        try:
            ai_phys_ch = f'/{self._device_name}/{self._sense_channel}'
            min_val, max_val = self._limits
            ai_task.ai_channels.add_ai_voltage_chan(physical_channel=ai_phys_ch,
                                                    min_val=min_val,
                                                    max_val=max_val,
                                                    terminal_config=TerminalConfiguration.RSE)  # for Single ended
        except Exception as err:
            try:
                ai_task.close()
            except ni.DaqError:
                pass
            raise RuntimeError('Error while configuring NI analog out task') from err
        self._ai_task_handle = ai_task

    def _terminate_ai_task(self):
        if self._ai_task_handle is not None:
            try:
                if not self._ai_task_handle.is_task_done():
                    self._ai_task_handle.stop()
                self._ai_task_handle.close()
            except ni.DaqError:
                self.log.exception('Error while trying to terminate analog input task.')
            self._ai_task_handle = None

    #  -----------------------------------------------------------------------------------------------------------------
    #  ------ Scanning related things without interface ----------------------------------------------------------------
    #  -----------------------------------------------------------------------------------------------------------------

    def start_scan(self, scan_velocity=None, scan_range=None, scan_direction=None):

        """
        Starts a continuous back and forth scan from min(scan_range) to max(scan_range). This function is non-blocking,
        yet can be called consecutively while scan is running.
        """
        # TODO: HW limits of velocity laser 700Hz triangle waveform on +- 3V; 2kHz on +-0.3V so insanely fast.
        #  is that seriously 8000 V/s for 700 Hz??? (with +- 3V output)

        self._last_scan_velocity = float(scan_velocity) if scan_velocity is not None else float(
            self._default_scan_velocity)
        self._last_scan_range = tuple(scan_range) if scan_range is not None else tuple(self._limits)
        if isinstance(scan_direction, str):
            self._current_direction = 'down' if scan_direction.lower().startswith('d') else 'up'
        else:
            self._current_direction = None

        if scan_range is not None:
            assert len(scan_range) == 2, f'No proper scan range provided'
            assert (min(scan_range) >= min(self._limits)) & (max(scan_range) <= max(self._limits)), \
                f'Scan range {scan_range} out of bounds {self._limits}'
            # assert (self.get_process_value() > min(scan_range)) & (self.get_process_value() < max(scan_range)),\
            # f"Current position is out of the scan range. Please consider another range, including current location"

        min_val, max_val = self._limits if scan_range is None else scan_range

        sample_rate = 30  # TODO keep hardcoded?
        if scan_velocity is not None:
            number_of_one_way_samples = int((max_val - min_val) / scan_velocity * sample_rate)
        else:
            number_of_one_way_samples = int((max_val - min_val) / self._default_scan_velocity * sample_rate)

        self.log.debug(f'Scanning with range{scan_range}, meaning a distance {(max_val - min_val)} with N:'
                       f' {number_of_one_way_samples}, which is a calculated '
                       f'velocity {(max_val - min_val) / number_of_one_way_samples * sample_rate}')

        # TODO limit values somehow to DAC range? So from 2^16 values on lower sample rate?
        if number_of_one_way_samples < 1e7:
            pass
        elif number_of_one_way_samples < 1e8:
            self.log.warning(f'Number of samples per scan direction getting pretty big ({number_of_one_way_samples})')
        else:
            raise ValueError(f'Number of samples per scan direction too big which will cause issues')

        if self._scan_ao_task_handle:
            self._terminate_scan_ao_task()  # To enable "back and forth scanning"
            # raise ValueError(f'AO task already present.')

        # # Scan to starting point of scan range, if current position is out of scan_range
        # if self.get_process_value() < min_val:  # TODO Does not work anymore, since wait till done does not work.
        #     self.scan_to(min_val, scan_velocity=scan_velocity)
        # elif self.get_process_value() > max_val:
        #     self.scan_to(max_val, scan_velocity=scan_velocity)
        # else:
        #     pass

        if self.get_process_value() < min_val or self.get_process_value() > max_val:
            raise ValueError('Scan limits too close to current value. Increase boundaries, please')
        else:
            pass

        try:
            scan_ao_task = ni.Task('USB Scan AO')
        except ni.DaqError as err:
            raise RuntimeError(f'Unable to create NI task USB Scan AO') from err
        try:
            scan_ao_task.ao_channels.add_ao_voltage_chan(physical_channel=f'/{self._device_name}/ao0:1',  # ao_phys_ch,
                                                         min_val=min_val,
                                                         max_val=max_val)

            scan_ao_task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS,
                                                    samps_per_chan=number_of_one_way_samples*2)  # back and forth -> *2

        except Exception as err:
            try:
                scan_ao_task.close()
            except ni.DaqError:
                pass
            raise RuntimeError('Error while configuring NI analog out task') from err

        scan_data = self._initialize_continous_scan_data(number_of_one_way_samples=number_of_one_way_samples,
                                                         scan_range=scan_range,
                                                         scan_direction=scan_direction)

        with self._thread_lock:
            self._writer = AnalogMultiChannelWriter(scan_ao_task.out_stream)
            self._writer.write_many_sample(scan_data)
            self._scan_ao_task_handle = scan_ao_task
            self._scan_ao_task_handle.start()

        self.sigScanStateChanged.emit(True, self._current_direction or 'up')
        if self._scan_watchdog is not None:
            self._scan_watchdog.start()

    def scan_to(self, value=None, scan_velocity=None):

        """
        Scans from current setpoint (control_value) to value, if value within limits. This function is blocking,
        und returns once setpoint is reached.
        """

        # TODO: Add the possibility to do it None blocking? But then what happens if scan end is reached and
        #  scan task not terminated?

        min_val, max_val = self._limits
        assert (value <= max_val) & (value >= min_val), f'Value {value} out of limits {self._limits}'

        if self._scan_ao_task_handle:
            self._terminate_scan_ao_task()  # Stop any currently running scan

        with self._thread_lock:
            current_ai_val = self.get_process_value()  # Has to be the sensed value, in case called while scanning
            dist = abs(value - current_ai_val)
            sample_rate = 5000  # TODO keep hardcoded?
            if scan_velocity is not None:
                number_of_samples = int(dist / scan_velocity * sample_rate)
            else:
                number_of_samples = int(dist / self._default_scan_velocity * sample_rate)

            assert number_of_samples >= 2, f'No samples needed to get to {value}'

            self.log.debug(f'Moving from {self._setpoint} to {value}, meaning a distance {dist} with N:'
                           f' {number_of_samples}, which is a calculated velocity {dist/number_of_samples*sample_rate}')

            ls = np.linspace(current_ai_val, value, number_of_samples)
            scan_data = np.vstack((ls, ls))

        # TODO limit values somehow to DAC range? So from 2^16 values on lower sample rate?
        if number_of_samples < 1e7:
            pass
        elif number_of_samples < 1e8:
            self.log.warning(f'Number of samples per scan direction getting pretty big ({number_of_samples})')
        else:
            raise ValueError(f'Number of samples per scan direction too big which will cause issues')

        try:
            scan_ao_task = ni.Task('USB Scan AO')
        except ni.DaqError as err:
            raise RuntimeError(f'Unable to create NI task USB Scan AO') from err
        try:
            scan_ao_task.ao_channels.add_ao_voltage_chan(physical_channel=f'/{self._device_name}/ao0:1',  # ao_phys_ch,
                                                         min_val=min_val,
                                                         max_val=max_val)

            scan_ao_task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.FINITE,
                                                    samps_per_chan=number_of_samples)

        except Exception as err:
            try:
                scan_ao_task.close()
            except ni.DaqError:
                pass
            raise RuntimeError('Error while configuring NI analog out task') from err

        with self._thread_lock:
            self._writer = AnalogMultiChannelWriter(scan_ao_task.out_stream)
            self._writer.write_many_sample(scan_data)
            self._scan_ao_task_handle = scan_ao_task
            self._scan_ao_task_handle.start()
            self.log.debug(f'Wait until done with {number_of_samples+100} samples for {(number_of_samples+100)/sample_rate} s')
            # self._scan_ao_task_handle.wait_until_done((number_of_samples+100)/sample_rate)
            # self._scan_ao_task_handle.wait_until_done(WAIT_INFINITELY)
            # self._terminate_scan_ao_task()
            self._setpoint = value

    def stop_scan(self):
        self._terminate_scan_ao_task()
        with self._thread_lock:
            self._setpoint = self.get_process_value()
        if self._scan_watchdog is not None:
            self._scan_watchdog.stop()
        self._current_direction = None
        self._last_pv = None
        self.sigScanStateChanged.emit(False, 'none')

    def _terminate_scan_ao_task(self):
        if self._scan_ao_task_handle is not None:
            try:
                if not self._scan_ao_task_handle.is_task_done():
                    self._scan_ao_task_handle.stop()
                self._scan_ao_task_handle.close()
            except ni.DaqError:
                self.log.exception('Error while trying to terminate analog input task.')
            self._scan_ao_task_handle = None

    def _initialize_continous_scan_data(self, number_of_one_way_samples, scan_range, scan_direction):

        with self._thread_lock:
            one_direction_array = np.linspace(*scan_range, number_of_one_way_samples)

            current_ao_from_ai_value = self.get_process_value()

            # self.log.debug(f'Difference of setpoint to ai_value {self._setpoint - current_ao_from_ai_value}')

            # get nearest index of current ao value. Do not use setpoint in case of consecutive scans
            nearest_idx = (np.abs(one_direction_array - current_ao_from_ai_value)).argmin()

            if scan_direction == 'up':
                scan_direction_encoding = 0
            elif scan_direction == 'down':
                scan_direction_encoding = 1
            else:
                scan_direction_encoding = np.abs(current_ao_from_ai_value - scan_range).argmin()
                # if 1 -> closer to upper bound, scan down
                # if 0 -> closer to lower bound, scan up

            if scan_direction_encoding:
                scan_array = np.concatenate((one_direction_array[:nearest_idx][::-1],
                                             one_direction_array,
                                             one_direction_array[nearest_idx:][::-1]))
            else:
                scan_array = np.concatenate((one_direction_array[nearest_idx:],
                                             one_direction_array[::-1],
                                             one_direction_array[:nearest_idx]))

            return np.vstack((scan_array, scan_array))
