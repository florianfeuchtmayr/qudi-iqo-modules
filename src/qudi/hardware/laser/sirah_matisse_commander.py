# -*- coding: utf-8 -*-
"""
Sirah Matisse Commander client implementing ScannableLaserInterface.

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

import socket
import struct
import time
from typing import Optional, Tuple

from qtpy import QtCore

from qudi.core.configoption import ConfigOption
from qudi.interface.scannable_laser_interface import (
    ScannableLaserInterface,
    ScannableLaserConstraints,
    ScannableLaserSettings,
    LaserScanMode,
    LaserScanDirection,
)
from qudi.util.constraints import ScalarConstraint


class SirahMatisseCommanderLaser(ScannableLaserInterface):
    """
    This is the Hardware class for the control of sirah matisse laser.

    Example config:

    sirah_matisse:
        module.Class: 'laser.sirah_matisse_commander.SirahMatisseCommanderLaser'
        options:
            address: '127.0.0.1' #IP in Matisse Commander: Matisse -> Communication Options -> Network Server Settings
            port: 5902
            timeout_s: 1.0
            position_bounds: [0.0, 0.65]
            speed_bounds: [0.0, 0.010]
            position_default: 0.3
            speed_default: 0.001
    """
    # Connection
    _address: str = ConfigOption('address', default='127.0.0.1', missing='nothing')
    _port: int = ConfigOption('port', default=5900, missing='nothing')
    _timeout_s: float = ConfigOption('timeout_s', default=1.0, missing='warn')

    # Bounds and defaults (constraints / GUI)
    _value_bounds: Tuple[float, float] = ConfigOption('position_bounds', default=(0.0, 0.65), missing='warn')
    _speed_bounds: Tuple[float, float] = ConfigOption('speed_bounds', default=(0.0, 0.006), missing='warn')
    _speed_default: float = ConfigOption('speed_default', default=0.001, missing='nothing')

    # Default mode (accepts enum, str or int via inline coercion)
    _mode_default: LaserScanMode = ConfigOption(
        'mode_default',
        default=LaserScanMode.CONTINUOUS,
        missing='nothing',
        constructor=lambda v: (
            LaserScanMode[v.strip().upper()] if isinstance(v, str)
            else (LaserScanMode(v) if isinstance(v, int) else v)
        )
    )

    # Poll/supervision interval
    __supervisor_interval_ms = 200

    sigPositionChanged = QtCore.Signal(float)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._sock: Optional[socket.socket] = None
        self._constraints: Optional[ScannableLaserConstraints] = None
        self._settings: Optional[ScannableLaserSettings] = None
        self._current_direction: LaserScanDirection = LaserScanDirection.UP
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(False)
        self._timer.setInterval(self.__supervisor_interval_ms)
        self._timer.timeout.connect(self._supervise, QtCore.Qt.QueuedConnection)

    def on_activate(self):
        # Set constraints
        lo, hi = tuple(self._value_bounds)
        center = 0.5 * (lo + hi)

        value_c = ScalarConstraint(default=center,
                                   bounds=(lo, hi),
                                   increment=1e-4)
        speed_c = ScalarConstraint(default=self._speed_default,
                                   bounds=tuple(self._speed_bounds),
                                   increment=1e-4)
        reps_c = ScalarConstraint(default=1, bounds=(1, 10000), increment=1, enforce_int=True)
        self._constraints = ScannableLaserConstraints(
            value=value_c,
            unit='V',
            speed=speed_c,
            repetitions=reps_c,
            initial_directions=(LaserScanDirection.UP, LaserScanDirection.DOWN),
            modes=(LaserScanMode.CONTINUOUS, LaserScanMode.REPETITIONS),
        )

        # Open socket (no handshake)
        host = (self._address or '').strip()
        port = int(self._port)
        # allow "host:port" in address
        if ':' in host and host.count(':') == 1 and host.rfind(']') == -1:
            maybe_host, maybe_port = host.split(':')
            if maybe_host and maybe_port.isdigit():
                host, port = maybe_host.strip(), int(maybe_port)
        self._sock = socket.create_connection((host, port), timeout=float(self._timeout_s))

        # Default scan settings
        mode = self._mode_default
        self._settings = ScannableLaserSettings(bounds=tuple(self._value_bounds),
                                                speed=self._speed_default,
                                                mode=mode,
                                                repetitions=0,
                                                initial_direction=LaserScanDirection.UP)

    def on_deactivate(self):
        try:
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            if self.module_state() == 'locked':
                self.stop_scan()
        finally:
            if self._sock is not None:
                try:
                    # Politely request remote to close (optional; ignore errors)
                    self._send('Close_Network_Connection')
                    time.sleep(0.1)
                except Exception:
                    pass
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None

    @property
    def constraints(self) -> ScannableLaserConstraints:
        return self._constraints

    @property
    def scan_settings(self) -> ScannableLaserSettings:
        return self._settings

    # -------------------- main API --------------------

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
        self._constraints.speed.check(float(speed))

        mode_enum = mode if isinstance(mode, LaserScanMode) else LaserScanMode(int(mode))
        reps = int(repetitions or 0)
        if mode_enum == LaserScanMode.REPETITIONS and reps < 1:
            raise ValueError('repetitions must be >= 1 for REPETITIONS mode')

        init_dir = LaserScanDirection.UP if initial_direction == LaserScanDirection.UNDEFINED \
            else (initial_direction if isinstance(initial_direction, LaserScanDirection)
                  else LaserScanDirection(int(initial_direction)))

        # Use helpers for device settings
        self.set_scan_bounds(lo, hi)
        self.set_scan_speeds(float(speed))

        # Store settings
        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=float(speed),
                                                mode=mode_enum,
                                                repetitions=reps,
                                                initial_direction=init_dir)

    def start_scan(self) -> None:
        if self.module_state() == 'locked':
            return
        # Use helper to set initial direction and RUN
        self.set_scan_direction(self._settings.initial_direction)
        self._set('SCAN:STA', 'RU')  # RUN
        self.module_state.lock()
        QtCore.QMetaObject.invokeMethod(self, '_timer_start', QtCore.Qt.QueuedConnection)

    def stop_scan(self) -> None:
        try:
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            self._set('SCAN:STA', 'ST')  # STOP
        finally:
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def scan_to(self, value: float, blocking: Optional[bool] = False) -> None:
        """
        Move to a position using the actuator (reference cell scan position).
        Only allowed if not actively scanning.
        """
        if self.module_state() != 'idle':
            raise RuntimeError('Cannot move while scan is running. Stop the scan first.')

        v = float(value)
        v = max(self._value_bounds[0], min(v, self._value_bounds[1]))

        # Use helper to set speeds once here (optional convenience)
        self.set_scan_speeds(self._settings.speed)

        # Move to position
        self._set('SCAN:NOW', v)
        self.sigPositionChanged.emit(v)

        if blocking:
            try:
                t0 = time.time()
                while time.time() - t0 < 5.0:
                    pos = float(self._query('SCAN:NOW?'))
                    if abs(pos - v) < 1e-4:
                        break
                    time.sleep(0.05)
            except Exception:
                pass

    # -------------------- public helpers --------------------

    def set_scan_speeds(self, rising_speed: float, falling_speed: Optional[float] = None) -> None:
        """Set rising and falling speeds. If falling_speed is None, use rising_speed for both."""
        rs = float(rising_speed)
        fs = float(falling_speed) if falling_speed is not None else rs
        self._set('SCAN:RSPD', rs)
        self._set('SCAN:FSPD', fs)

    def set_scan_bounds(self, lower: float, upper: float) -> None:
        """Set lower and upper bounds for the scan window."""
        lo = float(lower)
        hi = float(upper)
        if lo >= hi:
            raise ValueError('lower must be < upper')
        self._set('SCAN:LLM', lo)
        self._set('SCAN:ULM', hi)

    def set_scan_direction(self, direction: LaserScanDirection) -> None:
        """Set scan direction (UP->0, DOWN->1)."""
        if not isinstance(direction, LaserScanDirection):
            direction = LaserScanDirection(int(direction))
        self._set('SCAN:MODE', 0 if direction == LaserScanDirection.UP else 1)
        self._current_direction = direction

    # -------------------- supervision --------------------

    @QtCore.Slot()
    def _supervise(self) -> None:
        """
        Minimal supervision:
        - Read current position via SCAN:NOW?
        - If scanning UP and at/beyond upper bound -> SCAN:MODE DOWN
        - If scanning DOWN and at/beyond lower bound -> SCAN:MODE UP
        """
        try:
            status = self._query('SCAN:STA?')  # RUN/STOP token
            if isinstance(status, str):
                is_running = status.strip().upper().startswith('RUN')
            else:
                is_running = bool(status)
            if not is_running:
                return

            pos = float(self._query('SCAN:NOW?'))
            lo, hi = self._settings.bounds
            if self._current_direction == LaserScanDirection.UP:
                if pos >= hi:
                    self.set_scan_direction(LaserScanDirection.DOWN)
            else:
                if pos <= lo:
                    self.set_scan_direction(LaserScanDirection.UP)

        except Exception:
            # Any failure: stop supervision and unlock module
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            try:
                self._set('SCAN:STA', 'ST')
            except Exception:
                pass
            if self.module_state() == 'locked':
                self.module_state.unlock()
            self.log.exception('Laser scan supervision failed.')

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

    # -------------------- low-level I/O (minimal) -----------------

    def _send(self, data: str) -> None:
        if self._sock is None:
            raise RuntimeError('Not connected to Matisse Commander.')
        payload = data.encode('ascii')
        header = struct.pack('>L', len(payload))
        sent = self._sock.send(header + payload)
        if sent == 0:
            raise RuntimeError('No data sent to server.')

    def _recv(self) -> str:
        if self._sock is None:
            raise RuntimeError('Not connected to Matisse Commander.')
        hdr = b''
        while len(hdr) < 4:
            chunk = self._sock.recv(4 - len(hdr))
            if not chunk:
                raise TimeoutError('No header bytes from server')
            hdr += chunk
        n = struct.unpack('>L', hdr)[0]
        data = b''
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                raise TimeoutError('Connection closed while reading payload')
            data += chunk
        return data.decode('ascii')

    def _set(self, base_cmd: str, value) -> None:
        """
        Minimal setter: fire-and-forget. No parsing of reply needed.
        """
        s = f'{value:.6f}' if isinstance(value, float) else (f'{value:d}' if isinstance(value, int) else str(value))
        self._send(f'{base_cmd} {s}')
        try:
            # Read and discard reply
            _ = self._recv()
        except Exception:
            # Reply parsing is not required here; keep robust
            pass

    def _query(self, cmd: str):
        """
        Minimal query: returns a primitive (float/str/int) parsed from ':KEY: value' or 'OK'.
        """
        self._send(cmd)
        resp = self._recv()
        content = (resp or '').strip()

        # Plain OK
        if content.upper() == 'OK':
            return 'OK'

        # Expect ':KEY: value'
        if ':' in content:
            try:
                # take last token after ':'
                value = content.split(':', 2)[-1].strip()
            except Exception:
                return content
            # RUN/STOP
            if value.upper().startswith('RUN'):
                return 'RUN'
            if value.upper().startswith('STOP'):
                return 'STOP'
            # TRUE/FALSE
            if value.upper() in ('TRUE', 'FALSE'):
                return value.upper() == 'TRUE'
            # float or int
            try:
                if any(c in value for c in ('.', 'e', 'E')):
                    return float(value)
                return int(value)
            except Exception:
                return value
        return content