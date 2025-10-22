# -*- coding: utf-8 -*-
"""
Sirah Matisse Commander client implementing ScannableLaserInterface (Qudi Core).
"""

from __future__ import annotations

import socket
import struct
import re
import time

from typing import Optional, Tuple

from qtpy import QtCore

from qudi.core.configoption import ConfigOption
from qudi.util.constraints import ScalarConstraint
from qudi.interface.scannable_laser_interface import (
    ScannableLaserInterface,
    ScannableLaserConstraints,
    ScannableLaserSettings,
    LaserScanMode,
    LaserScanDirection,
)

# --- enum coercion helpers ---------------------------------------------------

def _coerce_scan_mode(val) -> LaserScanMode:
    if isinstance(val, LaserScanMode):
        return val
    if isinstance(val, str):
        # allow 'continuous', 'CONTINUOUS', 'repetitions', etc.
        key = val.strip().upper()
        return LaserScanMode[key]
    # allow integers 0/1
    return LaserScanMode(val)

def _coerce_scan_direction(val) -> LaserScanDirection:
    if isinstance(val, LaserScanDirection):
        return val
    if isinstance(val, str):
        key = val.strip().upper()
        return LaserScanDirection[key]
    return LaserScanDirection(val)


class SirahMatisseCommanderLaser(ScannableLaserInterface):
    # Connection
    _address: str = ConfigOption('address', default='127.0.0.1', missing='nothing')
    _port: int = ConfigOption('port', default=5900, missing='nothing')
    _timeout_s: float = ConfigOption('timeout_s', default=1.0, missing='warn')

    # Bounds and defaults (GUI constraints)
    _value_bounds: Tuple[float, float] = ConfigOption('position_bounds', default=(0.0, 0.65), missing='warn')
    _speed_bounds: Tuple[float, float] = ConfigOption('speed_bounds', default=(0.0, 0.006), missing='warn')
    _value_default: float = ConfigOption('position_default', default=0.3, missing='nothing')
    _speed_default: float = ConfigOption('speed_default', default=0.001, missing='nothing')

    # Allow string in config: mode_default: CONTINUOUS or REPETITIONS
    _mode_default: LaserScanMode = ConfigOption(
        'mode_default',
        default=LaserScanMode.CONTINUOUS,
        missing='nothing',
        constructor=_coerce_scan_mode
    )

    # Internal constants
    __handshake_query = 'Connection Valid?'
    __handshake_reply = 'Server alive'
    __close_cmd = 'Close_Network_Connection'
    __close_delay_s = 0.3

    # Poll/supervision interval
    __supervisor_interval_ms = 200

    sigPositionChanged = QtCore.Signal(float)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._sock: Optional[socket.socket] = None
        self._constraints: Optional[ScannableLaserConstraints] = None
        self._settings: Optional[ScannableLaserSettings] = None
        self._current_direction: LaserScanDirection = LaserScanDirection.UP
        self._remaining_sweeps: Optional[int] = None
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(False)
        self._timer.setInterval(self.__supervisor_interval_ms)
        self._timer.timeout.connect(self._poll_and_drive, QtCore.Qt.QueuedConnection)

    def on_activate(self):
        # constraints
        value = ScalarConstraint(default=self._value_default,
                                 bounds=tuple(self._value_bounds),
                                 increment=1e-4)
        speed = ScalarConstraint(default=self._speed_default,
                                 bounds=tuple(self._speed_bounds),
                                 increment=1e-4)
        reps = ScalarConstraint(default=1, bounds=(1, 10000), increment=1, enforce_int=True)
        self._constraints = ScannableLaserConstraints(
            value=value,
            unit='V',
            speed=speed,
            repetitions=reps,
            initial_directions=(LaserScanDirection.UP, LaserScanDirection.DOWN),
            modes=(LaserScanMode.CONTINUOUS, LaserScanMode.REPETITIONS),
        )

        # connect
        host = (self._address or '').strip()
        port = int(self._port)
        # allow "host:port" in address if user put it there
        if ':' in host and host.count(':') == 1 and host.rfind(']') == -1:
            maybe_host, maybe_port = host.split(':')
            if maybe_host and maybe_port.isdigit():
                host, port = maybe_host.strip(), int(maybe_port)
        try:
            self._sock = socket.create_connection((host, port), timeout=float(self._timeout_s))
        except socket.gaierror as e:
            raise ConnectionError(
                f'Host "{host}" could not be resolved. '
                f'Please set options address: "<Commander-IP-or-hostname>" and port: <Commander-port>.'
            ) from e
        except Exception as e:
            raise ConnectionError(
                f'Could not connect to Matisse Commander at {host}:{port} ({type(e).__name__}: {e}).'
            ) from e

        # handshake
        try:
            self._send(self.__handshake_query)
            reply = self._recv()
        except ConnectionResetError as e:
            try:
                self._sock.close()
            finally:
                self._sock = None
            raise ConnectionError(
                f'Connected to {host}:{port} but the remote closed the connection during handshake. '
                f'Is this the Sirah Commander server port?'
            ) from e

        if reply != self.__handshake_reply:
            try:
                self._sock.close()
            finally:
                self._sock = None
            raise ConnectionError(
                f'Unexpected handshake reply "{reply}". Are you sure {host}:{port} is the Matisse Commander server?'
            )

        # default scan settings (coerce enum)
        mode = _coerce_scan_mode(self._mode_default)
        self._settings = ScannableLaserSettings(
            bounds=tuple(self._value_bounds),
            speed=self._speed_default,
            mode=mode,
            repetitions=0,
            initial_direction=LaserScanDirection.UP,
        )

    def on_deactivate(self):
        try:
            self._timer.stop()
            if self.module_state() == 'locked':
                self._stop_scan_impl()
        finally:
            if self._sock is not None:
                try:
                    self._send(self.__close_cmd)
                    time.sleep(self.__close_delay_s)
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

        mode_enum = _coerce_scan_mode(mode)
        if mode_enum == LaserScanMode.REPETITIONS:
            if repetitions is None or int(repetitions) < 1:
                raise ValueError('repetitions must be >= 1 for REPETITIONS mode')
            self._constraints.repetitions.check(int(repetitions))

        if initial_direction == LaserScanDirection.UNDEFINED:
            initial_dir = LaserScanDirection.UP
        else:
            initial_dir = _coerce_scan_direction(initial_direction)

        # write to device
        self._set('SCAN:LLM', lo)
        self._set('SCAN:ULM', hi)
        self._set('SCAN:RSPD', float(speed))

        # store settings
        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=float(speed),
                                                mode=mode_enum,
                                                repetitions=int(repetitions or 0),
                                                initial_direction=initial_dir)

    def start_scan(self) -> None:
        if self.module_state() == 'locked':
            return
        settings = self._settings
        self._current_direction = settings.initial_direction
        self._remaining_sweeps = None if settings.mode == LaserScanMode.CONTINUOUS else int(settings.repetitions)
        self._set('SCAN:MODE', self._sirah_mode_for(self._current_direction))
        self._set('SCAN:STA', 'RU')
        self.module_state.lock()
        self._timer.start()

    def stop_scan(self) -> None:
        try:
            self._stop_scan_impl()
        finally:
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def _stop_scan_impl(self) -> None:
        self._timer.stop()
        try:
            self._set('SCAN:STA', 'ST')
        except Exception:
            pass
        self._remaining_sweeps = None

    def scan_to(self, value: float, blocking: Optional[bool] = False) -> None:
        if self.module_state() != 'idle':
            raise RuntimeError('Cannot move while scan is running. Stop the scan first.')
        v = float(value)
        v = max(self._value_bounds[0], min(v, self._value_bounds[1]))
        try:
            self._set('SCAN:STA', 'ST')
        except Exception:
            pass
        self._set('SCAN:POS', v)
        self.sigPositionChanged.emit(v)
        if blocking:
            try:
                t0 = time.time()
                while time.time() - t0 < 5.0:
                    pos = self._get_float('SCAN:POS?')
                    if abs(pos - v) < 1e-4:
                        break
                    time.sleep(0.05)
            except Exception:
                pass

    @QtCore.Slot()
    def _poll_and_drive(self) -> None:
        try:
            status = self._get_status()
            if status == 1:
                return  # still RUN
            if self._remaining_sweeps is not None:
                if self._remaining_sweeps <= 1:
                    self._timer.stop()
                    self.module_state.unlock()
                    return
                else:
                    self._remaining_sweeps -= 1
            self._current_direction = (
                LaserScanDirection.DOWN if self._current_direction == LaserScanDirection.UP
                else LaserScanDirection.UP
            )
            self._set('SCAN:MODE', self._sirah_mode_for(self._current_direction))
            self._set('SCAN:STA', 'RU')
        except Exception:
            self._timer.stop()
            try:
                self._set('SCAN:STA', 'ST')
            except Exception:
                pass
            if self.module_state() == 'locked':
                self.module_state.unlock()
            self.log.exception('Laser scan supervision failed.')

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
        hdr = self._sock.recv(4)
        if len(hdr) != 4:
            raise RuntimeError('Invalid header from server')
        n = struct.unpack('>L', hdr)[0]
        data = self._sock.recv(n)
        if len(data) != n:
            raise RuntimeError('Invalid payload from server')
        return data.decode('ascii')

    def _query(self, cmd: str):
        self._send(cmd)
        return self._parse_response(self._recv())

    def _set(self, base_cmd: str, value) -> None:
        if isinstance(value, float):
            s = f'{value:.6f}'
        elif isinstance(value, int):
            s = f'{value:d}'
        else:
            s = str(value)
        self._send(f'{base_cmd} {s}')
        self._parse_response(self._recv())

    def _get_status(self) -> int:
        _, val = self._query('SCAN:STA?')
        if isinstance(val, str):
            return 1 if val.strip().upper().startswith('RUN') else 0
        return int(val)

    def _get_float(self, cmd: str) -> float:
        _, val = self._query(cmd)
        return float(val)

    @staticmethod
    def _sirah_mode_for(direction: LaserScanDirection) -> int:
        # 6: increase voltage, stop at either limit; 7: decrease voltage, stop at either limit
        return 6 if direction == LaserScanDirection.UP else 7

    @staticmethod
    def _parse_response(content: str):
        """
        Parse Sirah Commander responses.

        Accepts:
        - ':<key>: <value>' (floats, ints, booleans, RUN/STOP, quoted strings)
        - 'OK'                                    -> ('ACK', True)
        - ':<key>: OK'                            -> (key, True)
        - '!ERROR <code> "...message..."'         -> raises ValueError
        """
        content = (content or '').strip()

        # Plain OK (some set-commands reply just 'OK')
        if re.fullmatch(r'OK', content, flags=re.IGNORECASE):
            return 'ACK', True

        # Standard ':key: value' format
        m = re.match(r'^\s*:([^\s]+):\s*(.+)\s*$', content)
        if m is not None:
            key, value = m.groups()

            # Value may be an unquoted token OK
            if re.fullmatch(r'OK', value, flags=re.IGNORECASE):
                return key, True

            # Quoted string
            sm = re.match(r'^\s*"([^"]+)"\s*$', value)
            if sm:
                return key, sm.group(1)

            # Float
            fm = re.match(r'^\s*([+\-]?[0-9]*\.[0-9]*(?:[eE][+\-]?[0-9]+)?)\s*$', value)
            if fm:
                return key, float(fm.group(1))

            # Int
            im = re.match(r'^\s*([+\-]?[0-9]+)\s*$', value)
            if im:
                return key, int(im.group(1))

            # RUN/STOP
            rm = re.match(r'^\s*(RUN|STOP)\s*$', value, re.IGNORECASE)
            if rm:
                return key, 1 if rm.group(1).upper() == 'RUN' else 0

            # TRUE/FALSE
            bm = re.match(r'^\s*(TRUE|FALSE)\s*$', value, re.IGNORECASE)
            if bm:
                return key, bm.group(1).upper() == 'TRUE'

            # !ERROR ... "message"
            em = re.match(r'^\s*!ERROR\s+(\d+).+?"(.*?)"', value)
            if em:
                code, msg = em.groups()
                raise ValueError(f'Server error {code}: {msg}')

            # Fallback: return raw value
            return key, value

        # Top-level !ERROR
        em2 = re.match(r'^\s*!ERROR\s+(\d+).+?"(.*?)"', content)
        if em2:
            code, msg = em2.groups()
            raise ValueError(f'Server error {code}: {msg}')

        raise ValueError(f'Invalid response: {content!r}')