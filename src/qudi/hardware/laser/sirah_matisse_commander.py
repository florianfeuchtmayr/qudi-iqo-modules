# -*- coding: utf-8 -*-
"""
Sirah Matisse Commander client implementing ScannableLaserInterface (Qudi Core).
"""

from __future__ import annotations

import socket
import struct
import re
import time
import threading

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

    # Actuator channel used for position (your firmware does not support SCAN:POS)
    _position_channel: str = ConfigOption('position_channel', default='SPZT', missing='nothing')

    # Turnaround behavior (tunable via config)
    _turnaround_margin: float = ConfigOption('turnaround_margin', default=0.003, missing='nothing')        # V
    _reposition_offset: float = ConfigOption('reposition_offset', default=0.003, missing='nothing')        # V
    _turnaround_cooldown_ms: int = ConfigOption('turnaround_cooldown_ms', default=200, missing='nothing')  # ms

    # Post-RUN speed re-apply delay (ms)
    _speed_apply_post_run_delay_ms: int = ConfigOption('speed_apply_post_run_delay_ms', default=120, missing='nothing')

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

        self._io_lock = threading.Lock()
        self._inter_cmd_delay_s = 0.03

        # turnaround cooldown tracking
        self._last_flip_ts: float = 0.0

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
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
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

    # -------------------- helpers --------------------

    def _pos_cmd(self, suffix: str) -> str:
        # e.g. 'SPZT:NOW' or 'SPZT:NOW?'
        return f"{str(self._position_channel).strip().upper()}:{suffix}"

    def _apply_both_speeds(self, base_speed: float) -> None:
        """
        Apply both rising and falling speeds to Commander:
        SCAN:RSPD = base_speed, SCAN:FSPD = base_speed
        """
        try:
            self._set('SCAN:RSPD', float(base_speed))
        except Exception:
            pass
        try:
            self._set('SCAN:FSPD', float(base_speed))
        except Exception:
            pass

    def _reapply_both_speeds_later(self, base_speed: float) -> None:
        """
        Re-apply both speeds shortly after RUN to ensure Commander takes them.
        """
        try:
            QtCore.QTimer.singleShot(
                int(self._speed_apply_post_run_delay_ms),
                lambda: self._apply_both_speeds(float(base_speed))
            )
        except Exception:
            pass

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

        mode_enum = _coerce_scan_mode(mode)
        if mode_enum == LaserScanMode.REPETITIONS:
            if repetitions is None or int(repetitions) < 1:
                raise ValueError('repetitions must be >= 1 for REPETITIONS mode')
            self._constraints.repetitions.check(int(repetitions))

        if initial_direction == LaserScanDirection.UNDEFINED:
            initial_dir = LaserScanDirection.UP
        else:
            initial_dir = _coerce_scan_direction(initial_direction)

        # write limits immediately
        self._set('SCAN:LLM', lo)
        self._set('SCAN:ULM', hi)
        # also set both speeds now so Commander UI reflects them
        self._apply_both_speeds(float(speed))

        # store settings
        self._settings = ScannableLaserSettings(bounds=(lo, hi),
                                                speed=float(speed),
                                                mode=mode_enum,
                                                repetitions=int(repetitions or 0),
                                                initial_direction=initial_dir)

    def start_scan(self) -> None:
        """
        Start a scan without SCAN:MODE and without SCAN:POS.
        Place actuator just inside the chosen edge, set both RSPD/FSPD to GUI speed, RUN.
        """
        if self.module_state() == 'locked':
            return

        settings = self._settings
        self._current_direction = settings.initial_direction
        self._remaining_sweeps = None if settings.mode == LaserScanMode.CONTINUOUS else int(settings.repetitions)
        self._last_flip_ts = 0.0

        # Place inside the edge that matches the desired initial direction
        lo, hi = settings.bounds
        offs = float(self._reposition_offset)
        if self._current_direction == LaserScanDirection.UP:
            target = lo + max(offs, 1e-4)
        else:
            target = hi - max(offs, 1e-4)

        try:
            self._set(self._pos_cmd('NOW'), float(target))
        except Exception:
            pass
        time.sleep(0.01)

        # Apply both speeds to the same GUI speed, then RUN
        base_speed = float(settings.speed)
        self._apply_both_speeds(base_speed)
        self._set('SCAN:STA', 'RU')
        self._reapply_both_speeds_later(base_speed)

        self.module_state.lock()
        QtCore.QMetaObject.invokeMethod(self, '_timer_start', QtCore.Qt.QueuedConnection)

    def stop_scan(self) -> None:
        try:
            self._stop_scan_impl()
        finally:
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def _stop_scan_impl(self) -> None:
        QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
        try:
            self._set('SCAN:STA', 'ST')
        except Exception:
            pass
        self._remaining_sweeps = None

    def scan_to(self, value: float, blocking: Optional[bool] = False) -> None:
        """
        Move to a position using the actuator channel (e.g., SPZT:NOW).
        """
        if self.module_state() != 'idle':
            raise RuntimeError('Cannot move while scan is running. Stop the scan first.')
        v = float(value)
        v = max(self._value_bounds[0], min(v, self._value_bounds[1]))
        try:
            self._set('SCAN:STA', 'ST')
        except Exception:
            pass
        self._set(self._pos_cmd('NOW'), v)
        self.sigPositionChanged.emit(v)
        if blocking:
            try:
                t0 = time.time()
                while time.time() - t0 < 5.0:
                    _, pos = self._query(self._pos_cmd('NOW?'))
                    if abs(float(pos) - v) < 1e-4:
                        break
                    time.sleep(0.05)
            except Exception:
                pass

    @QtCore.Slot()
    def _poll_and_drive(self) -> None:
        """
        Supervision loop:
        - Read actuator position (position_channel:NOW?).
        - If near a bound (with turnaround_margin) and cooldown elapsed:
            STOP -> jump inside opposite edge -> set both speeds (RSPD/FSPD) -> RUN.
        - If STA? == 0 and repetitions remain: RUN again (set both speeds first).
        Notes:
        - Do NOT write SCAN:MODE or SCAN:POS on this firmware.
        - Position is controlled via the actuator (position_channel:NOW / NOW?).
        """
        try:
            status = self._get_status()  # 1 = RUN, 0 = STOP

            pos = None
            try:
                _, v = self._query(self._pos_cmd('NOW?'))
                pos = float(v)
            except Exception:
                pass

            # Window and margins
            lo = self._settings.bounds[0] if self._settings is not None else self._value_bounds[0]
            hi = self._settings.bounds[1] if self._settings is not None else self._value_bounds[1]
            margin = float(self._turnaround_margin)

            # Optional asymmetric offsets (fallback to reposition_offset)
            offs_lo = float(getattr(self, '_reposition_offset_lower', self._reposition_offset))
            offs_hi = float(getattr(self, '_reposition_offset_upper', self._reposition_offset))

            near_upper = (pos is not None) and (pos >= hi - margin)
            near_lower = (pos is not None) and (pos <= lo + margin)

            now = time.time()
            cooldown_ok = (now - self._last_flip_ts) * 1000.0 >= float(self._turnaround_cooldown_ms)

            leg_finished = (status == 0) or (cooldown_ok and (near_upper or near_lower))
            if not leg_finished:
                return

            # Repetitions handling
            if self._remaining_sweeps is not None:
                if self._remaining_sweeps <= 1:
                    QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
                    if self.module_state() == 'locked':
                        self.module_state.unlock()
                    return
                else:
                    self._remaining_sweeps -= 1

            base_speed = float(self._settings.speed)

            # If device stopped in mid-window without edge: just RUN again (set both speeds first)
            if status == 0 and not (near_upper or near_lower):
                self._apply_both_speeds(base_speed)
                self._set('SCAN:STA', 'RU')
                self._reapply_both_speeds_later(base_speed)
                self._last_flip_ts = now
                return

            # Flip direction by repositioning
            try:
                self._set('SCAN:STA', 'ST')
            except Exception:
                pass
            time.sleep(0.01)

            if near_upper:
                target = lo + max(offs_lo, 1e-4)
                new_dir = LaserScanDirection.DOWN
            elif near_lower:
                target = hi - max(offs_hi, 1e-4)
                new_dir = LaserScanDirection.UP
            else:
                target = hi - max(offs_hi, 1e-4)
                new_dir = LaserScanDirection.DOWN

            try:
                self._set(self._pos_cmd('NOW'), float(target))
            except Exception:
                pass
            time.sleep(0.01)

            # Apply both speeds (RSPD/FSPD) for the new leg, then RUN
            self._current_direction = new_dir
            self._apply_both_speeds(base_speed)
            self._set('SCAN:STA', 'RU')
            self._reapply_both_speeds_later(base_speed)

            self._last_flip_ts = now

        except Exception:
            QtCore.QMetaObject.invokeMethod(self, '_timer_stop', QtCore.Qt.QueuedConnection)
            try:
                self._set('SCAN:STA', 'ST')
            except Exception:
                pass
            if self.module_state() == 'locked':
                self.module_state.unlock()
            self.log.exception('Laser scan supervision failed.')

    # --------------- low-level I/O (serialized) -----------------

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

    def _query(self, cmd: str):
        with self._io_lock:
            self._send(cmd)
            try:
                resp = self._recv()
            finally:
                time.sleep(self._inter_cmd_delay_s)
        return self._parse_response(resp)

    def _set(self, base_cmd: str, value) -> None:
        if isinstance(value, float):
            s = f'{value:.6f}'
        elif isinstance(value, int):
            s = f'{value:d}'
        else:
            s = str(value)
        with self._io_lock:
            self._send(f'{base_cmd} {s}')
            try:
                resp = self._recv()
            finally:
                time.sleep(self._inter_cmd_delay_s)
        self._parse_response(resp)

    def _get_status(self) -> int:
        _, val = self._query('SCAN:STA?')
        if isinstance(val, str):
            return 1 if val.strip().upper().startswith('RUN') else 0
        return int(val)

    def _get_float(self, cmd: str) -> float:
        _, val = self._query(cmd)
        return float(val)

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

    def set_scan_speed(self, speed: float) -> None:
        """
        Update scan speed immediately on the device and store it in settings.
        Applies to both directions: SCAN:RSPD and SCAN:FSPD.
        Safe to call while running.
        """
        self._constraints.speed.check(float(speed))
        # Update stored settings
        self._settings = ScannableLaserSettings(bounds=self._settings.bounds,
                                                speed=float(speed),
                                                mode=self._settings.mode,
                                                repetitions=self._settings.repetitions,
                                                initial_direction=self._settings.initial_direction)
        # Apply both speeds now and once after RUN
        base_speed = float(speed)
        self._apply_both_speeds(base_speed)
        self._reapply_both_speeds_later(base_speed)