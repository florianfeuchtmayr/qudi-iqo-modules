# -*- coding: utf-8 -*-
"""
Hardware file for a vector magnet from cryomagentics.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
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
import threading
import re
import math
from typing import Dict, Optional, Any
from PySide2 import QtCore
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.interface.vector_magnet_interface import VectorMagnetHardwareInterface

try:
    import serial  # pyserial
except ImportError:
    serial = None


class _SerialPortWrapper:
    """Thin wrapper around pyserial for dependency isolation."""

    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial not installed. Run 'pip install pyserial'.")
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            rtscts=False,
            dsrdtr=False
        )

    def write_bytes(self, data: bytes):
        self._ser.write(data)

    def in_waiting(self) -> int:
        return self._ser.in_waiting

    def read(self, n: int) -> bytes:
        return self._ser.read(n)

    def reset_input_buffer(self):
        self._ser.reset_input_buffer()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


class VectorMagnetHardware(VectorMagnetHardwareInterface):
    """Concrete hardware driver implementing native sweep & status polling.
    Vector Magnet Hardware Driver for <https://cryomagnetics.com/products/model-4g-bipolar-power-supplies-superconducting-magnets/>

    Features:
     - Native sweep control for X, Y (dual supply) and Z (single supply)
     - Adaptive polling (faster while sweeping)
     - Automatic quench detection (pattern match in SWEEP? string)
     - Per-axis heater (persistent switch) control (auto-managed by logic)
     - Robust serial query with echo filtering and retry

    Threading:
     - A background polling thread runs until deactivation.
     - All serial access synchronized via a mutex to serialize CHAN selection and queries.

    Safety:
     - All exceptions in polling catch and emit a communication error without stopping the thread.

    Example Config:

    vector_magnet:
        module.Class: 'power_supply.vector_magnet.VectorMagnetHardware'
        options:
            dual_com: 'COM4'
            dual_baud: 9600
            single_com: 'COM10'
            single_baud: 9600
            line_termination: "\r"
            poll_interval_s: 1.0
            fast_poll_interval_s: 0.25
            calibration_matrix_diagonal_T_per_A: { x: 0.00982, y: 0.00987, z: 0.01073 }
            max_currents_A: { x: 1, y: 1, z: 1 }#{ x: 50.9158, y: 50.6816, z: 46.5793 }
            max_field_T: 0.01
            vector_field_limit_T: 0.01
            ramp_rates_A_per_s: { x: 0.1022, y: 0.1014, z: 0.0906 }
            current_tolerance_A: 0.002
            heater_warmup_s: 10
            heater_cooldown_s: 10
            default_persistent_mode: false
            persistent_idle_behavior: 'zero_leads'   # or 'hold_leads'
            enable_software_ramp_fallback: false
            use_native_sweep: true
            log_directory: ''          # optional override; empty = auto (data dir or CWD)

    """

    # Configuration options
    _dual_com: str = ConfigOption('dual_com', missing='error')
    _dual_baud: int = ConfigOption('dual_baud', default=9600, missing='warn')
    _single_com: str = ConfigOption('single_com', missing='error')
    _single_baud: int = ConfigOption('single_baud', default=9600, missing='warn')

    _poll_interval_s: float = ConfigOption('poll_interval_s', default=1.0, missing='nothing')
    _fast_poll_interval_s: float = ConfigOption('fast_poll_interval_s', default=0.25, missing='nothing')

    _cal_diag: Dict[str, float] = ConfigOption('calibration_matrix_diagonal_T_per_A', missing='error')
    _max_currents: Dict[str, float] = ConfigOption('max_currents_A', missing='error')
    _vector_field_limit_T: float = ConfigOption('vector_field_limit_T', default=0.5, missing='warn')
    _max_field_T: float = ConfigOption('max_field_T', default=0.5, missing='warn')
    _ramp_rates: Dict[str, float] = ConfigOption('ramp_rates_A_per_s', missing='error')
    _current_tolerance_A: float = ConfigOption('current_tolerance_A', default=0.01, missing='nothing')

    _heater_warmup_s: float = ConfigOption('heater_warmup_s', default=10.0, missing='nothing')
    _heater_cooldown_s: float = ConfigOption('heater_cooldown_s', default=10.0, missing='nothing')

    _default_persistent: bool = ConfigOption('default_persistent_mode', default=False, missing='nothing')
    _persistent_idle_behavior: str = ConfigOption('persistent_idle_behavior', default='zero_leads', missing='nothing')
    _use_native_sweep: bool = ConfigOption('use_native_sweep', default=True, missing='nothing')

    _log_directory: str = ConfigOption('log_directory', default='', missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()
        self._dual_port: Optional[_SerialPortWrapper] = None
        self._single_port: Optional[_SerialPortWrapper] = None
        self._stop_poll = False
        self._poll_thread: Optional[threading.Thread] = None

        # Channel mapping (dual supply)
        self._axis_chan = {'x': 1, 'y': 2}

        # Cached last polled status
        self._cached_status: Dict[str, Dict[str, Any]] = {'x': {}, 'y': {}, 'z': {}}

        # Regex for quench detection in SWEEP? response
        self._quench_pattern = re.compile(r'QUENCH', re.IGNORECASE)

        # Adaptive poll: fast interval used while any axis is sweeping
        self._adaptive_fast = True

        # Backoff for retry strategy in queries
        self._ramp_query_backoff_s = 0.03

        # Fixed line termination (CR)
        self._line_termination = '\r'

    # ---------------- Lifecycle ----------------

    def on_activate(self):
        print("VectorMagnetHardware: opening serial ports.")
        self._open_ports()
        try:
            self._dual_port.reset_input_buffer()
            self._single_port.reset_input_buffer()
        except Exception:
            pass
        self._enter_remote_mode()
        self._stop_poll = False
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def on_deactivate(self):
        self._stop_poll = True
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        self._close_ports()

    # ---------------- Serial Port Handling ----------------

    def _open_ports(self):
        try:
            self._dual_port = _SerialPortWrapper(self._dual_com, self._dual_baud, timeout=1.0)
        except Exception as exc:
            raise RuntimeError(f"Failed to open dual supply port {self._dual_com}: {exc}") from exc
        try:
            self._single_port = _SerialPortWrapper(self._single_com, self._single_baud, timeout=1.0)
        except Exception as exc:
            raise RuntimeError(f"Failed to open single supply port {self._single_com}: {exc}") from exc

    def _close_ports(self):
        for ref in (self._dual_port, self._single_port):
            if ref:
                ref.close()
        self._dual_port = None
        self._single_port = None

    # ---------------- Low-level I/O ----------------

    def _write(self, port: _SerialPortWrapper, cmd: str):
        """Send a single ASCII command with CR termination."""
        data = (cmd.strip() + self._line_termination).encode('ascii')
        port.write_bytes(data)

    def _collect_bytes(self, port: _SerialPortWrapper, window_s: float, silence_s: float = 0.05) -> bytes:
        """Collect incoming bytes for up to window_s or until silence duration met after any data."""
        end = time.time() + window_s
        buf = bytearray()
        got = False
        last = time.time()
        while time.time() < end:
            waiting = port.in_waiting()
            if waiting:
                buf.extend(port.read(waiting))
                got = True
                last = time.time()
            else:
                if got and (time.time() - last) >= silence_s:
                    break
                time.sleep(0.002)
        return bytes(buf)

    @staticmethod
    def _split_segments(raw: bytes) -> list[str]:
        if not raw:
            return []
        txt = raw.decode('ascii', errors='ignore')
        return [s.strip() for s in re.split(r'[\r\n]+', txt) if s.strip()]

    @staticmethod
    def _is_echo(seg: str, cmd_up: str) -> bool:
        up = seg.upper()
        if up == cmd_up:
            return True
        if up.startswith('CHAN '):
            return True
        if up in ('REMOTE', 'QRESET', 'PSHTR', 'ULIM', 'LLIM'):
            return True
        return False

    def _query(self, port: _SerialPortWrapper, cmd: str, attempts: int = 3) -> str:
        """
        Robust query:
          - Send command
          - Wait for response window
          - Filter out command echo / housekeeping tokens
          - Return first numeric or status string
          - Retry if only echoes received
        """
        cmd_up = cmd.strip().upper()
        first_non_echo = None
        for attempt in range(1, attempts + 1):
            self._write(port, cmd)
            time.sleep(0.15 if attempt == 1 else 0.10 + (attempt - 2) * self._ramp_query_backoff_s)
            raw = self._collect_bytes(port, window_s=0.35 if attempt == 1 else 0.25)
            segments = self._split_segments(raw)
            for seg in segments:
                if self._is_echo(seg, cmd_up):
                    continue
                if first_non_echo is None:
                    first_non_echo = seg
                up = seg.upper()
                if any(c.isdigit() for c in seg) or up in (
                        'STANDBY', 'RAMPING', 'HOLD', 'SWEEPING UP', 'SWEEPING DOWN'):
                    return seg
        if first_non_echo:
            return first_non_echo
        raise TimeoutError(f"No non-echo response for {cmd}")

    # ---------------- Public API (Interface Implementation) ----------------

    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float):
        self._ramp_rates[axis] = float(rate_A_per_s)

    def start_axis_sweep(self, axis: str, target_A: float, fast: bool = False):
        if not self._use_native_sweep or axis not in ('x', 'y', 'z'):
            return
        current = self.get_axis_current(axis, fresh=False)
        if math.isnan(current):
            current = 0.0
        if abs(target_A - current) <= self._current_tolerance_A:
            return

        direction_up = target_A > current
        if axis in ('x', 'y'):
            port = self._dual_port
            chan = self._axis_chan[axis]
        else:
            port = self._single_port
            chan = None

        with self._mutex:
            if chan is not None:
                self._write(port, f'CHAN {chan}')
                time.sleep(0.10)
                self._collect_bytes(port, 0.18, 0.04)

            # Configure ULIM/LLIM and issue sweep
            if direction_up:
                self._write(port, f'LLIM {min(current, target_A):.6f}')
                self._write(port, f'ULIM {target_A:.6f}')
                cmd = 'SWEEP UP'
            else:
                self._write(port, f'ULIM {max(current, target_A):.6f}')
                self._write(port, f'LLIM {target_A:.6f}')
                cmd = 'SWEEP DOWN'
            if fast:
                cmd += ' FAST'
            self._write(port, cmd)

    def sweep_zero(self, fast: bool = False):
        """Zero all axes using native SWEEP ZERO (dual supply both channels, then single)."""
        with self._mutex:
            # Dual supply X/Y
            for axis, chan in self._axis_chan.items():
                self._write(self._dual_port, f'CHAN {chan}')
                time.sleep(0.05)
                self._write(self._dual_port, 'SWEEP ZERO' + (' FAST' if fast else ''))
            # Single supply Z
            self._write(self._single_port, 'SWEEP ZERO' + (' FAST' if fast else ''))

    def set_axis_heater(self, axis: str, on: bool):
        """
        Set a per-axis heater (x/y share a physical supply but are addressed by CHAN selection).
        Silent on errors (instrument may not have active heater).
        """
        try:
            with self._mutex:
                if axis in ('x', 'y'):
                    chan = self._axis_chan[axis]
                    self._write(self._dual_port, f'CHAN {chan}')
                    time.sleep(0.05)
                    self._collect_bytes(self._dual_port, 0.15, 0.04)
                    self._write(self._dual_port, f'PSHTR {"ON" if on else "OFF"}')
                elif axis == 'z':
                    self._write(self._single_port, f'PSHTR {"ON" if on else "OFF"}')
        except Exception:
            pass  # ignore

    def query_axis_heater(self, axis: str) -> bool:
        """Query per-axis heater state (using CHAN for x/y). Returns False on error."""
        try:
            if axis in ('x', 'y'):
                chan = self._axis_chan[axis]
                self._write(self._dual_port, f'CHAN {chan}')
                time.sleep(0.05)
                self._collect_bytes(self._dual_port, 0.15, 0.04)
                resp = self._query(self._dual_port, 'PSHTR?')
                return resp.startswith('1')
            elif axis == 'z':
                resp = self._query(self._single_port, 'PSHTR?')
                return resp.startswith('1')
        except Exception:
            return False
        return False

    # Interface compatibility (group-level)
    def set_heater(self, group: str, on: bool):
        if group == 'xy':
            self.set_axis_heater('x', on)
            self.set_axis_heater('y', on)
        elif group in ('x', 'y', 'z'):
            self.set_axis_heater(group, on)
        else:
            raise ValueError(f"Unsupported heater group '{group}'")

    def query_heater(self, group: str) -> bool:
        if group == 'xy':
            return self.query_axis_heater('x') or self.query_axis_heater('y')
        if group in ('x', 'y', 'z'):
            return self.query_axis_heater(group)
        return False

    def reset_quench(self):
        with self._mutex:
            self._write(self._dual_port, 'QRESET')
            self._write(self._single_port, 'QRESET')

    def get_axis_current(self, axis: str, fresh: bool = False) -> float:
        if not fresh:
            v = self._cached_status[axis].get('IOUT')
            return v if v is not None else float('nan')
        val = self._query_axis_value(axis, 'IOUT?')
        if val is not None:
            self._cached_status[axis]['IOUT'] = val
        return val if val is not None else float('nan')

    def get_axis_magnet_current(self, axis: str, fresh: bool = True) -> float:
        if not fresh:
            v = self._cached_status[axis].get('IMAG')
            return v if v is not None else float('nan')
        val = self._query_axis_value(axis, 'IMAG?')
        if val is not None:
            self._cached_status[axis]['IMAG'] = val
        return val if val is not None else float('nan')

    # ---------------- Internal Helpers ----------------

    def _enter_remote_mode(self):
        """Enter remote mode on both supplies (best-effort)."""
        with self._mutex:
            for port, tag in ((self._dual_port, 'dual'), (self._single_port, 'single')):
                try:
                    self._write(port, 'REMOTE')
                    time.sleep(0.08)
                    self._collect_bytes(port, 0.25, 0.05)
                except Exception as e:
                    self.sigCommunicationError.emit(f'REMOTE fail {tag}: {e}')

    def _query_axis_value(self, axis: str, cmd: str) -> Optional[float]:
        """Helper to query numeric value for an axis with CHAN selection if needed."""
        port = self._dual_port if axis in ('x', 'y') else self._single_port
        try:
            with self._mutex:
                if axis in ('x', 'y'):
                    chan = self._axis_chan[axis]
                    self._write(port, f'CHAN {chan}')
                    time.sleep(0.10)
                    self._collect_bytes(port, 0.20, 0.04)
                resp = self._query(port, cmd)
            return self._parse_numeric(resp)
        except Exception:
            return None

    @staticmethod
    def _parse_numeric(resp: str) -> float:
        if not resp:
            return float('nan')
        s = resp.strip()
        for prefix in ('IOUT=', 'IMAG='):
            if s.upper().startswith(prefix):
                s = s[len(prefix):].strip()
        if s.endswith('A'):
            s = s[:-1].strip()
        try:
            return float(s)
        except ValueError:
            return float('nan')

    # ---------------- Polling Thread ----------------

    def _poll_loop(self):
        """Main polling loop: gather status for each axis and emit."""
        base = self._poll_interval_s
        fast = self._fast_poll_interval_s
        while not self._stop_poll:
            start = time.time()
            try:
                status = self._poll_status_once()
                any_sweeping = any('Sweeping' in (st.get('SWEEP', '') or '') for st in status.values())
                interval = fast if (any_sweeping and self._adaptive_fast) else base
                self.sigAxisStatus.emit(status)
                self._detect_quench(status)
            except Exception as exc:
                interval = base
                self.sigCommunicationError.emit(f'Polling error: {exc}')
            elapsed = time.time() - start
            time.sleep(max(0.02, interval - elapsed))

    def _safe_query(self, port, cmd: str) -> Optional[str]:
        try:
            return self._query(port, cmd)
        except TimeoutError:
            time.sleep(0.05)
            try:
                return self._query(port, cmd)
            except Exception:
                return None

    def _poll_status_once(self) -> Dict[str, Dict[str, Any]]:
        """Poll all axes (X/Y via channel select, Z separately)."""
        now = time.time()
        status: Dict[str, Dict[str, Any]] = {}

        # Dual supply axes
        with self._mutex:
            for axis, chan in self._axis_chan.items():
                self._write(self._dual_port, f'CHAN {chan}')
                time.sleep(0.08)
                self._collect_bytes(self._dual_port, 0.15, 0.04)
                iout = self._safe_query(self._dual_port, 'IOUT?') or ''
                imag = self._safe_query(self._dual_port, 'IMAG?') or ''
                sweep = self._safe_query(self._dual_port, 'SWEEP?') or ''
                heater = self._safe_query(self._dual_port, 'PSHTR?') or '0'
                status[axis] = {
                    'IOUT': self._parse_numeric(iout),
                    'IMAG': self._parse_numeric(imag),
                    'SWEEP': sweep,
                    'heater': heater.startswith('1'),
                    'timestamp': now
                }

        # Single supply axis Z
        with self._mutex:
            ioutz = self._safe_query(self._single_port, 'IOUT?') or ''
            imagz = self._safe_query(self._single_port, 'IMAG?') or ''
            sweepz = self._safe_query(self._single_port, 'SWEEP?') or ''
            heaterz = self._safe_query(self._single_port, 'PSHTR?') or '0'
        status['z'] = {
            'IOUT': self._parse_numeric(ioutz),
            'IMAG': self._parse_numeric(imagz),
            'SWEEP': sweepz,
            'heater': heaterz.startswith('1'),
            'timestamp': now
        }

        self._cached_status.update(status)
        return status

    # ---------------- Quench Detection ----------------

    def _detect_quench(self, status: Dict[str, Dict[str, Any]]):
        quench_axes = {ax: True for ax, st in status.items()
                       if self._quench_pattern.search(st.get('SWEEP', '') or '')}
        if quench_axes:
            self.sigQuench.emit(quench_axes)

    # ---------------- Introspection Helpers ----------------

    def axis_limits_A(self) -> Dict[str, float]:
        return dict(self._max_currents)

    def ramp_rates_A_per_s(self) -> Dict[str, float]:
        return dict(self._ramp_rates)