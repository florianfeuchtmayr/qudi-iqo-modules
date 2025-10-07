# -*- coding: utf-8 -*-
"""
Vector Magnet Hardware Driver – production version
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
    import serial
except ImportError:
    serial = None


class _SerialPortWrapper:
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
    # Config options
    _dual_com: str = ConfigOption('dual_com', missing='error')
    _dual_baud: int = ConfigOption('dual_baud', default=9600, missing='warn')
    _single_com: str = ConfigOption('single_com', missing='error')
    _single_baud: int = ConfigOption('single_baud', default=9600, missing='warn')
    _line_termination: str = ConfigOption('line_termination', default='\r', missing='nothing')

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
    _enable_software_ramp_fallback: bool = ConfigOption('enable_software_ramp_fallback', default=True, missing='nothing')
    _use_native_sweep: bool = ConfigOption('use_native_sweep', default=True, missing='nothing')

    _log_directory: str = ConfigOption('log_directory', default='', missing='nothing')
    _debug_io_config: bool = ConfigOption('debug_io', default=False, missing='nothing')

    # Signals
    sigAxisStatus = QtCore.Signal(dict)
    sigQuench = QtCore.Signal(dict)
    sigCommunicationError = QtCore.Signal(str)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()
        self._dual_port: Optional[_SerialPortWrapper] = None
        self._single_port: Optional[_SerialPortWrapper] = None
        self._stop_poll = False
        self._poll_thread: Optional[threading.Thread] = None
        self._axis_chan = {'x': 1, 'y': 2}
        self._cached_status: Dict[str, Dict[str, Any]] = {'x': {}, 'y': {}, 'z': {}}
        self._quench_pattern = re.compile(r'QUENCH', re.IGNORECASE)

        # Debug flag (set False to reduce noise after commissioning)
        self._debug_io = False
        self._adaptive_fast = True  # new flag for adaptive polling
        self._ramp_query_backoff_s = 0.03  # backoff per retry

    # ---------- Lifecycle ----------
    def on_activate(self):
        print("VectorMagnetHardware: opening ports", self._dual_com, self._single_com)
        self._sanitize_line_termination()
        self._open_ports()
        self._debug_io = self._debug_io_config
        # One-time flush
        try:
            self._dual_port.reset_input_buffer()
            self._single_port.reset_input_buffer()
        except Exception:
            pass
        self._enter_remote_mode()
        self._stop_poll = False
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        print("VectorMagnetHardware: poll thread started")

    def on_deactivate(self):
        self._stop_poll = True
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        self._close_ports()

    # ---------- Serial Port Handling ----------
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
        if self._dual_port:
            self._dual_port.close()
            self._dual_port = None
        if self._single_port:
            self._single_port.close()
            self._single_port = None

    # ---------- Line termination sanitization ----------
    def _sanitize_line_termination(self):
        # Handle accidental literal '\r' from single-quoted YAML
        mapping = {
            '\\r': '\r',
            '\\n': '\n',
            '\\r\\n': '\r\n'
        }
        if self._line_termination in mapping:
            print(f"Sanitized line termination {repr(self._line_termination)} -> {repr(mapping[self._line_termination])}")
            self._line_termination = mapping[self._line_termination]

    # ---------- Low-level IO ----------
    def _write(self, port: _SerialPortWrapper, cmd: str):
        data = (cmd.strip() + self._line_termination).encode('ascii')
        port.write_bytes(data)
        if self._debug_io:
            print(f"TX {cmd}:"," ".join(f"{b:02X}" for b in data))

    def _collect_bytes(self, port: _SerialPortWrapper, window_s: float, silence_s: float = 0.05) -> bytes:
        end = time.time() + window_s
        buf = bytearray()
        got = False
        last = time.time()
        while time.time() < end:
            w = port.in_waiting()
            if w:
                chunk = port.read(w)
                buf.extend(chunk)
                got = True
                last = time.time()
            else:
                if got and (time.time() - last) >= silence_s:
                    break
                time.sleep(0.002)
        return bytes(buf)

    def _split_segments(self, raw: bytes) -> list[str]:
        if not raw:
            return []
        txt = raw.decode('ascii', errors='ignore')
        return [s.strip() for s in re.split(r'[\r\n]+', txt) if s.strip()]

    def _is_echo(self, seg: str, cmd_up: str) -> bool:
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
                Test-script style:
                  - send cmd
                  - wait fixed ~0.15 s
                  - read burst
                  - parse segments; skip echoes; accept first numeric/status
                  - retry once if only echoes
                """
        cmd_up = cmd.strip().upper()
        first_non_echo = None
        for attempt in range(1, attempts + 1):
            self._write(port, cmd)
            # Wait: slightly longer on attempt 1, then shorter plus a backoff if previous had only echo
            time.sleep(0.15 if attempt == 1 else 0.10 + (attempt - 2) * self._ramp_query_backoff_s)
            raw = self._collect_bytes(port, window_s=0.35 if attempt == 1 else 0.25)
            segs = self._split_segments(raw)
            if self._debug_io:
                print(f"RX {cmd} attempt {attempt}: {segs}")
            saw_non_echo_this_attempt = False
            for seg in segs:
                if self._is_echo(seg, cmd_up):
                    continue
                if first_non_echo is None:
                    first_non_echo = seg
                saw_non_echo_this_attempt = True
                up = seg.upper()
                if any(c.isdigit() for c in seg) or up in (
                'STANDBY', 'RAMPING', 'HOLD', 'SWEEPING UP', 'SWEEPING DOWN'):
                    if self._debug_io:
                        print(f"SER >> {cmd} << {repr(seg)}")
                    return seg
            # If we saw a non-echo but it wasn't numeric/status, try next attempt (maybe next response chunk)
            # If we saw nothing but echoes, we also retry (unless last attempt).
        if first_non_echo:
            if self._debug_io:
                print(f"SER >> {cmd} (fallback) << {repr(first_non_echo)}")
            return first_non_echo
        raise TimeoutError(f"No non-echo response for {cmd}")

    # ---------- Public API ----------
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
                self._collect_bytes(port, 0.20, 0.04)  # drain echo
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
        with self._mutex:
            # X
            self._write(self._dual_port, f'CHAN {self._axis_chan["x"]}')
            time.sleep(0.05)
            self._write(self._dual_port, 'SWEEP ZERO' + (' FAST' if fast else ''))
            # Y
            self._write(self._dual_port, f'CHAN {self._axis_chan["y"]}')
            time.sleep(0.05)
            self._write(self._dual_port, 'SWEEP ZERO' + (' FAST' if fast else ''))
            # Z
            self._write(self._single_port, 'SWEEP ZERO' + (' FAST' if fast else ''))

    def set_heater(self, group: str, on: bool):
        port = self._dual_port if group == 'xy' else self._single_port
        with self._mutex:
            self._write(port, f'PSHTR {"ON" if on else "OFF"}')

    def query_heater(self, group: str) -> bool:
        port = self._dual_port if group == 'xy' else self._single_port
        resp = self._query(port, 'PSHTR?')
        return resp.startswith('1')

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

    # ---------- Internal helpers ----------
    def _enter_remote_mode(self):
        with self._mutex:
            for port, tag in ((self._dual_port, 'dual'), (self._single_port, 'single')):
                try:
                    self._write(port, 'REMOTE')
                    time.sleep(0.08)
                    self._collect_bytes(port, 0.25, 0.05)  # discard echo
                except Exception as e:
                    self.sigCommunicationError.emit(f'REMOTE fail {tag}: {e}')

    def _query_axis_value(self, axis: str, cmd: str) -> Optional[float]:
        port = self._dual_port if axis in ('x', 'y') else self._single_port
        try:
            with self._mutex:
                if axis in ('x', 'y'):
                    chan = self._axis_chan[axis]
                    self._write(port, f'CHAN {chan}')
                    time.sleep(0.10)
                    self._collect_bytes(port, 0.20, 0.04)  # drain echo only
                resp = self._query(port, cmd)
            return self._parse_value(resp)
        except Exception:
            return None

    # ---------- Polling ----------
    def _poll_loop(self):
        base = self._poll_interval_s
        fast = self._fast_poll_interval_s
        while not self._stop_poll:
            t0 = time.time()
            try:
                status = self._poll_status_once()
                # decide interval adaptively
                any_sweeping = any('Sweeping' in (st.get('SWEEP', '')) for st in status.values())
                current_interval = fast if (any_sweeping and self._adaptive_fast) else base
                self.sigAxisStatus.emit(status)
                self._detect_quench(status)
            except Exception as exc:
                current_interval = base
                print("VectorMagnetHardware poll exception:", exc)
                self.sigCommunicationError.emit(f'Polling error: {exc}')
            elapsed = time.time() - t0
            time.sleep(max(0.02, current_interval - elapsed))

    # Add a safe single-command retry wrapper for polling only (to reduce full-cycle failures)
    def _safe_query(self, port, cmd: str) -> Optional[str]:
        try:
            return self._query(port, cmd)
        except TimeoutError:
            # One short extra attempt after small backoff
            time.sleep(0.05)
            try:
                return self._query(port, cmd)
            except Exception:
                return None

    def _poll_status_once(self) -> Dict[str, Dict[str, Any]]:
        now = time.time()
        status: Dict[str, Dict[str, Any]] = {}
        with self._mutex:
            # Dual channels
            for axis, chan in self._axis_chan.items():
                self._write(self._dual_port, f'CHAN {chan}')
                time.sleep(0.10)
                self._collect_bytes(self._dual_port, 0.20, 0.04)
                iout = self._safe_query(self._dual_port, 'IOUT?') or 'IOUT?'
                imag = self._safe_query(self._dual_port, 'IMAG?') or 'IMAG?'
                sweep = self._safe_query(self._dual_port, 'SWEEP?') or 'SWEEP?'
                htr = self._safe_query(self._dual_port, 'PSHTR?') or '0'
                status[axis] = {
                    'IOUT': self._parse_value(iout),
                    'IMAG': self._parse_value(imag),
                    'SWEEP': sweep,
                    'heater': htr.startswith('1'),
                    'timestamp': now
                }
        # Single Z
        with self._mutex:
            ioutz = self._safe_query(self._single_port, 'IOUT?') or 'IOUT?'
            imagz = self._safe_query(self._single_port, 'IMAG?') or 'IMAG?'
            sweepz = self._safe_query(self._single_port, 'SWEEP?') or 'SWEEP?'
            htrz = self._safe_query(self._single_port, 'PSHTR?') or '0'
        status['z'] = {
            'IOUT': self._parse_value(ioutz),
            'IMAG': self._parse_value(imagz),
            'SWEEP': sweepz,
            'heater': htrz.startswith('1'),
            'timestamp': now
        }
        self._cached_status.update(status)
        return status

    # ---------- Parsing & Quench ----------
    def _parse_value(self, resp: str) -> float:
        if not resp:
            return float('nan')
        s = resp.strip()
        for p in ('IOUT=', 'IMAG='):
            if s.upper().startswith(p):
                s = s[len(p):].strip()
        if s.endswith('A'):
            s = s[:-1].strip()
        try:
            return float(s)
        except ValueError:
            return float('nan')

    def _detect_quench(self, status: Dict[str, Dict[str, Any]]):
        quench_axes = {}
        for ax, st in status.items():
            swp = st.get('SWEEP', '')
            if self._quench_pattern.search(swp):
                quench_axes[ax] = True
        if quench_axes:
            self.sigQuench.emit(quench_axes)