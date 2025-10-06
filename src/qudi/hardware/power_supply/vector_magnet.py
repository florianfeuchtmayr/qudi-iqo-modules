# -*- coding: utf-8 -*-
"""
Hardware composite driver for a 3-axis vector superconducting magnet powered by:
 - One dual Cryomagnetics 4G supply (X and Y channels selected via CHAN 1/2)
 - One single Cryomagnetics 4G supply (Z)

Implements:
 - Native sweep control (ULIM/LLIM + SWEEP UP/DOWN/ZERO)
 - Heater control (PSHTR ON/OFF) for dual block and single supply
 - Periodic polling (Iout, Imag, Sweep, PSHTR)
 - Quench detection (non-interference)
 - Emergency zero sweep

Non-blocking design: Polling runs in a thread; all device I/O serialized with a Mutex.

NOTE:
 If your environment already provides a generic serial abstraction inside Qudi, replace
 the _SerialPortWrapper with that abstraction.

Author: Generated for your lab (2025)
"""
from __future__ import annotations
import time
import threading
import re
import os
from typing import Dict, Optional, Any
from PySide2 import QtCore
from qudi.core.module import Base
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex

from qudi.interface.vector_magnet_interface import VectorMagnetHardwareInterface

try:
    import serial  # pyserial
except ImportError:
    serial = None


class _SerialPortWrapper:
    """Minimal serial port wrapper to allow easy substitution if Qudi has a base class."""
    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial not installed. Please 'pip install pyserial'.")
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=1.0)

    def write_line(self, line: str):
        self._ser.write(line.encode('ascii', errors='ignore'))

    def readline(self) -> str:
        raw = self._ser.readline()
        return raw.decode('ascii', errors='ignore')

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass


class VectorMagnetHardware(VectorMagnetHardwareInterface):
    """
    Exports signals consumed by logic:

    sigAxisStatus: dict per axis:
        {
          'x': {'IOUT': float, 'IMAG': float, 'SWEEP': str, 'heater': bool, 'timestamp': float},
          'y': {...},
          'z': {...}
        }

    sigQuench: dict e.g. {'x': True, 'y': True} if those axes show quench text in SWEEP?

    sigCommunicationError: str with diagnostic message

    Configuration keys (see .cfg):
      dual_com, dual_baud, single_com, single_baud
      line_termination
      poll_interval_s, fast_poll_interval_s
      calibration_matrix_diagonal_T_per_A
      max_currents_A
      vector_field_limit_T, max_field_T
      ramp_rates_A_per_s
      current_tolerance_A
      heater_warmup_s, heater_cooldown_s
      default_persistent_mode
      persistent_idle_behavior
      enable_software_ramp_fallback
      use_native_sweep
      log_directory
    """
    _dual_com: str = ConfigOption('dual_com', missing='error')
    _dual_baud: int = ConfigOption('dual_baud', default=115115, missing='warn')
    _single_com: str = ConfigOption('single_com', missing='error')
    _single_baud: int = ConfigOption('single_baud', default=115115, missing='warn')
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

        self._axis_chan = {'x': 1, 'y': 2}  # CHAN mapping for dual supply
        self._cached_status: Dict[str, Dict[str, Any]] = {'x': {}, 'y': {}, 'z': {}}

        # Quench detection heuristics
        self._quench_pattern = re.compile(r'QUENCH', re.IGNORECASE)

    # ---------------- Lifecycle ----------------
    def on_activate(self):
        self._open_ports()
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
        if self._dual_port:
            self._dual_port.close()
            self._dual_port = None
        if self._single_port:
            self._single_port.close()
            self._single_port = None

    # ---------------- Low-level IO ----------------
    def _write(self, port: _SerialPortWrapper, cmd: str):
        # Multi-subcommand chaining possible but we keep one per write for clarity
        line = cmd.strip() + self._line_termination
        port.write_line(line)

    def _query(self, port: _SerialPortWrapper, cmd: str, timeout: float = 1.0) -> str:
        with self._mutex:
            self._write(port, cmd)
            t0 = time.time()
            while True:
                resp = port.readline()
                if resp:
                    return resp.strip()
                if time.time() - t0 > timeout:
                    raise TimeoutError(f"Timeout waiting response for {cmd}")
                time.sleep(0.01)

    # ---------------- Public Hardware Control API (used by logic) ----------------
    def set_axis_ramp_rate(self, axis: str, rate_A_per_s: float):
        """Update internal ramp rates (applies to future sweeps)."""
        self._ramp_rates[axis] = float(rate_A_per_s)
        # If using native multi-range, you could map to RATE 0 ... For now: not applied directly here.

    def start_axis_sweep(self, axis: str, target_A: float, fast: bool = False):
        if not self._use_native_sweep:
            # Software fallback: mark a simple target and let logic handle stepping
            return  # logic will handle if fallback selected
        if axis not in ('x', 'y', 'z'):
            return
        # Acquire current to decide sweep direction
        current = self.get_axis_current(axis, fresh=True)
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
            if axis in ('x', 'y'):
                self._write(port, f'CHAN {chan}')
            # Set appropriate limit
            if direction_up:
                self._write(port, f'ULIM {target_A:.6f}')
                cmd = 'SWEEP UP'
            else:
                self._write(port, f'LLIM {target_A:.6f}')
                cmd = 'SWEEP DOWN'
            if fast:
                cmd += ' FAST'
            self._write(port, cmd)

    def sweep_zero(self, fast: bool = False):
        """Issue SWEEP ZERO to all axes (X,Y on dual, Z on single)."""
        with self._mutex:
            # Dual
            self._write(self._dual_port, f'CHAN {self._axis_chan["x"]}')
            self._write(self._dual_port, 'SWEEP ZERO' + (' FAST' if fast else ''))
            self._write(self._dual_port, f'CHAN {self._axis_chan["y"]}')
            self._write(self._dual_port, 'SWEEP ZERO' + (' FAST' if fast else ''))
            # Single
            self._write(self._single_port, 'SWEEP ZERO' + (' FAST' if fast else ''))

    def set_heater(self, group: str, on: bool):
        """
        group: 'xy' or 'z'
        We do not toggle for fast automatically (per requirement).
        """
        port = self._dual_port if group == 'xy' else self._single_port
        with self._mutex:
            self._write(port, f'PSHTR {"ON" if on else "OFF"}')

    def query_heater(self, group: str) -> bool:
        port = self._dual_port if group == 'xy' else self._single_port
        resp = self._query(port, 'PSHTR?')
        return resp.startswith('1')

    def reset_quench(self):
        with self._mutex:
            # Safe attempt both supplies
            self._write(self._dual_port, 'QRESET')
            self._write(self._single_port, 'QRESET')

    def get_axis_current(self, axis: str, fresh: bool = False) -> float:
        if not fresh and self._cached_status[axis].get('IOUT') is not None:
            return self._cached_status[axis]['IOUT']
        val = self._query_axis_value(axis, 'IOUT?')
        if val is not None:
            self._cached_status[axis]['IOUT'] = val
        return val if val is not None else float('nan')

    def get_axis_magnet_current(self, axis: str, fresh: bool = True) -> float:
        val = self._query_axis_value(axis, 'IMAG?') if fresh else self._cached_status[axis].get('IMAG', float('nan'))
        if val is not None:
            self._cached_status[axis]['IMAG'] = val
        return val if val is not None else float('nan')

    # ---------------- Internal Helpers ----------------
    def _enter_remote_mode(self):
        with self._mutex:
            try:
                self._write(self._dual_port, 'REMOTE')
            except Exception:
                pass
            try:
                self._write(self._single_port, 'REMOTE')
            except Exception:
                pass

    def _query_axis_value(self, axis: str, cmd: str) -> Optional[float]:
        port = self._dual_port if axis in ('x', 'y') else self._single_port
        if axis in ('x', 'y'):
            chan = self._axis_chan[axis]
            with self._mutex:
                self._write(port, f'CHAN {chan}')
                resp = self._query(port, cmd)
        else:
            with self._mutex:
                resp = self._query(port, cmd)
        try:
            # Responses like "87.935 A" or "87.9350 A"
            token = resp.split()[0]
            return float(token)
        except Exception:
            return None

    # ---------------- Poll Loop ----------------
    def _poll_loop(self):
        base_interval = self._poll_interval_s
        while not self._stop_poll:
            t0 = time.time()
            try:
                status = self._poll_status_once()
                self.sigAxisStatus.emit(status)
                self._detect_quench(status)
            except Exception as exc:
                self.sigCommunicationError.emit(f'Polling error: {exc}')
            elapsed = time.time() - t0
            sleep_for = max(0.05, base_interval - elapsed)
            time.sleep(sleep_for)

    def _poll_status_once(self) -> Dict[str, Dict[str, Any]]:
        now = time.time()
        status: Dict[str, Dict[str, Any]] = {}
        # Dual axes
        with self._mutex:
            for axis, chan in self._axis_chan.items():
                self._write(self._dual_port, f'CHAN {chan}')
                iout = self._query(self._dual_port, 'IOUT?')
                imag = self._query(self._dual_port, 'IMAG?')
                sweep = self._query(self._dual_port, 'SWEEP?')
                htr = self._query(self._dual_port, 'PSHTR?')
                status[axis] = {
                    'IOUT': self._parse_value(iout),
                    'IMAG': self._parse_value(imag),
                    'SWEEP': sweep,
                    'heater': htr.startswith('1'),
                    'timestamp': now
                }
        # Single Z
        with self._mutex:
            ioutz = self._query(self._single_port, 'IOUT?')
            imagz = self._query(self._single_port, 'IMAG?')
            sweepz = self._query(self._single_port, 'SWEEP?')
            htrz = self._query(self._single_port, 'PSHTR?')
        status['z'] = {
            'IOUT': self._parse_value(ioutz),
            'IMAG': self._parse_value(imagz),
            'SWEEP': sweepz,
            'heater': htrz.startswith('1'),
            'timestamp': now
        }
        self._cached_status.update(status)
        return status

    def _parse_value(self, resp: str) -> float:
        try:
            return float(resp.split()[0])
        except Exception:
            return float('nan')

    def _detect_quench(self, status: Dict[str, Dict[str, Any]]):
        quench_axes = {}
        for ax, st in status.items():
            swp = st.get('SWEEP', '')
            if self._quench_pattern.search(swp):
                quench_axes[ax] = True
        if quench_axes:
            self.sigQuench.emit(quench_axes)