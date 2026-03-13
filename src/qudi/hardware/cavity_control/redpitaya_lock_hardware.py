# -*- coding: utf-8 -*-
"""
Red Pitaya hardware module implementing RedPitayaLockInterface for the lock-in+PID harmonic app.

- SSH-based register read/write using c/lock tool.
- Ramp configuration (low/high limits, step ticks, direction, sawtooth), enable/reset.
- Output mux selection (out1_sw, out2_sw).
- Oscilloscope configuration and acquisition:
  Uses a robust path: run a small Python snippet on RP to perform single-shot measurement
  and save a .npy dump, then SCP back and load into numpy for (t, chA, chB).

Note: This module focuses on the lock-in+PID app; paths assume the app is installed at
/opt/redpitaya/www/apps/lock_in+pid_harmonic on the Red Pitaya.
"""

import io
import os
import time
import json
import tempfile
import numpy as np
import paramiko
from scp import SCPClient
from typing import Tuple, Sequence, Optional, Dict

from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.core.module import Base
from qudi.util.mutex import Mutex

from qudi.interface.cavity_control_interface import RedPitayaLockInterface


class RedPitayaLockHardware(Base, RedPitayaLockInterface):
    # Connection/config
    _address: str = ConfigOption('address', default='', missing='error')
    _user: str = ConfigOption('user', default='root', missing='warn')
    _password: str = ConfigOption('password', default='root', missing='warn')

    # App paths
    _app_path: str = ConfigOption('app_path',
                                  default='/opt/redpitaya/www/apps/lock_in+pid_harmonic',
                                  missing='warn')
    _python_path: str = ConfigOption('python_path', default='/usr/bin/python3', missing='warn')

    # Osc defaults
    _osc_decimation: int = ConfigOption('osc_decimation', default=128, missing='warn')
    _osc_trigger_source: int = ConfigOption('osc_trigger_source', default=2, missing='warn')  # 2=Scan floor
    _osc_trig_pos: int = ConfigOption('osc_trig_pos', default=8191, missing='warn')
    _osc_hysteresis: int = ConfigOption('osc_hysteresis', default=1, missing='warn')
    _osc_threshold: int = ConfigOption('osc_threshold', default=0, missing='warn')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._ssh: Optional[paramiko.SSHClient] = None
        self._scp: Optional[SCPClient] = None
        self._lock = Mutex()

        self._connected: bool = False
        self._last_ao1_pos_v: Optional[float] = None

    # Lifecycle
    def on_activate(self) -> None:
        self.connect()

    def on_deactivate(self) -> None:
        self.disconnect()

    # RedPitayaLockInterface
    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            cli = paramiko.SSHClient()
            cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                cli.connect(hostname=str(self._address).strip(),
                            username=str(self._user).strip(),
                            password=str(self._password).strip())
            except Exception as e:
                self.log.exception('SSH connect failed')
                raise ConnectionError(f'Could not connect to RedPitaya at {self._address}: {e}') from e
            self._ssh = cli
            try:
                self._scp = SCPClient(self._ssh.get_transport())
            except Exception:
                self._scp = None
            self._connected = True
            self.log.info(f'Connected to RedPitaya @ {self._address}')

    def disconnect(self) -> None:
        with self._lock:
            try:
                if self._scp is not None:
                    try:
                        self._scp.close()
                    except Exception:
                        pass
                    self._scp = None
                if self._ssh is not None:
                    try:
                        self._ssh.close()
                    except Exception:
                        pass
                    self._ssh = None
            finally:
                self._connected = False
                self.log.info(f'Disconnected from RedPitaya @ {self._address}')

    def _exec(self, cmd: str, timeout: float = 5.0) -> Tuple[bytes, bytes, int]:
        """
        Simple SSH exec using Paramiko's exec_command.
        This mirrors the working path used in your rp_lock_in_pid_h toolchain.
        """
        if not self._connected or self._ssh is None:
            raise RuntimeError('Not connected to RedPitaya.')
        self.log.debug(f'RP exec: {cmd}')
        try:
            stdin, stdout, stderr = self._ssh.exec_command(cmd, timeout=timeout)
            out = stdout.read()
            err = stderr.read()
            try:
                status = stdout.channel.recv_exit_status()
            except Exception:
                status = 0
            return out, err, status
        except Exception as e:
            self.log.exception('SSH exec failed')
            raise

    def _lock_cmd(self, reg: str, val: Optional[int] = None) -> Tuple[int, bytes]:

        reg_arg = str(reg)
        cmd = f'/opt/redpitaya/www/apps/lock_in+pid_harmonic/c/lock {reg_arg}' if val is None else f'/opt/redpitaya/www/apps/lock_in+pid_harmonic/c/lock {reg_arg} {int(val)}'

        out, err, status = self._exec(cmd)
        if status != 0:
            self.log.error(f'Lock command error [{status}]: {err.decode("utf-8", errors="ignore")}')
        content = out.strip()
        try:
            txt = content.decode('utf-8', errors='ignore')
        except Exception:
            txt = ''
        parts = txt.split(':', 1)
        val_str = parts[1].strip() if len(parts) > 1 else parts[0].strip()
        try:
            value = int(val_str)
        except Exception:
            try:
                value = int(val_str, 0)
            except Exception:
                value = 0
        return value, content

    # RedPitayaLockInterface impl
    def write_reg(self, name: str, value: int) -> None:
        # Single locking layer only here
        with self._lock:
            _ = self._lock_cmd(name, int(value))

    def read_reg(self, name: str) -> int:
        with self._lock:
            value, _ = self._lock_cmd(name)
            return value

    def configure_ramp(self,
                       low_lim: int,
                       high_lim: int,
                       step_ticks: int,
                       direction_up: bool,
                       sawtooth: bool = False) -> None:
        # Do NOT lock here; write_reg handles locking
        self.write_reg('ramp_low_lim', int(low_lim))
        self.write_reg('ramp_hig_lim', int(high_lim))
        self.write_reg('ramp_step', int(step_ticks))
        self.write_reg('ramp_direction', 1 if direction_up else 0)
        self.write_reg('ramp_sawtooth', 1 if sawtooth else 0)

    def enable_ramp(self, enable: bool) -> None:
        # Do NOT lock here; write_reg handles locking
        self.write_reg('ramp_enable', 1 if enable else 0)

    def reset_ramp(self) -> None:
        # Do NOT lock here; write_reg handles locking
        self.write_reg('ramp_reset', 1)
        time.sleep(0.03)
        self.write_reg('ramp_reset', 0)

    def set_out_mux(self, out1_sel: Optional[int] = None, out2_sel: Optional[int] = None) -> None:
        # Do NOT lock here; write_reg handles locking
        if out1_sel is not None:
            self.write_reg('out1_sw', int(out1_sel))
        if out2_sel is not None:
            self.write_reg('out2_sw', int(out2_sel))

    def osc_config(self,
                   decimation: int,
                   trigger_source: int,
                   trig_pos: int = 8191,
                   hysteresis: int = 1,
                   threshold: int = 0) -> None:
        """
        Configure osc registers via c/osc tool (if available) or via c/lock mapping:
        We rely on the Python osc_get_ch path for acquisition, so here we only
        store desired parameters; acquisition uses them.
        """
        with self._lock:
            # Store desired osc params in JSON on the RP for the acquisition script to read
            params = {
                'decimation': int(decimation),
                'trigger_source': int(trigger_source),
                'trig_pos': int(trig_pos),
                'hysteresis': int(hysteresis),
                'threshold': int(threshold)
            }
            # Write a small JSON file to /root/qudi_osc_params.json
            payload = json.dumps(params)
            cmd = f"python3 -c 'open(\"/root/qudi_osc_params.json\",\"w\").write(\"{payload}\")'"
            try:
                self._exec(cmd)
            except Exception:
                # Fallback: ignore
                pass

            # Keep defaults locally too
            self._osc_decimation = int(decimation)
            self._osc_trigger_source = int(trigger_source)
            self._osc_trig_pos = int(trig_pos)
            self._osc_hysteresis = int(hysteresis)
            self._osc_threshold = int(threshold)

    def osc_acquire(self, wait: bool = True) -> None:
        """
        Execute a small Python acquisition on RP that:
        - Uses the app's Python API (hugo/control_finn) to configure osc and measure
        - Saves npy dump to /root/qudi_osc_dump.npy
        """
        with self._lock:
            py = f"""
        import numpy as np, json, sys
        sys.path.append("{self._app_path}/resources/remote_control/http_version")
        from control_finn import RedPitayaApp
        rp = RedPitayaApp("http://127.0.0.1/lock_in+pid_harmonic/?type=run")
        try:
            params = json.loads(open("/root/qudi_osc_params.json").read())
        except Exception:
            params = {{"decimation": {self._osc_decimation}, "trigger_source": {self._osc_trigger_source},
                       "trig_pos": {self._osc_trig_pos}, "hysteresis": {self._osc_hysteresis},
                       "threshold": {self._osc_threshold}}}
        rp.osc.measure('A_ri', dec=params['decimation'], trig_pos=params['trig_pos'],
                       hysteresis=params['hysteresis'], threshold=params['threshold'], wait=True)
        tt, ch1, ch2 = rp.osc.curv(raw=False)
        np.save('/root/qudi_osc_dump.npy', {{"t": np.array(tt, dtype=float),
                                             "ch1": np.array(ch1, dtype=float),
                                             "ch2": np.array(ch2, dtype=float)}})
        """
            script = py.replace("\n", "; ")
            cmd = f'{self._python_path} -c "{script}"'
            self._exec(cmd)

    def osc_curves(self, raw: bool = False) -> Tuple[Sequence[float], Sequence[float], Sequence[float]]:
        """SCP the last dump and return (t, ch1, ch2) arrays."""
        with self._lock:
            if self._scp is None:
                raise RuntimeError('SCP client not available')
            local_tmp = tempfile.NamedTemporaryFile(suffix='.npy', delete=False)
            local_path = local_tmp.name
            local_tmp.close()
            try:
                self._scp.get('/root/qudi_osc_dump.npy', local_path)
            except Exception as e:
                self.log.exception('SCP get failed')
                raise
            try:
                data = np.load(local_path, allow_pickle=True).item()
                t = np.array(data['t'], dtype=float)
                ch1 = np.array(data['ch1'], dtype=float)
                ch2 = np.array(data['ch2'], dtype=float)
                return t, ch1, ch2
            finally:
                try:
                    os.unlink(local_path)
                except Exception:
                    pass

    def get_ao1_position(self) -> Optional[float]:
        """
        Best-effort AO1 position (volts). If routing maps AO1 to a readable register, return it;
        otherwise return last inferred value if cached.
        """
        try:
            # Some apps expose out1 or ctrl_A; we try pidA_out first then out1
            pid_out = self.read_reg('pidA_out')
            # Convert 14-bit signed to volts: app uses 1/8192 scaling (see control scripts).
            v = float(pid_out) * (1.0 / 8192.0)
            self._last_ao1_pos_v = v
            return v
        except Exception:
            return self._last_ao1_pos_v

    def get_pid_status(self) -> Dict[str, int]:
        status = {}
        for reg in ('pidA_sw', 'pidA_sp', 'pidA_kp', 'pidA_ki', 'pidA_kd', 'pidA_out'):
            try:
                status[reg] = self.read_reg(reg)
            except Exception:
                status[reg] = 0
        return status