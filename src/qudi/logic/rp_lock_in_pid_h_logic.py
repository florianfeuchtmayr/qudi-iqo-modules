import numpy as np

from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.core.module import LogicBase, Base

class RPLockInPIDHLogic(LogicBase):

    # declare connector
    _rp = Connector(name = 'redpitaya', interface = Base)

    # status vars
    registers = StatusVar('registers', default={
        'oscA_sw': 1,
        'oscB_sw': 4,
        'trig_sw': 2,
        'ramp_low_lim': -8191,   # -8191
        'ramp_hig_lim': 8191,   # 8191
        'ramp_step': 126,  # -1
        'ramp_enable': 1,
        'gen_mod_hp': 8,
        'gen_mod_phase': 650,
        'lpf_F1': 33,
        'sg_amp1': 3,
        'mod_out1': 10,
        'error_sw': 3,
        'pidA_kp': -21,
        'pidA_ki': -3,
        'pidA_kd': 0,
        'lock_control': 1360
    })

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.threadlock = Mutex()

        # intialize attributes
        self.rp = None

    def on_activate(self) -> None:
        self.rp = self._rp()
        if self.rp.is_connected:
            # set registers to stored values
            for reg, val in self.registers.items():
                self.set_lock_register(reg, val)

    def set_lock_register(self, register, value):
        with self.threadlock:
            if self.rp.is_connected:
                command = f'/opt/redpitaya/www/apps/lock_in+pid_harmonic/c/lock {register} {value}'
                self.rp.write_command(command)
            else:
                self.log.warning('No open connection to RP!')

    def get_lock_register(self, register):
        with self.threadlock:
            if self.rp.is_connected:
                command = f'/opt/redpitaya/www/apps/lock_in+pid_harmonic/c/lock {register}'
                output = self.rp.write_command(command)
                if output is None:
                    self.log.error(f'No output from RP for register {register}')
                    return None
                output = output.strip()
                parts = output.split(b':', 1)
                val_str = parts[1].strip() if len(parts) > 1 else parts[0].strip()
                try:
                    value = int(val_str)
                    return value
                except ValueError:
                    self.log.error(f'Could not convert output to int: {output}')
                    return None
            else:
                self.log.warning('No open connection to RP!')
                return None

    def on_deactivate(self):
        if self.rp.is_connected:
            # set registers to stored values
            for reg in self.registers.keys():
                self.registers[reg] = self.get_lock_register(reg)
