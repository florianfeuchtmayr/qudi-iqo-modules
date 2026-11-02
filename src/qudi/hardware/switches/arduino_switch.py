# -*- coding: utf-8 -*-
"""
Dummy implementation for switching interface.

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

from qudi.interface.switch_interface import SwitchInterface
from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
import serial as pyserial
import time

class ArduinoSwitch(SwitchInterface):
    """ Methods to control slow switching devices.

    arduinoswitch:
        module.Class: 'switches.arduino_switch.ArduinoSwitch'
        name: 'Fast/Slowcounter Switch'
        serial_address: 'ASRL5::INSTR'
        serial_baudrate: 9600
        switches:
            Detector_QuPhots: ['Slowcounter', 'Fastcounter']
            Detector_CC: ['Slowcounter', 'Fastcounter']
    """

    # ConfigOptions
    # customize available switches in config. Each switch needs a tuple of at least 2 state names.
    _serial_port = ConfigOption('serial_port', missing='error')
    _serial_baudrate = ConfigOption(name='serial_baudrate', default=9600, missing='warn')
    _hardware_name = ConfigOption(name='name', default=None, missing='nothing')
    _switches = ConfigOption(name='switches', missing='error')

    _states = StatusVar(name='states', default=None)

    def on_activate(self):
        """ Activate the module and fill status variables.
        """
        """ Prepare module, connect to hardware.
                """
        try:
            self._instrument = pyserial.Serial(
                port=self._serial_port,
                baudrate=self._serial_baudrate,
                timeout=500
            )
            self._states = {switch: states[0] for switch, states in self._switches.items()}
            time.sleep(1)
        except Exception as e:
            self.log.exception("Check Arduino connection and settings.")
            print(e)
            return False
        else:
            return True

    def on_deactivate(self):
        """ Deactivate the module and clean up.
        """
        self._instrument.close()
        time.sleep(1)

    @property
    def name(self):
        """ Name of the hardware as string.

        @return str: The name of the hardware
        """
        return self._hardware_name

    @property
    def available_states(self):
        """ Names of the states as a dict of tuples.

        The keys contain the names for each of the switches. The values are tuples of strings
        representing the ordered names of available states for each switch.

        @return dict: Available states per switch in the form {"switch": ("state1", "state2")}
        """
        return self._switches.copy()

    def get_state(self, switch):
        """ Query state of single switch by name

        @param str switch: name of the switch to query the state for
        @return str: The current switch state
        """
        assert switch in self.available_states, f'Invalid switch name: "{switch}"'
        return self._states[switch]

    def set_state(self, switch, state):
        """ Query state of single switch by name

        @param str switch: name of the switch to change
        @param str state: name of the state to set
        """
        avail_states = self.available_states
        assert switch in avail_states, f'Invalid switch name: "{switch}"'
        assert state in avail_states[switch], f'Invalid state name "{state}" for switch "{switch}"'
        try:
            self._states[switch] = state
            # The first number notes the number of the servo and the second number notes the state

            if switch == "Powermeter":
                if state == 'Out':
                    self._instrument.write(bytes('10', 'utf-8'))
                    #time.sleep(0.01)
                else:
                    self._instrument.write(bytearray('11', 'utf-8'))
                    #time.sleep(0.01)
            if switch == "Green Laser":
                if state == 'Off':
                    self._instrument.write(bytes('21', 'utf-8'))
                    #time.sleep(0.01)
                else:
                    self._instrument.write(bytearray('20', 'utf-8'))
                    #time.sleep(0.01)

            if switch == "Red Laser":
                if state == 'Off':
                    self._instrument.write(bytes('30', 'utf-8'))
                    #time.sleep(0.01)
                else:
                    self._instrument.write(bytearray('31', 'utf-8'))
                    #time.sleep(0.01)

            if switch == "Spectrometer":
                if state == 'Out':
                    self._instrument.write(bytes('41', 'utf-8'))
                    #time.sleep(0.01)
                else:
                    self._instrument.write(bytearray('40', 'utf-8'))
                    #time.sleep(0.01)
            else:
                pass

            if switch == "ND filter":
                if state == 'In':
                    self._instrument.write(bytes('51', 'utf-8'))
                    #time.sleep(0.01)
                else:
                    self._instrument.write(bytearray('50', 'utf-8'))
                    #time.sleep(0.01)
            else:
                pass
        except:
            all()


    # def convert_state(self, ):
    #     """
    #
    #
    #     :return:
    #     """
    #     return str((self._switches[key].index(new_states[key])*1**(32-self._output_port[i]) for i, key in enumerate(self._switches))).zfill(32)