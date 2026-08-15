import paramiko
from scp import SCPClient

from plumbum.machines.paramiko_machine import paramiko
from qudi.core.configoption import ConfigOption
from qudi.core.module import Base

class RedPitaya(Base):

    _address = ConfigOption('address', default='', missing = 'error')
    _user = ConfigOption('user', default='root', missing = 'info')
    _password = ConfigOption('password', default='root', missing = 'info')

    _path_to_bitfile = ConfigOption('path_to_bitfile', default=None, missing = 'nothing')
    _filename_on_rp = ConfigOption('filename_on_rp', default='system_wrapper', missing = 'info')
    is_connected = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.is_connected = False
        self._ssh_client = None
        self._scp_client = None

    def connect_rp(self, address, user, password):
        self._ssh_client = paramiko.SSHClient()
        self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self._ssh_client.connect(hostname=address,
                                    username=user,
                                    password=password)
        except paramiko.SSHException as e:
            self.log.exception('Something went wrong in the SSH connection attempt: ', e)
            self.is_connected = False
            return
        self.is_connected = True
        self.log.info('Connected to RedPitaya @ ' + address)

    def disconnect_rp(self):
        if self.is_connected:
            self._ssh_client.close()
            self.is_connected = False
            self.log.info('Disconnected from RedPitaya @ ' + self._address)

    def on_activate (self):
        self.connect_rp(self._address, self._user, self._password)
        if self._path_to_bitfile is not None:
            self._program_rp(self._path_to_bitfile)

    def program_rp(self):
        if self.is_connected:
            self._scp_client = SCPClient(self._ssh_client.get_transport())
            self._scp_client.put(self._path_to_bitfile, remote_path=f'/root/{self._filename_on_rp}.bit')
            self._scp_client.close()
            self._write_command(f'cat {self._filename_on_rp}.bit > /dev/xdevcfg')
        else:
            self.log.warning('No open connection to RP!')

    def write_command(self, command):
        if self.is_connected:
            try:
                stdin, stdout, stderr = self._ssh_client.exec_command(command)
                return stdout.read()
            except paramiko.SSHException as e:
                self.log.exception('Something went wrong trying to write the command: ', e)
        else:
            self.log.warning('No open connection to RP!')
            return None

    def set_register(self, address, value):
        command = f'monitor {address} {value}'
        self._write_command(command)

    def get_register(self, address):
        command = f'monitor {address}'
        output = self._write_command(command)
        if output is not None:
            return int(output)
        else:
            return None

    def on_deactivate(self):
        self.disconnect_rp()