# -*- coding: utf-8 -*-
"""
TimeTagger fast counter for Qudi pulsed measurements (local USB or network).

- Local:  tt.createTimeTagger()
- Network: TimeTagger.createTimeTaggerNetwork('host:port')

Implements FastCounterInterface for use with pulsed_measurement_logic.

Config example:
    fastcounter_timetagger:
        module.Class: 'swabian_instruments.timetagger_fast_counter.TimeTaggerFastCounter'
        options:
            network: True                 # set False for local USB
            address: '134.60.31.152:5353'
            timetagger_serial: ''         # optional, for local USB selection
            timetagger_resolution: 'Standard'  # optional
            timetagger_channel_apd_0: 0
            timetagger_channel_apd_1: 1
            timetagger_channel_detect: 2
            timetagger_channel_sequence: 3
            timetagger_sum_channels: true # sum APD0+APD1 (virtual channel)
            trigger_level_detect_v: 0.25  # optional, volts
            trigger_level_apd_v: 0.05     # optional, volts

"""

import numpy as np
import TimeTagger as tt

from qudi.interface.fast_counter_interface import FastCounterInterface
from qudi.core.configoption import ConfigOption


class TimeTaggerFastCounter(FastCounterInterface):
    # Connection options
    _network: bool = ConfigOption('network', default=False, missing='warn')
    _address: str = ConfigOption('address', default='', missing='warn')
    _timetagger_serial: str = ConfigOption('timetagger_serial', default='', missing='warn')
    _timetagger_resolution: str = ConfigOption('timetagger_resolution', default='Standard', missing='warn')

    # Channel mapping
    _channel_apd_0: int = ConfigOption('timetagger_channel_apd_0', missing='error')
    _channel_apd_1: int = ConfigOption('timetagger_channel_apd_1', missing='error')
    _channel_detect: int = ConfigOption('timetagger_channel_detect', missing='error')
    _channel_sequence: int = ConfigOption('timetagger_channel_sequence', missing='error')

    # Sum APD channels (virtual combiner)
    _sum_channels: bool = ConfigOption('timetagger_sum_channels', default=True, missing='warn',
                                       constructor=lambda v: bool(v))

    # Optional trigger levels (volts)
    _trig_level_detect_v: float = ConfigOption('trigger_level_detect_v', default=None, missing='nothing')
    _trig_level_apd_v: float = ConfigOption('trigger_level_apd_v', default=None, missing='nothing')

    def on_activate(self):
        """Create/attach to the TimeTagger (local or network) and prepare channels."""
        # Create tagger
        if self._network:
            if not self._address:
                raise ValueError('network=True but no "address" provided (expected "host:port").')
            self._tagger = tt.createTimeTaggerNetwork(self._address)
        else:
            # Optional: honor specific serial if present, otherwise default
            self._tagger = tt.createTimeTagger(self._timetagger_serial) if self._timetagger_serial \
                else tt.createTimeTagger()

        # Reset to a known state
        try:
            self._tagger.reset()
        except Exception:
            # Some network wrappers may not expose reset; safe to ignore if unavailable
            pass

        # Build APD input (optionally summed)
        if self._sum_channels:
            comb = tt.Combiner(self._tagger, channels=[self._channel_apd_0, self._channel_apd_1])
            self._channel_apd = comb.getChannel()
            self._combiner = comb
        else:
            self._combiner = None
            self._channel_apd = self._channel_apd_0

        # Optional trigger levels (set only if provided)
        try:
            if self._trig_level_detect_v is not None:
                self._tagger.setTriggerLevel(self._channel_detect, float(self._trig_level_detect_v))
            if self._trig_level_apd_v is not None:
                # Apply to both raw APD channels if available; also to the combined virtual channel if supported
                self._tagger.setTriggerLevel(self._channel_apd_0, float(self._trig_level_apd_v))
                try:
                    self._tagger.setTriggerLevel(self._channel_apd_1, float(self._trig_level_apd_v))
                except Exception:
                    pass
                try:
                    # some backends may not accept setting on virtual channel
                    self._tagger.setTriggerLevel(self._channel_apd, float(self._trig_level_apd_v))
                except Exception:
                    pass
        except Exception as e:
            self.log.warning(f'Failed to set trigger levels on TimeTagger: {e}')

        # Defaults
        self._number_of_gates = 1
        self._bin_width_s = 1e-9
        self._record_length_s = 4e-6
        self._pulsed = None
        self.statusvar = 0  # 0=unconfigured

        self.log.info(f'TimeTagger fast counter ready (network={self._network}, '
                      f'address="{self._address}" if network).')
        self.load_path = None

    def on_deactivate(self):
        """Stop/clear measurement and free the tagger."""
        try:
            if self.module_state() == 'locked':
                self.stop_measure()
        except Exception:
            pass
        # Clear measurement objects
        try:
            if self._pulsed is not None:
                try:
                    self._pulsed.stop()
                except Exception:
                    pass
                try:
                    self._pulsed.clear()
                except Exception:
                    pass
                self._pulsed = None
        except Exception:
            pass
        # Free the tagger
        try:
            tt.freeTimeTagger(self._tagger)
        except Exception:
            pass
        finally:
            self._tagger = None

    # ---------- FastCounterInterface implementation ----------

    def get_constraints(self):
        """Return supported hardware bin widths (seconds per bin)."""
        # Keep conservative defaults; pulsed logic can rebin in software if needed
        return {
            'hardware_binwidth_list': [
                1000e-9,       # 1000 ns
                500e-9,        # 500 ns
                200e-9,        # 200 ns
                100e-9,        # 100 ns
                50e-9,         # 50 ns
                20e-9,         # 20 ns
                10e-9,         # 10 ns
                5e-9,          # 5 ns
                2e-9,          # 2 ns
                1.0 / 1000e6,  # 1 ns
                0.5e-9,        # 0.5 ns
                0.2e-9,        # 0.2 ns
                0.1e-9         # 0.1 ns
            ]
        }

    def configure(self, bin_width_s, record_length_s, number_of_gates=0):
        """
        Configure TimeTagger.TimeDifferences for pulsed histograms.

        Parameters:
            bin_width_s: float (seconds per bin)
            record_length_s: float (seconds per histogram window)
            number_of_gates: int (pulsed logic "n_histograms")

        Returns:
            (bin_width_s, record_length_s, number_of_gates)
        """
        # Store settings
        self._number_of_gates = int(max(1, number_of_gates))
        self._bin_width_s = float(bin_width_s)
        self._record_length_s = float(record_length_s)

        # Convert to TimeTagger units:
        # TimeDifferences wants binwidth in ps and n_bins as integer
        binwidth_ps = int(np.round(self._bin_width_s * 1e12))
        n_bins = int(1 + np.floor(self._record_length_s / self._bin_width_s))

        # Create/replace measurement
        if self._pulsed is not None:
            try:
                self._pulsed.stop()
            except Exception:
                pass
            try:
                self._pulsed.clear()
            except Exception:
                pass
            self._pulsed = None

        self._pulsed = tt.TimeDifferences(
            tagger=self._tagger,
            click_channel=self._channel_apd,
            start_channel=self._channel_detect,
            next_channel=self._channel_detect,      # standard pulsed timing
            sync_channel=tt.CHANNEL_UNUSED,         # unused here
            binwidth=binwidth_ps,
            n_bins=n_bins,
            n_histograms=self._number_of_gates
        )
        # Ensure measurement is in a known state
        try:
            self._pulsed.stop()
        except Exception:
            pass

        self.statusvar = 1  # idle
        return self._bin_width_s, self._record_length_s, self._number_of_gates

    def start_measure(self):
        """Start pulsed counting."""
        self.module_state.lock()
        if self._pulsed is None:
            raise RuntimeError('TimeTaggerFastCounter not configured.')
        try:
            self._pulsed.clear()
        except Exception:
            pass
        self._pulsed.start()
        self.statusvar = 2  # running
        return 0

    def stop_measure(self):
        """Stop pulsed counting."""
        if self.module_state() == 'locked':
            try:
                if self._pulsed is not None:
                    self._pulsed.stop()
            finally:
                self.module_state.unlock()
        self.statusvar = 1  # idle
        return 0

    def pause_measure(self):
        """Pause pulsed counting."""
        if self.module_state() == 'locked':
            try:
                if self._pulsed is not None:
                    self._pulsed.stop()
            finally:
                self.statusvar = 3  # paused
        return 0

    def continue_measure(self):
        """Continue pulsed counting after pause."""
        if self.module_state() == 'locked':
            if self._pulsed is None:
                raise RuntimeError('TimeTaggerFastCounter not configured.')
            self._pulsed.start()
            self.statusvar = 2
        return 0

    def is_gated(self):
        """This counter is used in gated/pulsed mode."""
        return False

    def get_data_trace(self):
        """
        Return histogram data for all gates as int64 array and info dict:
            shape = [n_histograms, n_bins]
        """
        info_dict = {'elapsed_sweeps': None, 'elapsed_time': None}
        if self.load_path is not None:
            return np.genfromtxt(self.load_path), info_dict
        else:
            return np.array(self._pulsed.getData(), dtype='int64')[0], info_dict

    def get_status(self):
        """0=unconfigured, 1=idle, 2=running, 3=paused, -1=error."""
        return self.statusvar

    def get_binwidth(self):
        """Return current bin width (seconds)."""
        return float(self._bin_width_s)