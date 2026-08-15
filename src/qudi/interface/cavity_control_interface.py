# -*- coding: utf-8 -*-
"""
Unified interface for cavity control operations exposed by the interfuse:
- Set alpha (attenuation between fine AO1 and coarse AO2)
- Set AO1 ramp parameters and osc settings
- Set/get coarse voltage
- Perform one ramp acquisition and return correlated data (V_eff, AI1)
- Convenience getters for AO1 position and constraints
"""

from abc import ABC, abstractmethod
from typing import Tuple, Any, Optional, Sequence, Dict


class CavityControlInterface(ABC):
    @abstractmethod
    def set_alpha(self, alpha: float) -> None:
        """Set attenuation factor alpha (AO1 contribution scaled by 1/alpha relative to AO2)."""
        raise NotImplementedError

    @abstractmethod
    def set_ao1_ramp_params(self,
                            low_lim_v: float,
                            high_lim_v: float,
                            speed_v_per_s: float,
                            direction_up: bool,
                            sawtooth: bool = False) -> None:
        """Set AO1 ramp limits, speed (V/s), direction, shape."""
        raise NotImplementedError

    @abstractmethod
    def set_osc_params(self,
                       decimation: int,
                       trigger_source: int,
                       trig_pos: int = 8191,
                       hysteresis: int = 1,
                       threshold: int = 0) -> None:
        """Set oscilloscope parameters."""
        raise NotImplementedError

    @abstractmethod
    def set_coarse_voltage(self, value: float) -> None:
        """Set coarse actuator voltage (V)."""
        raise NotImplementedError

    @abstractmethod
    def get_coarse_voltage(self) -> float:
        """Get coarse actuator voltage (V)."""
        raise NotImplementedError

    @abstractmethod
    def single_ramp_acquire(self) -> Tuple[Sequence[float], Sequence[float]]:
        """
        Perform a single AO1 ramp acquisition:
        - Configure routing so osc channel A records Ramp A (fine AO1 equivalent)
        - Configure osc channel B to record photodiode signal
        - Trigger acquisition, fetch (t, rampA, ai1), compute V_eff = V_AO2 + rampA/alpha
        Return (V_eff_array, AI1_array)
        """
        raise NotImplementedError

    @abstractmethod
    def get_ao1_position(self) -> float:
        """Return current AO1 position (V) if available; otherwise best-effort."""
        raise NotImplementedError

    @abstractmethod
    def get_constraints(self) -> Dict[str, Dict[str, float]]:
        """
        Return a dict of constraints for AO1 and AO2, e.g.:
        {'ao1': {'min': 0.0, 'max': 1.0, 'unit': 'V'}, 'ao2': {'min': -10.0, 'max': 10.0, 'unit': 'V'}}
        """
        raise NotImplementedError


class CoarseActuatorInterface(ABC):
    @abstractmethod
    def activate(self) -> None:
        """Prepare the coarse actuator hardware to accept set_voltage calls."""
        raise NotImplementedError

    @abstractmethod
    def deactivate(self) -> None:
        """Safely deactivate the coarse actuator (e.g., stop tasks, release resources)."""
        raise NotImplementedError

    @abstractmethod
    def set_voltage(self, value: float) -> None:
        """Set the coarse actuator voltage in volts (respect constraints)."""
        raise NotImplementedError

    @abstractmethod
    def get_voltage(self) -> float:
        """Return the last set coarse actuator voltage in volts."""
        raise NotImplementedError

    @abstractmethod
    def constraints(self) -> Dict[str, float]:
        """Return constraints (e.g., {'min': -10.0, 'max': 10.0, 'unit': 'V'})."""
        raise NotImplementedError

class RedPitayaLockInterface(ABC):
    @abstractmethod
    def connect(self) -> None:
        """Establish connection to the Red Pitaya (SSH and/or HTTP)."""
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the Red Pitaya."""
        raise NotImplementedError

    @abstractmethod
    def read_reg(self, name: str) -> int:
        """Read a lock module register by symbolic name."""
        raise NotImplementedError

    @abstractmethod
    def write_reg(self, name: str, value: int) -> None:
        """Write a lock module register by symbolic name."""
        raise NotImplementedError

    # Ramp/scan
    @abstractmethod
    def configure_ramp(self,
                       low_lim: int,
                       high_lim: int,
                       step_ticks: int,
                       direction_up: bool,
                       sawtooth: bool = False) -> None:
        """Configure ramp limits, period (step ticks), direction, and shape."""
        raise NotImplementedError

    @abstractmethod
    def enable_ramp(self, enable: bool) -> None:
        """Enable or disable the ramp generator."""
        raise NotImplementedError

    @abstractmethod
    def reset_ramp(self) -> None:
        """Reset ramp state/counters."""
        raise NotImplementedError

    # Output routing
    @abstractmethod
    def set_out_mux(self, out1_sel: int, out2_sel: int) -> None:
        """Select sources for DAC OUT1 and OUT2 via out1_sw and out2_sw."""
        raise NotImplementedError

    # Oscilloscope
    @abstractmethod
    def osc_config(self,
                   decimation: int,
                   trigger_source: int,
                   trig_pos: int = 8191,
                   hysteresis: int = 1,
                   threshold: int = 0) -> None:
        """Configure oscilloscope acquisition."""
        raise NotImplementedError

    @abstractmethod
    def osc_acquire(self, wait: bool = True) -> None:
        """Trigger a single-shot oscilloscope acquisition."""
        raise NotImplementedError

    @abstractmethod
    def osc_curves(self, raw: bool = False) -> Tuple[Sequence[float], Sequence[float], Sequence[float]]:
        """Return (t, chA, chB) arrays of the last acquisition (volts by default)."""
        raise NotImplementedError

    # Convenience
    @abstractmethod
    def get_ao1_position(self) -> Optional[float]:
        """Return the current AO1 position (volts) if available."""
        raise NotImplementedError

    @abstractmethod
    def get_pid_status(self) -> Dict[str, Any]:
        """Return a snapshot of relevant PID status registers."""
        raise NotImplementedError