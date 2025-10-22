# -*- coding: utf-8 -*-

"""
Attocube AMC300 stepper-based scanning probe hardware for Qudi.

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
from typing import Dict, List, Optional

from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.interface.scanning_probe_interface import (
    ScanningProbeInterface,
    ScanConstraints,
    ScannerAxis,
    ScannerChannel,
    ScanSettings,
    ScanData,
    BackScanCapability,
)
from qudi.util.constraints import ScalarConstraint

#from AMC_API import AMC

class AMC300_stepper(ScanningProbeInterface):
    """
    Implements ScanningProbeInterface for motion only (stepping; no analog fine output).
    Units:
    - Qudi external API uses meters.
    - AMC300 API uses nanometers for linear axes. We convert m <-> nm.

    Key Python API calls (from AMC Interface Manual):
        import AMC
        dev = AMC.Device(ip)
        dev.connect()
        dev.close()

        # Motion
        dev.move.setNSteps(axis, backward, step)
        dev.move.setSingleStep(axis, backward)
        dev.move.getPosition(axis) -> position_nm
        dev.status.getStatusMoving(axis) -> 0 idle, 1 moving, 2 pending
        dev.control.setControlOutput(axis, enable)

    Example Config:

    amc300_stepper:
        module.Class: 'attocube.AMC300_stepper.AMC300_stepper'
        options:
            ip_address: '192.168.1.1'
            axis_map: { x: 0, y: 1, z: 2 }
            step_size_m: { x: 2e-7, y: 2e-7, z: 2e-7 }   # meters per step
            position_ranges:
                x: [1.5e-3, 4.5e-3]
                y: [1.5e-3, 4.5e-3]
                z: [1.5e-3, 4.5e-3]
            frequency_ranges:
                x: [1, 500]
                y: [1, 500]
                z: [1, 100]
            resolution_ranges:
                x: [1, 100000]
                y: [1, 100000]
                z: [1, 100000]
            input_channel_units:
                APD: 'c/s'
            drive_enable_on_activate: false
            settle_time_s: 0.001
            max_move_timeout_s: 5.0

    """


    _threaded = True

    # Connection/config
    _ip_address: str = ConfigOption('ip_address', default='127.0.0.1', missing='error')

    # Axis mapping and step sizes
    _axis_map: Dict[str, int] = ConfigOption('axis_map', default={'x': 0, 'y': 1, 'z': 2}, missing='warn')
    _step_size_m: Dict[str, float] = ConfigOption('step_size_m', missing='error')  # meters/step

    # Constraints
    _position_ranges: Dict[str, List[float]] = ConfigOption('position_ranges', missing='error')
    _frequency_ranges: Dict[str, List[float]] = ConfigOption('frequency_ranges', default={}, missing='warn')
    _resolution_ranges: Dict[str, List[int]] = ConfigOption('resolution_ranges', default={}, missing='warn')
    _input_channel_units: Dict[str, str] = ConfigOption('input_channel_units', default={}, missing='warn')

    # Behavior
    _drive_enable_on_activate: bool = ConfigOption('drive_enable_on_activate', default=True, missing='warn')
    _settle_time_s: float = ConfigOption('settle_time_s', default=0.001, missing='warn')
    _max_move_timeout_s: float = ConfigOption('max_move_timeout_s', default=5.0, missing='warn')
    _force_single_steps: bool = ConfigOption('force_single_steps', default=False, missing='nothing')
    _reset_frontpanel_nsteps_after_move: bool = ConfigOption(
        'reset_frontpanel_nsteps_after_move', default=False, missing='nothing'
    )

    sigPositionChanged = QtCore.Signal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = Mutex()
        self._dev = None  # AMC.Device instance
        self._target_m: Dict[str, float] = {ax: 0.0 for ax in self._axis_map}

    # Lifecycle
    def on_activate(self):

        # Sanity check: require matching axes across ranges
        assert set(self._position_ranges) == set(self._frequency_ranges) == set(self._resolution_ranges), \
            f'Channels in position ranges, frequency ranges and resolution ranges do not coincide'

        # Connect to AMC before building constraints so we can seed defaults with actual position
        try:
            from qudi.hardware.attocube.AMC_API import AMC  # import AMC  # Provided by Attocube
        except Exception as exc:
            raise RuntimeError('AMC300_stepper: Could not import AMC Python package') from exc

        with self._thread_lock:
            self._dev = AMC.Device(self._ip_address)
            self._dev.connect()

            # Optionally enable drives
            if self._drive_enable_on_activate:
                for ax, ch in self._axis_map.items():
                    try:
                        self._dev.control.setControlOutput(ch, True)
                    except Exception:
                        # Some axes or configs may fail here; log but continue
                        self.log.warning(f'AMC: setControlOutput failed for axis {ax}')

            # Read current physical position for each axis (no movement)
            current_pos: Dict[str, float] = {}
            for ax, ch in self._axis_map.items():
                try:
                    pos_nm = float(self._dev.move.getPosition(ch))
                    pos_m = pos_nm * 1e-9
                except Exception:
                    # Fallback to lower bound if reading fails
                    pos_m = float(self._position_ranges[ax][0])
                # Clip to configured bounds
                rng = self._position_ranges[ax]
                pos_m = min(max(pos_m, float(rng[0])), float(rng[1]))
                current_pos[ax] = pos_m

            # Set the internal target to the measured current position (no movement)
            self._target_m = dict(current_pos)

        # Build constraints AFTER we know the current position to use as default
        axes = list()
        for axis in self._position_ranges:
            position_range = tuple(self._position_ranges[axis])
            resolution_range = tuple(self._resolution_ranges[axis])
            res_default = 50
            if not resolution_range[0] <= res_default <= resolution_range[1]:
                res_default = resolution_range[0]
            frequency_range = tuple(self._frequency_ranges[axis])
            freq_default = 500
            if not frequency_range[0] <= freq_default <= frequency_range[1]:
                freq_default = frequency_range[0]
            max_step = abs(position_range[1] - position_range[0])

            # Use the measured current position as the default
            pos_default = float(self._target_m.get(axis, position_range[0]))

            position = ScalarConstraint(default=pos_default, bounds=position_range)
            resolution = ScalarConstraint(default=res_default, bounds=resolution_range, enforce_int=True)
            frequency = ScalarConstraint(default=freq_default, bounds=frequency_range)
            step = ScalarConstraint(default=0, bounds=(0, max_step))

            axes.append(ScannerAxis(name=axis,
                                    unit='m',
                                    position=position,
                                    step=step,
                                    resolution=resolution,
                                    frequency=frequency)
                        )

        channels = list()
        for channel, unit in self._input_channel_units.items():
            channels.append(ScannerChannel(name=channel,
                                           unit=unit,
                                           dtype='float64'))

        back_scan_capability = BackScanCapability.AVAILABLE | BackScanCapability.RESOLUTION_CONFIGURABLE
        self._constraints = ScanConstraints(axis_objects=tuple(axes),
                                            channel_objects=tuple(channels),
                                            back_scan_capability=back_scan_capability,
                                            has_position_feedback=False,
                                            square_px_only=False)

        # Notify listeners about the current position/target so GUIs can place the cursor immediately
        try:
            self.sigPositionChanged.emit(dict(self._target_m))
        except Exception:
            pass

    def on_deactivate(self):
        """
        #Possible to disable channels
        with self._thread_lock:
            try:
                if self._dev is not None:
                    # Try to stop outputs gracefully (optional)
                    for ax, ch in self._axis_map.items():
                        try:
                            self._dev.control.setControlOutput(ch, True)
                        except Exception:
                            pass
            finally:
                try:
                    if self._dev is not None:
                        self._dev.close()
                except Exception:
                    pass
                self._dev = None
        """
        return

    # ScanningProbeInterface: constraints and configuration
    @property
    def constraints(self) -> ScanConstraints:
        """ Read-only property returning the constraints of this scanning probe hardware.
        """
        return self._constraints

    def reset(self) -> None:
        # Stop outputs if possible
        with self._thread_lock:
            if self._dev is not None:
                for ax, ch in self._axis_map.items():
                    try:
                        self._dev.control.setControlOutput(ch, False)
                    except Exception:
                        pass

    @property
    def scan_settings(self) -> Optional[ScanSettings]:
        return None

    @property
    def back_scan_settings(self) -> Optional[ScanSettings]:
        return None

    def configure_scan(self, settings: ScanSettings) -> None:
        # Motion-only module does not perform scans by itself
        raise RuntimeError('AMC300_stepper is motion-only. Use AMC300NIScanningProbeInterfuse for scanning.')

    def configure_back_scan(self, settings: ScanSettings) -> None:
        return

    # Movement
    def move_absolute(self, position: Dict[str, float], velocity: Optional[float] = None,
                      blocking: bool = False) -> Dict[str, float]:
        with self._thread_lock:
            dev = self._require_dev()
            for ax, target_m in position.items():
                ch = self._axis_to_channel(ax)
                step_m = float(self._step_size_m[ax])
                # Current position from device in meters (fallback to target)
                try:
                    pos_nm = float(dev.move.getPosition(ch))
                    pos_m = pos_nm * 1e-9
                except Exception:
                    pos_m = self._target_m.get(ax, 0.0)
                # Clip and compute steps
                tgt_m = self._clip(ax, float(target_m))
                delta = tgt_m - pos_m
                n_steps = int(round(delta / step_m))
                if n_steps == 0:
                    self._target_m[ax] = pos_m
                    continue
                backward = True if n_steps < 0 else False
                steps = abs(n_steps)
                # Try bulk step; fallback to single-step loop
                if self._force_single_steps:
                    # Execute exactly 'steps' single steps
                    for _ in range(steps):
                        dev.move.setSingleStep(ch, bool(backward))
                else:
                    # Existing behavior: try bulk N-steps, fall back to single steps if unavailable
                    try:
                        dev.move.setNSteps(ch, bool(backward), int(steps))
                    except Exception:
                        for _ in range(steps):
                            dev.move.setSingleStep(ch, bool(backward))

                self._target_m[ax] = pos_m + n_steps * step_m
                if self._reset_frontpanel_nsteps_after_move:
                    try:
                        dev.move.writeNSteps(ch, 1)
                    except Exception:
                        pass

        if blocking:
            for ax in position.keys():
                self._wait_axis_idle(ax, self._max_move_timeout_s)
            time.sleep(self._settle_time_s)

        self.sigPositionChanged.emit(dict(self._target_m))
        return dict(self._target_m)

    def move_relative(self, distance: Dict[str, float], velocity: Optional[float] = None,
                      blocking: bool = False) -> Dict[str, float]:
        with self._thread_lock:
            curr = self.get_target()
        absolute = {ax: curr.get(ax, 0.0) + float(d) for ax, d in distance.items()}
        return self.move_absolute(absolute, velocity=velocity, blocking=blocking)

    def get_target(self) -> Dict[str, float]:
        with self._thread_lock:
            return dict(self._target_m)

    def get_position(self) -> Dict[str, float]:
        with self._thread_lock:
            dev = self._require_dev()
            pos = {}
            for ax, ch in self._axis_map.items():
                try:
                    nm = float(dev.move.getPosition(ch))
                    pos[ax] = nm * 1e-9
                except Exception:
                    pos[ax] = self._target_m.get(ax, 0.0)
            return pos

    # Controller closed-loop move with a single window parameter in nm
    def move_absolute_closed_loop(
            self,
            position: Dict[str, float],
            *,
            window_nm: int,
            timeout_s: float = 1.5,
            disable_after: bool = True,
            enable_output: bool = False,
            poll_interval_s: float = 0.02,
    ) -> Dict[str, float]:
        """
        Use AMC controller closed-loop to move to target(s) and stop when within window_nm.
        - window_nm is used for BOTH:
          • control.setControlTargetRange(axis, window_nm)        [in-target flag window]
          • control.setMotionControlThreshold(axis, window_nm*1e3)[pm threshold]
        - Will wait until status.getStatusTargetRange(axis) is true or timeout_s elapses.
        - Optionally disables closed-loop afterwards.

        Returns the updated target dictionary in meters.
        """
        with self._thread_lock:
            dev = self._require_dev()
            # Pre-configure each requested axis and command target
            for ax, tgt_m in position.items():
                ch = self._axis_to_channel(ax)

                # Enable output (relay) if requested
                if enable_output:
                    try:
                        dev.control.setControlOutput(ch, True)
                    except Exception:
                        self.log.warning(f'AMC: setControlOutput failed for axis {ax}')

                # Set controller windows (range in nm; threshold in pm)
                try:
                    dev.control.setControlTargetRange(ch, int(window_nm))
                except Exception:
                    self.log.warning(f'AMC: setControlTargetRange failed for axis {ax}')
                try:
                    dev.control.setMotionControlThreshold(ch, int(window_nm) * 1000)
                except Exception:
                    self.log.warning(f'AMC: setMotionControlThreshold failed for axis {ax}')

                # Enable closed-loop approach
                try:
                    dev.control.setControlMove(ch, True)
                except Exception:
                    self.log.warning(f'AMC: setControlMove(True) failed for axis {ax}')

                # Command absolute target in nm
                tgt_m = self._clip(ax, float(tgt_m))
                tgt_nm = float(tgt_m * 1e9)
                try:
                    dev.move.setControlTargetPosition(ch, tgt_nm)
                except Exception as exc:
                    raise RuntimeError(f'AMC: setControlTargetPosition failed for axis {ax}') from exc

                # Update local target immediately (avoid UI snap-back)
                self._target_m[ax] = tgt_m

        # Wait for all axes to be in target range or timeout
        t0 = time.time()
        axes = list(position.keys())
        while True:
            all_in = True
            with self._thread_lock:
                try:
                    for ax in axes:
                        ch = self._axis_to_channel(ax)
                        in_range = bool(self._dev.status.getStatusTargetRange(ch))  # type: ignore
                        if not in_range:
                            all_in = False
                            break
                except Exception:
                    # If status is unavailable, break on timeout using settle time as fallback
                    pass
            if all_in:
                break
            if time.time() - t0 > float(timeout_s):
                self.log.debug('AMC closed-loop move: timeout waiting for target range.')
                break
            time.sleep(float(poll_interval_s))

        # Optionally disable controller closed-loop
        if disable_after:
            with self._thread_lock:
                for ax in axes:
                    ch = self._axis_to_channel(ax)
                    try:
                        self._dev.control.setControlMove(ch, False)  # type: ignore
                    except Exception:
                        pass

        if self._reset_frontpanel_nsteps_after_move:
            with self._thread_lock:
                dev = self._require_dev()
                for ax in axes:
                    ch = self._axis_to_channel(ax)
                    try:
                        dev.move.writeNSteps(ch, 1)
                    except Exception:
                        pass

        # Notify listeners and return
        try:
            self.sigPositionChanged.emit(dict(self._target_m))
        except Exception:
            pass
        return dict(self._target_m)

    # Scan lifecycle (not used here)
    def start_scan(self) -> None:
        raise RuntimeError('AMC300_stepper does not implement scanning.')

    def stop_scan(self) -> None:
        return

    def get_scan_data(self) -> Optional[ScanData]:
        return None

    def get_back_scan_data(self) -> Optional[ScanData]:
        return None

    def emergency_stop(self) -> None:
        with self._thread_lock:
            if self._dev is None:
                return
            for ax, ch in self._axis_map.items():
                try:
                    # No dedicated "stop" method; disable output as a soft stop
                    self._dev.control.setControlOutput(ch, False)
                except Exception:
                    pass

    # Helpers
    def _require_dev(self):
        if self._dev is None:
            raise RuntimeError('AMC device not connected')
        return self._dev

    def _axis_to_channel(self, axis: str) -> int:
        if axis not in self._axis_map:
            raise KeyError(f'Unknown axis "{axis}"')
        return int(self._axis_map[axis])

    def _clip(self, axis: str, value: float) -> float:
        rng = self._position_ranges.get(axis)
        if not rng or len(rng) < 2:
            return value
        return min(max(value, float(rng[0])), float(rng[1]))

    def _wait_axis_idle(self, axis: str, timeout_s: float):
        ch = self._axis_to_channel(axis)
        t0 = time.time()
        while True:
            try:
                status = int(self._dev.status.getStatusMoving(ch))  # type: ignore
                moving = (status != 0)
            except Exception:
                # Fallback: assume settle_time sufficient
                time.sleep(self._settle_time_s)
                return
            if not moving:
                return
            if time.time() - t0 > timeout_s:
                self.log.warning(f'AMC axis {axis}: wait idle timeout.')
                return
            time.sleep(0.002)

    def set_step_size_m(self, axis: str, step_size_m: float) -> None:
        """Override the configured step size for an axis (meters per step)."""
        with self._thread_lock:
            if axis not in self._step_size_m:
                raise KeyError(f'Unknown axis "{axis}"')
            self._step_size_m[axis] = float(step_size_m)

    def get_step_size_m(self, axis: str) -> float:
        """Return the current (possibly overridden) step size for an axis (meters per step)."""
        with self._thread_lock:
            if axis not in self._step_size_m:
                raise KeyError(f'Unknown axis "{axis}"')
            return float(self._step_size_m[axis])

    def calibration_movement(
        self,
        *,
        axis: str,
        start_m: float,
        end_m: float,
        window_nm: int = 200,
        max_steps: Optional[int] = None,
        timeout_s: float = 30.0,
        poll_interval_s: float = 0.002,
    ) -> Dict[str, float]:
        """
        Calibrate step size on a single axis by stepping from start_m to end_m while monitoring position.
        Returns a dict {axis: measured_step_size_m}.
        """
        ch = self._axis_to_channel(axis)
        window_m = float(window_nm) * 1e-9
        with self._thread_lock:
            dev = self._require_dev()

        # 1) Closed-loop to start position (disable_after=True to avoid fighting stepping)
        self.move_absolute_closed_loop({axis: float(start_m)}, window_nm=int(window_nm),
                                       timeout_s=float(timeout_s), disable_after=True)

        with self._thread_lock:
            dev = self._require_dev()
            # Read initial measured position
            try:
                pos_nm = float(dev.move.getPosition(ch))
                pos_m_start = pos_nm * 1e-9
            except Exception:
                # If reading fails, fall back to commanded start
                pos_m_start = float(start_m)

        # 2) Step towards end_m while counting steps
        direction_backward = end_m < pos_m_start
        step_count = 0
        t0 = time.time()
        pos_m = pos_m_start
        while abs(pos_m - float(end_m)) > window_m:
            with self._thread_lock:
                # Single step towards target
                dev.move.setSingleStep(ch, bool(direction_backward))
            step_count += 1

            # Optional limiters
            if max_steps is not None and step_count >= int(max_steps):
                break
            if time.time() - t0 > float(timeout_s):
                break

            time.sleep(float(poll_interval_s))
            with self._thread_lock:
                try:
                    pos_nm = float(dev.move.getPosition(ch))
                    pos_m = pos_nm * 1e-9
                except Exception:
                    # If read fails, continue a bit and re-check
                    pos_m = pos_m

        # Final measured position
        with self._thread_lock:
            try:
                pos_nm = float(dev.move.getPosition(ch))
                pos_m_end = pos_nm * 1e-9
            except Exception:
                pos_m_end = pos_m

        # 3) Compute measured step size (fallback to existing if no steps)
        measured_step = self.get_step_size_m(axis)
        if step_count > 0:
            delta = abs(pos_m_end - pos_m_start)
            if delta > 0:
                measured_step = delta / float(step_count)

        return {axis: float(measured_step)}

    def set_force_single_steps(self, enabled: bool) -> None:
        """
        Enable/disable forcing moves to use only single-step commands (never setNSteps).
        This helps keep the controller's 'Single step' UI at 1 and can improve reliability.
        Runtime setting (not persisted to config).
        """
        with self._thread_lock:
            self._force_single_steps = bool(enabled)
        try:
            self.log.info(f'force_single_steps set to {self._force_single_steps}')
        except Exception:
            pass

    def get_force_single_steps(self) -> bool:
        """
        Return whether single-step-only motion is currently forced.
        """
        # Read without lock is fine; if you prefer, wrap with self._thread_lock
        return bool(getattr(self, '_force_single_steps', False))

    def set_reset_frontpanel_nsteps_after_move(self, enabled: bool) -> None:
        """
        Enable/disable automatically resetting AMC front-panel 'Single step' count to 1
        after every move (open-loop and closed-loop).
        """
        with self._thread_lock:
            self._reset_frontpanel_nsteps_after_move = bool(enabled)
        try:
            self.log.info(f'reset_frontpanel_nsteps_after_move set to {self._reset_frontpanel_nsteps_after_move}')
        except Exception:
            pass

    def get_reset_frontpanel_nsteps_after_move(self) -> bool:
        """Return whether automatic reset of front-panel 'Single step' count is enabled."""
        return bool(getattr(self, '_reset_frontpanel_nsteps_after_move', False))

    def set_frontpanel_step_count(self, axis: str, steps: int) -> None:
        """
        Set the controller front-panel 'Single step' count for a given axis.
        Uses AMC.move.writeNSteps; affects manual stepping/UI only (PRO feature).
        """
        steps = int(steps)
        if steps <= 0:
            raise ValueError('steps must be a positive integer')
        with self._thread_lock:
            dev = self._require_dev()
            ch = self._axis_to_channel(axis)
            try:
                dev.move.writeNSteps(ch, steps)
            except Exception:
                self.log.warning('AMC: writeNSteps not available or failed.')

    def get_frontpanel_step_count(self, axis: str) -> int:
        """
        Read the controller front-panel 'Single step' count for a given axis via AMC.move.getNSteps.
        """
        with self._thread_lock:
            dev = self._require_dev()
            ch = self._axis_to_channel(axis)
            try:
                val = int(dev.move.getNSteps(ch))
            except Exception:
                self.log.warning('AMC: getNSteps not available or failed.')
                val = 1
        return val

    def reset_frontpanel_step_count(self, axis: Optional[str] = None) -> None:
        """
        Reset 'Single step' count to 1 on one axis or all axes (UI convenience).
        """
        with self._thread_lock:
            dev = self._require_dev()
            axes = [axis] if axis is not None else list(self._axis_map.keys())
            for ax in axes:
                ch = self._axis_to_channel(ax)
                try:
                    dev.move.writeNSteps(ch, 1)
                except Exception:
                    # Non‑PRO or older firmware may not support this; ignore
                    pass

    def calibrate_axis_step_size(
            self,
            *,
            axis: str,
            distance_m: float = 10e-6,
            window_nm: int = 600,
            timeout_s: float = 30.0,
            max_steps: Optional[int] = None,
            poll_interval_s: float = 0.002,
    ) -> float:
        """
        Perform a calibration movement on a single axis over 'distance_m' (default 10 µm),
        measure the effective step size, update self._step_size_m[axis], and return the new step size (m/step).

        Procedure:
          1) Read current device position on the axis.
          2) Choose a valid [start, end] segment within configured position range with length ~distance_m.
          3) Use calibration_movement(...) to step and measure.
          4) Update internal step size for the axis via set_step_size_m(axis, measured).

        Notes:
          - Uses controller closed-loop to approach the start, then steps open-loop as in scanning calibration.
          - Does not change scanning settings; this only updates the step size used by normal open-loop moves.
        """
        axis = str(axis)
        # Validate axis and read ranges
        if axis not in self._axis_map:
            raise KeyError(f'Unknown axis "{axis}"')
        if distance_m <= 0:
            raise ValueError('distance_m must be > 0')

        with self._thread_lock:
            dev = self._require_dev()
            ch = self._axis_to_channel(axis)
            rng = self._position_ranges.get(axis, None)
            if not rng or len(rng) < 2:
                raise RuntimeError(f'No valid position range for axis "{axis}"')
            lo, hi = float(rng[0]), float(rng[1])

            # Current measured position (fallback to target)
            try:
                pos_nm = float(dev.move.getPosition(ch))
                curr_m = pos_nm * 1e-9
            except Exception:
                curr_m = self._target_m.get(axis, (lo + hi) * 0.5)

        # Choose a calibration segment near current position; clamp to range
        seg = float(distance_m)
        total_span = hi - lo
        if seg > total_span:
            # If requested distance exceeds range, reduce to full span
            seg = total_span
        # Try to center around current position
        start_m = max(lo, min(curr_m - 0.5 * seg, hi - seg))
        end_m = start_m + seg

        # Run the step-counting calibration over the chosen segment
        measured = self.calibration_movement(
            axis=axis,
            start_m=float(start_m),
            end_m=float(end_m),
            window_nm=int(window_nm),
            max_steps=max_steps,
            timeout_s=float(timeout_s),
            poll_interval_s=float(poll_interval_s),
        )
        step_m = float(measured[axis])

        # Update configured step size for this axis
        self.set_step_size_m(axis, step_m)

        # Optionally, keep the panel 'Single step' at 1 if that feature is enabled
        try:
            if getattr(self, '_reset_frontpanel_nsteps_after_move', False):
                with self._thread_lock:
                    dev = self._require_dev()
                    ch = self._axis_to_channel(axis)
                    try:
                        dev.move.writeNSteps(ch, 1)
                    except Exception:
                        pass
        except Exception:
            pass

        try:
            self.log.info(f'Calibration axis {axis}: step size set to {step_m:.3e} m/step '
                          f'(segment {seg:.3e} m, window {int(window_nm)} nm)')
        except Exception:
            pass
        return step_m

    def calibrate_all_axes_step_size(
            self,
            *,
            distance_m: float = 10e-6,
            window_nm: int = 600,
            timeout_s: float = 30.0,
            max_steps: Optional[int] = None,
            poll_interval_s: float = 0.002,
    ) -> Dict[str, float]:
        """
        Calibrate and update step sizes for all configured axes.
        Returns a dict {axis: step_size_m}.
        """
        results: Dict[str, float] = {}
        for axis in list(self._axis_map.keys()):
            try:
                step_m = self.calibrate_axis_step_size(
                    axis=axis,
                    distance_m=distance_m,
                    window_nm=window_nm,
                    timeout_s=timeout_s,
                    max_steps=max_steps,
                    poll_interval_s=poll_interval_s,
                )
                results[axis] = step_m
            except Exception:
                # Log and continue with other axes
                try:
                    self.log.exception(f'Calibration failed on axis {axis}')
                except Exception:
                    pass
        return results
