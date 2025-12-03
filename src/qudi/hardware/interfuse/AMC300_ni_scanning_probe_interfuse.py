# -*- coding: utf-8 -*-

"""
Interfuse: AMC300 motion (stepping) + NI InStreamer (APD) as a ScanningProbeInterface.

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

import threading
import time
import numpy as np
from typing import Dict, List, Optional

from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import Mutex

from qudi.interface.scanning_probe_interface import (
    ScanningProbeInterface,
    ScanConstraints,
    ScannerChannel,
    ScanSettings,
    ScanData,
    BackScanCapability,
)

from qudi.interface.data_instream_interface import DataInStreamInterface, SampleTiming


class AMC300NIScanningProbeInterfuse(ScanningProbeInterface):
    """
    - Motion is executed stepwise via a connected AMC300_stepper (ScanningProbeInterface).
    - APD data is acquired via NIXSeriesInStreamer (DataInStreamInterface).
    - Scans are software-stepped: for each pixel, move->settle->read NI samples->aggregate->store.

    This mirrors the structure of NiScanningProbeInterfuseBare, but without NI AO output.

    Example config:

    hardware:
        amc300_ni_scanner:
            module.Class: 'interfuse.AMC300_ni_scanning_probe_interfuse.AMC300NIScanningProbeInterfuse'
            connect:
                motion: 'amc300_stepper'
                ni_input: 'nicard_6343_instreamer'
            options:
                ni_channel_mapping:
                    fluorescence: 'PFI8'
                input_channel_units:
                    fluorescence: 'c/s'
                default_dwell_time_s: 0.5e-3    # optional if not deriving from frequency
                ni_sample_rate_hz: 50e1         # choose ≥ 1/dwell resolution you need
                settle_time_s: 0.001            # waiting time between pixels
                back_scan_available: true

                _defer_cursor_moves: true           #optional
                _cursor_move_debounce_ms: 350       #optional
                _use_closed_loop_for_deferred: true #optional
                _closed_loop_window_nm: 300         #optional
                _closed_loop_timeout_s: 5           #optional
                _closed_loop_disable_after: true    #optional

                scan_closed_loop_timeout_s: 5.0       # increase if moves are long
                scan_closed_loop_disable_after: true  # disable CL after each pixel to minimize controller load
                scan_closed_loop_enable_output: false # set true if AMC needs explicit output enable each time
                scan_motion_mode: linewise_open_fast       # or 'per_pixel_closed_loop' (default)
                calibration_window_nm: 500                 # optional, nm window for calibration approach

    Change config in Qudi console as follows:

        scanner = amc300_ni_scanner             # from config

        scanner.set_closed_loop_timeout_s(5)    # how long the scanner has time to move to target range
        scanner.set_cursor_move_debounce_ms(350)# how long after last cursor movement the deferred move is performed
        scanner.set_closed_loop_window_nm(300)  # size of the target range for deferred move

        scanner.set_scan_motion_mode('per_pixel_closed_loop')  # mode of scanning: 'linewise_open_fast' or 'per_pixel_closed_loop'
        scanner.set_calibration_window_nm(300)              # int value in nm; target range for calibration move
        scanner.set_scan_closed_loop_timeout_s(5)           # how long the scanner has time to move to target range in scans
        scanner.set_preferred_fast_axis('y')                # What os the fast axes
        scammer.set_ni_sample_rate_hz(500)                  # How many pixels per sample

        scanner.set_follow_gui_cursor_moves(False)          # Disable cursor movements (still allows scans) Default True

    """

    _threaded = True

    # Connectors
    _motion = Connector(name='motion', interface='ScanningProbeInterface')
    _ni_in = Connector(name='ni_input', interface='DataInStreamInterface')

    # Constraints mirrored to GUI/logic
    _input_channel_units: Dict[str, str] = ConfigOption('input_channel_units', default={}, missing='warn')

    # Acquisition and motion timing
    _ni_channel_mapping: Dict[str, str] = ConfigOption(name='ni_channel_mapping', missing='error')
    _default_dwell_time_s: float = ConfigOption('default_dwell_time_s', default=0.0005, missing='warn')
    _ni_sample_rate_hz: float = ConfigOption('ni_sample_rate_hz', default=50000.0, missing='warn')
    _settle_time_s: float = ConfigOption('settle_time_s', default=0.001, missing='warn')
    __default_backward_resolution: int = ConfigOption(name='default_backward_resolution', default=50)

    # Defered movement when cursor are slider is moved: Use controller closed-loop for the one deferred move (single window parameter, nm)
    _defer_cursor_moves: bool = ConfigOption('defer_cursor_moves', default=True, missing='nothing')
    _cursor_move_debounce_ms: int = ConfigOption('cursor_move_debounce_ms', default=250, missing='nothing')
    _use_closed_loop_for_deferred: bool = ConfigOption('use_closed_loop_for_deferred', default=True, missing='nothing')
    _closed_loop_window_nm: int = ConfigOption('closed_loop_window_nm', default=200, missing='nothing')
    _closed_loop_timeout_s: float = ConfigOption('closed_loop_timeout_s', default=1.5, missing='nothing')
    _closed_loop_disable_after: bool = ConfigOption('closed_loop_disable_after', default=True, missing='nothing')
    _follow_gui_cursor_moves: bool = ConfigOption('follow_gui_cursor_moves', default=True, missing='nothing')

    # Closed-loop parameters for per-pixel scan moves
    _scan_cl_timeout_s: float = ConfigOption('scan_closed_loop_timeout_s', default=2.0, missing='nothing')
    _scan_cl_disable_after: bool = ConfigOption('scan_closed_loop_disable_after', default=True, missing='nothing')
    _scan_cl_enable_output: bool = ConfigOption('scan_closed_loop_enable_output', default=False, missing='nothing')
    # Scan motion mode: 'per_pixel_closed_loop' (existing) or 'linewise_open_fast'
    _scan_motion_mode: str = ConfigOption('scan_motion_mode', default='per_pixel_closed_loop', missing='warn')
    # Calibration window for measuring step size (nm)
    _calibration_window_nm: int = ConfigOption('calibration_window_nm', default=300, missing='nothing')
    _suppress_gui_moves_after_activate_ms: int = ConfigOption('suppress_gui_moves_after_activate_ms', default=2000,
                                                              missing='nothing')

    # Internal state
    sigPositionChanged = QtCore.Signal(dict)
    _sigDeferredMoveRequested = QtCore.Signal(dict)
    _sigCancelDeferredMove = QtCore.Signal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._scan_settings: Optional[ScanSettings] = None
        self._scan_data: Optional[ScanData] = None
        self._back_scan_data: Optional[ScanData] = None
        self._constraints: Optional[ScanConstraints] = None
        self._saved_step_size_fast: Optional[Dict[str, float]] = None

        self._thread_lock_data = Mutex()

        # NI presentation/mapping
        self._present_channels: List[str] = []  # e.g. ['fluorescence']
        self._present_to_ni: Dict[str, str] = {}  # e.g. {'fluorescence': 'PFI8'}
        self._ni_channels_in_order: List[str] = []  # fixed order for readout

        #Defered Movement when cursor are slider is moved
        self._scan_active: bool = False  # if not already present
        self._move_debounce_timer: Optional[QtCore.QTimer] = None
        self._pending_move_target: Optional[Dict[str, float]] = None
        self._ui_target: Dict[str, float] = {}
        self._scan_intent: bool = False

        # Thread for scanner and target before scan
        self._worker_thread: Optional[threading.Thread] = None
        self._stored_target_pos: Dict[str, float] = {}
        self._suppress_until_monotonic: float = 0.0
        self._preferred_fast_axis: Optional[str] = None

    # Lifecycle
    def on_activate(self):

        #Constraints
        # 1) Axes from motion
        mcon: ScanConstraints = self._motion().constraints
        try:
            axis_objects = tuple(mcon.axis_objects.values())  # axes likely a dict
        except Exception:
            axis_objects = tuple(mcon.axis_objects)  # fallback if already a sequence

        # 2) Channels from NI in-streamer
        channels = list()
        for channel, unit in self._input_channel_units.items():
            channels.append(ScannerChannel(name=channel,
                                           unit=unit,
                                           dtype='float64'))

        back_scan_capability = BackScanCapability.AVAILABLE | BackScanCapability.RESOLUTION_CONFIGURABLE
        self._constraints = ScanConstraints(
            axis_objects=axis_objects,
            channel_objects=tuple(channels),
            back_scan_capability=back_scan_capability,
            has_position_feedback=False,
            square_px_only=False,
        )

        # Build channel maps for NI readout
        self._present_channels = list(self._input_channel_units.keys())
        self._present_to_ni = {present: self._ni_channel_mapping[present] for present in self._present_channels}
        self._ni_channels_in_order = [self._present_to_ni[p] for p in self._present_channels]

        # Re-emit position updates from motion so logic/GUI listening to THIS scanner get live cursor updates
        try:
            self._motion().sigPositionChanged.connect(self.sigPositionChanged.emit, QtCore.Qt.QueuedConnection)
        except Exception:
            pass

        # Emit current position once so the cursor/target snap to the actual position on activation (no motion)
        try:
            curr = self._motion().get_position()
            self._ui_target = dict(curr)  # CHANGED
            self.sigPositionChanged.emit(dict(curr))
        except Exception:
            pass

        self._suppress_until_monotonic = time.monotonic() + (float(self._suppress_gui_moves_after_activate_ms) / 1000.0)

        #Defered Movement
        self._move_debounce_timer = QtCore.QTimer(self)
        self._move_debounce_timer.setSingleShot(True)
        self._move_debounce_timer.timeout.connect(self._perform_deferred_move, QtCore.Qt.QueuedConnection)
        self._sigDeferredMoveRequested.connect(self._on_deferred_move_requested, QtCore.Qt.QueuedConnection)
        self._sigCancelDeferredMove.connect(self._on_cancel_deferred_move, QtCore.Qt.QueuedConnection)

        # Ensure clean state
        try:
            if self.module_state() != 'idle':
                self.module_state.unlock()
        except Exception:
            pass
        self._scan_active = False
        self._scan_intent = False
        self._stop_requested = False

    def on_deactivate(self):
        # Attempt to stop everything
        try:
            if self.module_state() != 'idle':
                self.stop_scan()
        except Exception:
            pass


    # Constraints and settings
    @property
    def constraints(self) -> ScanConstraints:
        """ Read-only property returning the constraints of this scanning probe hardware.
        """
        return self._constraints


    def reset(self) -> None:
        """ Hard reset of the hardware.
                """
        pass

    @property
    def scan_settings(self) -> Optional[ScanSettings]:
        """ Property returning all parameters needed for a 1D or 2D scan. Returns None if not configured.
                """
        if self._scan_data:
            return self._scan_data.settings
        else:
            return None

    @property
    def back_scan_settings(self) -> Optional[ScanSettings]:
        """ Property returning all parameters of the backwards scan. Returns None if not configured or not available.
                """
        if self._back_scan_data:
            return self._back_scan_data.settings
        else:
            return None

    def configure_scan(self, settings: ScanSettings) -> None:
        """ Configure the hardware with all parameters needed for a 1D or 2D scan.
                Raise an exception if the settings are invalid and do not comply with the hardware constraints.

                @param ScanSettings settings: ScanSettings instance holding all parameters
                """
        if self.is_scan_running:
            raise RuntimeError('Unable to configure scan parameters while scan is running. '
                               'Stop scanning and try again.')

        # Validate and clip settings against constraints
        settings = self.constraints.clip(settings)
        self.constraints.check_settings(settings)


        with self._thread_lock_data:
            self._scan_data = ScanData.from_constraints(settings, self._constraints)

            # reset back scan to defaults
            if len(settings.axes) == 1:
                back_resolution = (self.__default_backward_resolution,)
            else:
                back_resolution = (self.__default_backward_resolution, settings.resolution[1])
            back_scan_settings = ScanSettings(
                channels=settings.channels,
                axes=settings.axes,
                range=settings.range,
                resolution=back_resolution,
                frequency=settings.frequency,
            )
            self._back_scan_data = ScanData.from_constraints(back_scan_settings, self._constraints)

        # Configure NI InStreamer
        ni: DataInStreamInterface = self._ni_in()
        # Set sample rate if available on backend
        try:
            ni.set_sample_rate(self._ni_sample_rate_hz)
        except Exception:
            pass

    def configure_back_scan(self, settings: ScanSettings) -> None:
        """ Configure the hardware with all parameters of the backwards scan.
                Raise an exception if the settings are invalid and do not comply with the hardware constraints.

                @param ScanSettings settings: ScanSettings instance holding all parameters for the back scan
                """
        if self.is_scan_running:
            raise RuntimeError('Unable to configure scan parameters while scan is running. '
                               'Stop scanning and try again.')

        forward_settings = self.scan_settings
        # check settings - will raise appropriate exceptions if something is not right
        self.constraints.check_back_scan_settings(settings, forward_settings)
        self.log.debug('Back scan settings fulfill constraints.')
        with self._thread_lock_data:
            self._back_scan_data = ScanData.from_constraints(settings, self._constraints)
            self.log.debug(f'New back scan data created.')

    # Movement passthrough to motion module
    def move_absolute(self, position: Dict[str, float], velocity: Optional[float] = None,
                      blocking: bool = False) -> Dict[str, float]:
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
                velocity.

                Log error and return current target position if something fails or a scan is in progress.
                """

        # assert not self.is_running, 'Cannot move the scanner while, scan is running'
        if self.is_scan_running:
            self.log.error('Cannot move the scanner while scan is running')
            return self._motion().get_target()

        if not set(position).issubset(self.constraints.axes):
            self.log.error('Invalid axes name in position')
            return self._motion().get_target()

        is_module_thread = (self.thread() is QtCore.QThread.currentThread())
        # For interactive GUI drags (non-blocking), defer the actual hardware move
        if (
                not blocking
                and not self._scan_intent
                and not is_module_thread
        ):
            # Suppression window active right after activation?
            try:
                if time.monotonic() < self._suppress_until_monotonic:
                    # Read the real device position (fallback to target if read fails)
                    try:
                        curr = self._motion().get_position()
                    except Exception:
                        curr = self._motion().get_target()
                    # Update our UI shadow and broadcast to GUI
                    self._ui_target = dict(curr)
                    try:
                        self.sigPositionChanged.emit(dict(self._ui_target))
                    except Exception:
                        pass
                    # Important: return the actual position (not the requested target)
                    return dict(self._ui_target)
            except Exception:
                # If anything goes wrong in the guard, fall through to normal handling
                pass

            # If following GUI cursor is enabled, keep existing deferred behavior
            if self._defer_cursor_moves and self._follow_gui_cursor_moves:
                try:
                    self._sigDeferredMoveRequested.emit(dict(position))
                except Exception:
                    # if anything goes wrong, fall back to immediate move
                    return self._motion().move_absolute(position, velocity=velocity, blocking=blocking)
                # Return the requested target so GUI/logic won’t snap back while we defer the hardware move
                return dict(position)

            # Otherwise: update the cursor only (no hardware movement)
            if not self._ui_target:
                try:
                    self._ui_target = dict(self._motion().get_target())
                except Exception:
                    self._ui_target = {}
            for ax, val in position.items():
                self._ui_target[ax] = float(val)
            try:
                self.sigPositionChanged.emit(dict(self._ui_target))
            except Exception:
                pass
            return dict(position)

        return self._motion().move_absolute(position, velocity=velocity, blocking= blocking)

    def move_relative(self, distance: Dict[str, float], velocity: Optional[float] = None,
                      blocking: bool = False) -> Dict[str, float]:
        """ Move the scanning probe by a relative distance from the current target position as fast
                as possible or with a defined velocity.

                Log error and return current target position if something fails or a 1D/2D scan is in
                progress.
                """
        if self.is_scan_running:
            self.log.error('Cannot move the scanner while, scan is running')
            return self._motion().get_target()

            # Convert to absolute based on current target, then reuse move_absolute (will defer if configured)
        curr = self.get_target()
        absolute = {ax: curr.get(ax, 0.0) + float(d) for ax, d in distance.items()}
        return self.move_absolute(absolute, velocity=velocity, blocking=blocking)


    def get_target(self) -> Dict[str, float]:
        if self._defer_cursor_moves and self._ui_target:
            return dict(self._ui_target)
        return self._motion().get_target()

    def get_position(self) -> Dict[str, float]:
        return self._motion().get_position()

    @QtCore.Slot(dict)
    def _on_deferred_move_requested(self, position: Dict[str, float]):
        """For change of cursor or silder, the target position is updated and stored as pending target.
            The timer is reseted.
            """
        # Update shadow target immediately for smooth UI; clip to axes present
        if not self._ui_target:
            self._ui_target = dict(self._motion().get_target())
        for ax, val in position.items():
            self._ui_target[ax] = float(val)
        try:
            self.sigPositionChanged.emit(dict(self._ui_target))
        except Exception:
            pass

        # Coalesce pending move; restart debounce
        self._pending_move_target = dict(self._ui_target)

        if not self._follow_gui_cursor_moves:
            # Do not actually start a deferred hardware move
            self._pending_move_target = None
            return

        if self._move_debounce_timer is not None:
            self._move_debounce_timer.start(int(self._cursor_move_debounce_ms))

    @QtCore.Slot()
    def _on_cancel_deferred_move(self):
        """Cancel deferred movement and stops timer
            """
        self._pending_move_target = None
        if self._move_debounce_timer is not None:
            self._move_debounce_timer.stop()

    @QtCore.Slot()
    def _perform_deferred_move(self):
        """ As soon as the timer finishes, the attocubes move to pending target position.
            """
        if self._pending_move_target is None:
            return
        if not self._follow_gui_cursor_moves:
            self._pending_move_target = None
            return
        # Do not interfere with scans
        if self.is_scan_running or self._scan_intent:
            self._pending_move_target = None
            return
        pos = self._pending_move_target
        self._pending_move_target = None
        try:
            # NEW: use controller closed-loop move for the consolidated cursor move (single window)
            if self._use_closed_loop_for_deferred and hasattr(self._motion(), 'move_absolute_closed_loop'):
                final_target = getattr(self._motion(), 'move_absolute_closed_loop')(
                    pos,
                    window_nm=int(self._closed_loop_window_nm),
                    timeout_s=float(self._closed_loop_timeout_s),
                    disable_after=bool(self._closed_loop_disable_after),
                )
            else:
                # Fallback: single consolidated move; blocking for cleanliness
                final_target = self._motion().move_absolute(pos, velocity=None, blocking=True)

            # Sync shadow target to hardware and notify UI
            self._ui_target = dict(final_target)
            self.sigPositionChanged.emit(dict(self._ui_target))
        except Exception:
            self.log.exception('Deferred move failed')

    # Scan lifecycle
    def start_scan(self):
        """Start a scan as configured beforehand.
        Log an error if something fails or a 1D/2D scan is in progress.

        Offload self._start_scan() from the caller to the module's thread.
        ATTENTION: Do not call this from within thread lock protected code to avoid deadlock (PR #178).
        :return:
        """
        self._scan_intent = True
        try:
            if self.thread() is not QtCore.QThread.currentThread():
                QtCore.QMetaObject.invokeMethod(self, '_start_scan', QtCore.Qt.BlockingQueuedConnection)
            else:
                self._start_scan()

        except:
            self._scan_intent = False
            self.log.exception("")

    @QtCore.Slot()
    def _start_scan(self):
        try:
            if self._scan_data is None:
                self.log.error('Scan Data is None. Scan settings need to be configured before starting')
                self._scan_intent = False
                return

            if self.is_scan_running:
                self.log.error('Cannot start a scan while scanning probe is already running')
                self._scan_intent = False
                return

            # Cancel any cursor deferrals
            try:
                self._sigCancelDeferredMove.emit()
            except Exception:
                pass

            # Store current target to restore after scan
            try:
                self._stored_target_pos = self._motion().get_target().copy()
            except Exception:
                self._stored_target_pos = {}

            # INITIALIZE SCAN BUFFERS HERE
            # Allocate the per-channel arrays in ScanData (and back-scan) and stamp metadata.
            with self._thread_lock_data:
                # Allocate forward scan arrays
                self._scan_data.new_scan()
                # Stamp the chosen scan mode into metadata for traceability
                try:
                    # Put it into coord_transform_info to persist in ScanData
                    self._scan_data.coord_transform_info['scan_motion_mode'] = self._scan_motion_mode
                    if self._back_scan_data is not None:
                        self._back_scan_data.coord_transform_info['scan_motion_mode'] = self._scan_motion_mode
                except Exception:
                    pass
                # Record where we started
                self._scan_data.scanner_target_at_start = dict(self._stored_target_pos)

                # Allocate back-scan arrays if configured
                if self._back_scan_data is not None:
                    self._back_scan_data.new_scan()
                    self._back_scan_data.scanner_target_at_start = dict(self._stored_target_pos)

            # Optional: pre-scan calibration for fast axis in 'linewise_open_fast' mode
            if self._scan_motion_mode == 'linewise_open_fast':
                settings = self._scan_data.settings
                axes_names = list(settings.axes)
                # Choose fast axis index: user override if valid, else 0
                fast_idx = 0
                if len(axes_names) >= 1 and isinstance(self._preferred_fast_axis,
                                                       str) and self._preferred_fast_axis in axes_names:
                    fast_idx = axes_names.index(self._preferred_fast_axis)
                fast_ax = axes_names[fast_idx]
                # Use the matching range entry
                fast_min, fast_max = settings.range[fast_idx]
                pos_start = {}
                for i, ax in enumerate(axes_names):
                    rng = settings.range[i]
                    pos_start[ax] = float(rng[0])  # start at min of each scan axis

                motion = self._motion()

                # Closed-loop to start of scan
                if hasattr(motion, 'move_absolute_closed_loop'):
                    motion.move_absolute_closed_loop(
                        pos_start,
                        window_nm=int(self._calibration_window_nm),
                        timeout_s=float(self._scan_cl_timeout_s),
                        disable_after=True,
                        enable_output=bool(self._scan_cl_enable_output),
                    )
                else:
                    motion.move_absolute(pos_start, blocking=True)

                # Calibration movement on fast axis: start -> end, step counting
                if hasattr(motion, 'calibration_movement'):
                    measured = motion.calibration_movement(
                        axis=fast_ax,
                        start_m=float(fast_min),
                        end_m=float(fast_max),
                        window_nm=int(self._calibration_window_nm),
                        timeout_s=max(2.0, float(self._scan_cl_timeout_s)),
                    )
                    # Closed-loop back to start for scanning
                    if hasattr(motion, 'move_absolute_closed_loop'):
                        motion.move_absolute_closed_loop(
                            pos_start,
                            window_nm=int(self._calibration_window_nm),
                            timeout_s=float(self._scan_cl_timeout_s),
                            disable_after=True,
                            enable_output=bool(self._scan_cl_enable_output),
                        )
                    else:
                        motion.move_absolute(pos_start, blocking=True)

                    # Save and override fast-axis step size (restored in _stop_scan)
                    try:
                        original = motion.get_step_size_m(fast_ax)
                        self._saved_step_size_fast = {fast_ax: original}
                        motion.set_step_size_m(fast_ax, float(measured[fast_ax]))
                        self.log.debug(f"Fast axis '{fast_ax}' calibrated step size: "
                                       f"{measured[fast_ax]:.3e} m/step (was {original:.3e})")
                    except Exception:
                        self.log.exception('Failed to override fast axis step size after calibration.')
                else:
                    self.log.warning('Motion module does not provide calibration_movement(); '
                                     'skipping fast-axis calibration.')

            # Start NI stream now
            try:
                self._ni_in().start_stream()
            except Exception:
                # Some backends auto-start on first read
                pass

            # Mark running and lock module
            self._stop_requested = False
            self._scan_active = True
            self.module_state.lock()

            # Launch worker thread
            self._worker_thread = threading.Thread(target=self._run_scan_worker, name='amc300-ni-scan', daemon=True)
            self._worker_thread.start()

        except Exception as e:
            self._scan_active = False
            self._scan_intent = False
            try:
                self.module_state.unlock()
            except Exception:
                pass
            self.log.exception("Starting scan failed.", exc_info=e)

    def stop_scan(self):
        """Stop the currently running scan.
        Log an error if something fails or no 1D/2D scan is in progress.

        Offload self._stop_scan() from the caller to the module's thread.
        ATTENTION: Do not call this from within thread lock protected code to avoid deadlock (PR #178).
        :return:
        """

        if self.thread() is not QtCore.QThread.currentThread():
            QtCore.QMetaObject.invokeMethod(self, '_stop_scan',
                                            QtCore.Qt.BlockingQueuedConnection)
        else:
            self._stop_scan()

    @QtCore.Slot()
    def _stop_scan(self):
        if not self.is_scan_running:
            self.log.error('No scan in progress. Cannot stop scan.')

            # Stop worker
        self._stop_requested = True
        thr = self._worker_thread
        if thr is not None and thr.is_alive():
            thr.join(timeout=5.0)
        self._worker_thread = None

        # Stop NI stream
        try:
            self._ni_in().stop_stream()
        except Exception:
            pass

        # Unlock and restore
        self._scan_active = False
        self._scan_intent = False
        try:
            self.module_state.unlock()
        except Exception:
            pass

        # Restore stored target
        if self._stored_target_pos:
            try:
                self._motion().move_absolute(self._stored_target_pos, blocking=True)
            except Exception:
                pass
        self._stored_target_pos = {}

        # Sync UI
        try:
            self._ui_target = self._motion().get_target()
            self.sigPositionChanged.emit(dict(self._ui_target))
        except Exception:
            pass

        # Restore fast-axis step size if overridden
        try:
            if self._saved_step_size_fast:
                for ax, val in self._saved_step_size_fast.items():
                    try:
                        self._motion().set_step_size_m(ax, float(val))
                    except Exception:
                        pass
        finally:
            self._saved_step_size_fast = None

    def emergency_stop(self) -> None:

        self._stop_requested = True
        thr = self._worker_thread
        if thr is not None and thr.is_alive():
            thr.join(timeout=1.0)
        self._worker_thread = None
        try:
            self._motion().emergency_stop()
        except Exception:
            pass
        try:
            self._ni_in().stop_stream()
        except Exception:
            pass
        try:
            if self.module_state() != 'idle':
                self.module_state.unlock()
        except Exception:
            pass
        self._sigCancelDeferredMove.emit()
        self._scan_intent = False
        try:
            self._ui_target = self._motion().get_target()
            self.sigPositionChanged.emit(dict(self._ui_target))
        except Exception:
            pass

    @property
    def is_scan_running(self):
        """
        Read-only flag indicating the module state.

        @return bool: scanning probe is running (True) or not (False)
        """
        # module state used to indicate hw timed scan running
        #self.log.debug(f"Module in state: {self.module_state()}")
        #assert self.module_state() in ('locked', 'idle')  # TODO what about other module states?

        if self.module_state() == 'locked':
            return True
        else:
            return False

    # Worker: software-stepped scan
    def _run_scan_worker(self):
        try:
            settings = self._scan_data.settings if self._scan_data else None
            data = self._scan_data
            if settings is None or data is None:
                raise RuntimeError('Scan not configured')

            # Build axis vectors
            axes_names = list(settings.axes)
            axis_values: List[np.ndarray] = []
            for i, ax in enumerate(axes_names):
                mn, mx = settings.range[i]
                n = int(settings.resolution[i])
                axis_values.append(np.linspace(float(mn), float(mx), n))

            fast_idx = 0
            if len(axes_names) == 2 and isinstance(self._preferred_fast_axis, str) and self._preferred_fast_axis in axes_names:
                fast_idx = axes_names.index(self._preferred_fast_axis)
            fast_ax = axes_names[fast_idx]
            fast_vals = axis_values[fast_idx]
            nx_fast = int(settings.resolution[fast_idx])

            is_2d = (len(axes_names) == 2)
            if is_2d:
                slow_idx = 1 - fast_idx
                slow_ax = axes_names[slow_idx]
                slow_vals = axis_values[slow_idx]
                ny_slow = int(settings.resolution[slow_idx])

            # Window size for closed-loop moves
            pixel_sizes_m: List[float] = []
            for i, ax in enumerate(axes_names):
                mn, mx = settings.range[i]
                n = int(settings.resolution[i])
                steps = max(n - 1, 1)
                pixel_sizes_m.append(abs(float(mx) - float(mn)) / steps)
            cl_window_nm = max(1, int(round(max(pixel_sizes_m) * 1e9)))

            # Dwell and sampling
            if settings.frequency <= 0:
                raise ValueError('Scan frequency must be > 0. Set it in the Scanner GUI.')
            dwell_s = 1.0 / float(settings.frequency)
            ni: DataInStreamInterface = self._ni_in()
            # CRITICAL: use the actual device sample rate to compute samples per pixel.
            try:
                sample_rate = float(getattr(ni, 'sample_rate'))
            except Exception:
                sample_rate = float(self._ni_sample_rate_hz)  # fallback
            samples_per_pixel = max(1, int(round(sample_rate * dwell_s)))

            # Active channel names
            try:
                active_ni_channels = list(getattr(ni, 'active_channels'))
            except Exception:
                try:
                    active_ni_channels = list(ni.get_active_channels())
                except Exception:
                    active_ni_channels = []

            normalized_active = []
            for name in active_ni_channels:
                try:
                    normalized_active.append(str(name).split('/')[-1])
                except Exception:
                    continue

            if not normalized_active:
                if self._ni_channels_in_order:
                    normalized_active = [str(n) for n in self._ni_channels_in_order]
                else:
                    normalized_active = ['PFI8']

            stream_ch_count = max(1, len(normalized_active))

            # Map presented alias -> index
            present_to_active_idx: Dict[str, int] = {}
            for alias in self._present_channels:
                ni_name = self._present_to_ni.get(alias, '')
                idx = -1
                try:
                    idx = normalized_active.index(ni_name)
                except ValueError:
                    if stream_ch_count == 1 and len(self._present_channels) == 1:
                        idx = 0
                present_to_active_idx[alias] = idx
                if idx < 0:
                    self.log.warning(f'NI channel {ni_name} for {alias} is not active in streamer. Active={normalized_active}')

            buf_dtype = getattr(getattr(ni, 'constraints', object()), 'data_type', np.float64)

            # Flush helper: drain all currently available samples so next read starts "now"
            def _drain_stream() -> None:
                try:
                    # Try fast-path if provided by buffer wrapper
                    total = 0
                    scratch = np.empty(stream_ch_count * 4096, dtype=buf_dtype)
                    while True:
                        avail = getattr(ni, 'available_samples', 0)
                        if avail <= 0:
                            break
                        n = int(min(avail, scratch.size // stream_ch_count))
                        if n <= 0:
                            break
                        try:
                            read = ni.read_available_data_into_buffer(scratch, None)
                        except AttributeError:
                            # Fallback: blocking read for currently available
                            ni.read_data_into_buffer(scratch[:n * stream_ch_count], samples_per_channel=n)
                            read = n
                        if read <= 0:
                            break
                        total += read
                except Exception:
                    # Never fail the scan if flushing fails
                    pass

            # Digital/rate detection
            def _is_digital_like(arr: np.ndarray) -> bool:
                # Treat as digital if values are close to 0/1
                if arr.size < 4:
                    return False
                vmin = float(np.nanmin(arr))
                vmax = float(np.nanmax(arr))
                return (vmin >= -0.25) and (vmax <= 1.25)

            # Acquire exactly one pixel window (after flush) and aggregate per channel
            def _acquire_and_aggregate() -> Dict[str, float]:
                # Discard any stale samples from before this pixel
                _drain_stream()

                # Now pull exactly the dwell window
                interleaved = np.zeros(stream_ch_count * samples_per_pixel, dtype=buf_dtype)
                ni.read_data_into_buffer(interleaved, samples_per_channel=samples_per_pixel)

                channel_vals: Dict[str, float] = {}
                for alias in self._present_channels:
                    idx = present_to_active_idx.get(alias, -1)
                    if idx < 0 or idx >= stream_ch_count:
                        channel_vals[alias] = np.nan
                        continue
                    ch_slice = interleaved[idx::stream_ch_count][:samples_per_pixel]

                    unit = (self._input_channel_units.get(alias, '') or '').strip().lower()
                    ni_name = (self._present_to_ni.get(alias, '') or '').lower()

                    # Heuristic:
                    # - If values look like rates (not near {0,1}), just average (this matches TimeSeries).
                    # - If values look digital-like and unit is 'c/s', do rising-edge counting → counts/s.
                    # - Otherwise, average.
                    looks_digital = _is_digital_like(ch_slice)
                    if unit == 'c/s' and looks_digital:
                        bin_slice = (ch_slice > 0.5).astype(np.uint8)
                        # Rising edges 0 -> 1
                        edge_count = int(np.count_nonzero((bin_slice[1:] == 1) & (bin_slice[:-1] == 0)))
                        dwell = float(samples_per_pixel) / float(sample_rate) if sample_rate > 0 else np.nan
                        channel_vals[alias] = (edge_count / dwell) if dwell and dwell > 0 else np.nan
                    else:
                        # Average (works for rate-like outputs and analog)
                        channel_vals[alias] = float(np.mean(ch_slice))

                return channel_vals

            # Motion shortcut
            motion = self._motion()

            # ... below: unchanged scan loops, but each place that reads a pixel uses ch_means = _acquire_and_aggregate() ...

            if self._scan_motion_mode == 'linewise_open_fast':
                data_dict = data.data
                if data_dict is None:
                    raise RuntimeError('ScanData.data not initialized. Did you call data.new_scan()?')

                if not is_2d:
                    for p in range(nx_fast):
                        if self._stop_requested:
                            break
                        if p > 0:
                            motion.move_absolute({fast_ax: float(fast_vals[p])}, blocking=True)
                            time.sleep(self._settle_time_s)

                        ch_means = _acquire_and_aggregate()
                        idx_tuple = (p,)
                        for ch_name in self._present_channels:
                            arr = data_dict.get(ch_name)
                            if arr is not None:
                                arr[idx_tuple] = ch_means.get(ch_name, np.nan)
                else:
                    for l in range(ny_slow):
                        if self._stop_requested:
                            break
                        pos_line_start = {fast_ax: float(fast_vals[0]), slow_ax: float(slow_vals[l])}
                        if hasattr(motion, 'move_absolute_closed_loop'):
                            motion.move_absolute_closed_loop(
                                pos_line_start,
                                window_nm=int(cl_window_nm),
                                timeout_s=float(self._scan_cl_timeout_s),
                                disable_after=True,
                                enable_output=bool(self._scan_cl_enable_output),
                            )
                        else:
                            motion.move_absolute(pos_line_start, blocking=True)
                        time.sleep(self._settle_time_s)

                        for p in range(nx_fast):
                            if self._stop_requested:
                                break
                            if p > 0:
                                motion.move_absolute({fast_ax: float(fast_vals[p])}, blocking=True)
                                time.sleep(self._settle_time_s)
                            ch_means = _acquire_and_aggregate()
                            idx_tuple = (p, l) if fast_idx == 0 else (l, p)
                            for ch_name in self._present_channels:
                                arr = data_dict.get(ch_name)
                                if arr is not None:
                                    arr[idx_tuple] = ch_means.get(ch_name, np.nan)
            else:
                data_dict = data.data
                if data_dict is None:
                    raise RuntimeError('ScanData.data not initialized. Did you call data.new_scan()?')

                if not is_2d:
                    for p in range(nx_fast):
                        if self._stop_requested:
                            break
                        pos = {fast_ax: float(fast_vals[p])}
                        if hasattr(motion, 'move_absolute_closed_loop'):
                            try:
                                motion.move_absolute_closed_loop(
                                    pos,
                                    window_nm=int(cl_window_nm),
                                    timeout_s=float(self._scan_cl_timeout_s),
                                    disable_after=bool(self._scan_cl_disable_after),
                                    enable_output=bool(self._scan_cl_enable_output),
                                )
                            except Exception:
                                motion.move_absolute(pos, blocking=True)
                        else:
                            motion.move_absolute(pos, blocking=True)
                        time.sleep(self._settle_time_s)

                        ch_means = _acquire_and_aggregate()
                        idx_tuple = (p,)
                        for ch_name in self._present_channels:
                            arr = data_dict.get(ch_name)
                            if arr is not None:
                                arr[idx_tuple] = ch_means.get(ch_name, np.nan)
                else:
                    for l in range(ny_slow):
                        for p in range(nx_fast):
                            if self._stop_requested:
                                break
                            pos = {fast_ax: float(fast_vals[p]), slow_ax: float(slow_vals[l])}
                            if hasattr(motion, 'move_absolute_closed_loop'):
                                try:
                                    motion.move_absolute_closed_loop(
                                        pos,
                                        window_nm=int(cl_window_nm),
                                        timeout_s=float(self._scan_cl_timeout_s),
                                        disable_after=bool(self._scan_cl_disable_after),
                                        enable_output=bool(self._scan_cl_enable_output),
                                    )
                                except Exception:
                                    motion.move_absolute(pos, blocking=True)
                            else:
                                motion.move_absolute(pos, blocking=True)
                            time.sleep(self._settle_time_s)

                            ch_means = _acquire_and_aggregate()
                            idx_tuple = (p, l) if fast_idx == 0 else (l, p)
                            for ch_name in self._present_channels:
                                arr = data_dict.get(ch_name)
                                if arr is not None:
                                    arr[idx_tuple] = ch_means.get(ch_name, np.nan)

            # Finish scan (unchanged)
            try:
                data.finish_scan()
            except Exception:
                pass
            try:
                if self._back_scan_data is not None:
                    self._back_scan_data.finish_scan()
            except Exception:
                pass

        except Exception:
            self.log.exception('Scan worker failed')
        finally:
            self._scan_active = False
            self._scan_intent = False
            try:
                if self.module_state() == 'locked':
                    self.module_state.unlock()
            except Exception:
                pass

    def get_scan_data(self) -> Optional[ScanData]:
        """ Read-only property returning the ScanData instance used in the scan.
        """
        if self._scan_data is None:
            return None
        else:
            with self._thread_lock_data:
                return self._scan_data.copy()

    def get_back_scan_data(self) -> Optional[ScanData]:
        """ Retrieve the ScanData instance used in the backwards scan.
        """
        if self._scan_data is None:
            return None
        else:
            with self._thread_lock_data:
                return self._back_scan_data.copy()

    def set_scan_motion_mode(self, mode: str) -> None:
        """
        Set the scan motion mode at runtime.
        Allowed values:
            - 'per_pixel_closed_loop'
            - 'linewise_open_fast'
        """
        mode = str(mode).strip()
        if mode not in ('per_pixel_closed_loop', 'linewise_open_fast'):
            raise ValueError("scan_motion_mode must be 'per_pixel_closed_loop' or 'linewise_open_fast'")
        self._scan_motion_mode = mode
        self.log.info(f"Scan motion mode set to: {self._scan_motion_mode}")

    def get_scan_motion_mode(self) -> str:
        """Return the current scan motion mode."""
        return str(self._scan_motion_mode)

    def set_calibration_window_nm(self, window_nm: int) -> None:
        """
        Set the calibration closed-loop target window (in nm) used during the pre-scan
        calibration in 'linewise_open_fast' mode.
        """
        try:
            w = int(window_nm)
        except Exception as exc:
            raise ValueError('window_nm must be an integer (nanometers).') from exc
        if w <= 0:
            raise ValueError('window_nm must be > 0 nm.')
        self._calibration_window_nm = w
        self.log.info(f'Calibration window set to {self._calibration_window_nm} nm')

    def get_calibration_window_nm(self) -> int:
        """Return the current calibration closed-loop target window (in nm)."""
        return int(self._calibration_window_nm)

    def set_closed_loop_window_nm(self, window_nm: int) -> None:
        """
        Set the closed-loop target window (in nm) used for deferred cursor moves.
        This does NOT affect the scanning closed-loop window, which is computed dynamically per pixel.
        """
        try:
            w = int(window_nm)
        except Exception as exc:
            raise ValueError('window_nm must be an integer (nanometers).') from exc
        if w <= 0:
            raise ValueError('window_nm must be > 0 nm.')
        self._closed_loop_window_nm = w
        self.log.info(f'Cursor closed-loop window set to {self._closed_loop_window_nm} nm')

    def get_closed_loop_window_nm(self) -> int:
        """Return the current cursor closed-loop target window (in nm)."""
        return int(self._closed_loop_window_nm)

    def set_cursor_move_debounce_ms(self, ms: int) -> None:
        """
        Set the debounce time (milliseconds) for deferred cursor/slider moves.
        """
        try:
            val = int(ms)
        except Exception as exc:
            raise ValueError('ms must be an integer (milliseconds).') from exc
        if val < 0:
            raise ValueError('ms must be >= 0.')
        self._cursor_move_debounce_ms = val
        # If a deferred move is currently pending, the new value will be used on the next start()
        self.log.info(f'Cursor move debounce set to {self._cursor_move_debounce_ms} ms')

    def get_cursor_move_debounce_ms(self) -> int:
        """Return the current debounce time (milliseconds) for deferred cursor moves."""
        return int(self._cursor_move_debounce_ms)

    def set_closed_loop_timeout_s(self, timeout_s: float) -> None:
        """
        Set the timeout (seconds) for deferred cursor closed-loop moves.
        This affects only the cursor's closed-loop path, not scanning.
        """
        try:
            val = float(timeout_s)
        except Exception as exc:
            raise ValueError('timeout_s must be a number (seconds).') from exc
        if val <= 0:
            raise ValueError('timeout_s must be > 0.')
        self._closed_loop_timeout_s = val
        self.log.info(f'Cursor closed-loop timeout set to {self._closed_loop_timeout_s} s')

    def get_closed_loop_timeout_s(self) -> float:
        """Return the current timeout (seconds) for cursor closed-loop moves."""
        return float(self._closed_loop_timeout_s)

    def set_scan_closed_loop_timeout_s(self, timeout_s: float) -> None:
        """
        Set the timeout (seconds) used for closed-loop moves during scanning
        (e.g., per-pixel CL or line-start CL in linewise mode).
        """
        try:
            val = float(timeout_s)
        except Exception as exc:
            raise ValueError('timeout_s must be a number (seconds).') from exc
        if val <= 0:
            raise ValueError('timeout_s must be > 0.')
        self._scan_cl_timeout_s = val
        self.log.info(f'Scan closed-loop timeout set to {self._scan_cl_timeout_s} s')

    def get_scan_closed_loop_timeout_s(self) -> float:
        """Return the current timeout (seconds) used for closed-loop moves during scanning."""
        return float(self._scan_cl_timeout_s)

    def set_follow_gui_cursor_moves(self, enabled: bool) -> None:
        """
        Enable/disable hardware following non-blocking GUI cursor moves.
        When disabled, the hardware will not move for GUI-driven, non-blocking move_absolute calls.
        """
        self._follow_gui_cursor_moves = bool(enabled)
        if not self._follow_gui_cursor_moves:
            # Cancel any pending deferred move
            try:
                self._sigCancelDeferredMove.emit()
            except Exception:
                pass
        self.log.info(f'follow_gui_cursor_moves set to {self._follow_gui_cursor_moves}')

    def get_follow_gui_cursor_moves(self) -> bool:
        """Return whether hardware follows non-blocking GUI cursor moves."""
        return bool(self._follow_gui_cursor_moves)

    # Add inside class AMC300NIScanningProbeInterfuse, e.g. near other public API methods.

    def set_ni_sample_rate_hz(self, rate_hz: float) -> None:
        """
        Set desired NI sample rate (Hz).
        - If no scan is running, it also tries to apply the rate to the NI streamer immediately.
        - If a scan is running, the value is stored and will be applied on the next configure/start.
        """
        try:
            val = float(rate_hz)
        except Exception as exc:
            raise ValueError("rate_hz must be a number (Hz).") from exc
        if val <= 0:
            raise ValueError("rate_hz must be > 0 Hz.")

        # Store new rate
        self._ni_sample_rate_hz = val

        # Apply immediately only when not scanning; otherwise defer
        if self.is_scan_running:
            try:
                self.log.info(f"NI sample rate set to {val} Hz (deferred until next scan).")
            except Exception:
                pass
            return

        # Best-effort apply to NI backend now
        try:
            self._ni_in().set_sample_rate(val)
            try:
                self.log.info(f"NI sample rate set to {val} Hz (applied to NI streamer).")
            except Exception:
                pass
        except Exception:
            # Some backends only accept rate during configuration/start
            try:
                self.log.info(f"NI sample rate set to {val} Hz (will apply on next configure/start).")
            except Exception:
                pass

    def get_ni_sample_rate_hz(self) -> float:
        """
        Return the currently configured NI sample rate (Hz) stored in this interfuse.
        Note: The NI backend might still be using an older rate until the next configure/start.
        """
        return float(self._ni_sample_rate_hz)

    def set_preferred_fast_axis(self, axis: Optional[str]) -> None:
        """
        Set the preferred fast axis name for 2D scans.
        - axis: e.g. 'x' or 'y'. Use None to clear and use default (first axis).
        - Takes effect on the next scan start.
        """
        if axis is None:
            self._preferred_fast_axis = None
            try:
                self.log.info('preferred_fast_axis cleared (default fast axis will be used).')
            except Exception:
                pass
            return
        axis = str(axis).strip()
        # Validate against available axes
        avail = tuple(self._constraints.axes.keys()) if self._constraints else tuple()
        if not avail:
            try:
                avail = tuple(self._motion().constraints.axes.keys())
            except Exception:
                avail = tuple()
        if axis not in avail:
            raise ValueError(f"Unknown axis '{axis}'. Available axes: {avail}")
        self._preferred_fast_axis = axis
        try:
            self.log.info(f'preferred_fast_axis set to {self._preferred_fast_axis} (applies on next scan).')
        except Exception:
            pass

    def get_preferred_fast_axis(self) -> Optional[str]:
        """Return the preferred fast axis or None if default is used."""
        return self._preferred_fast_axis