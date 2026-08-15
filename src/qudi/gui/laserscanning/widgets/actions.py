# -*- coding: utf-8 -*-
"""
Contains the QActions for the laser scanning toolchain GUI.

Copyright (c) 2024, the qudi developers. See the AUTHORS.md file at the top-level directory of this
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

__all__ = ['LaserScanningActions']

import os
from PySide6 import QtWidgets, QtGui

from qudi.util.paths import get_artwork_dir


class LaserScanningActions:
    """ """

    action_close: QtGui.QAction
    action_save: QtGui.QAction
    action_show_fit_configuration: QtGui.QAction
    action_start_stop_record: QtGui.QAction
    action_start_stop_scan: QtGui.QAction
    action_clear_data: QtGui.QAction
    action_show_frequency: QtGui.QAction
    action_autoscale_histogram: QtGui.QAction
    action_show_histogram_region: QtGui.QAction
    action_restore_view: QtGui.QAction
    action_laser_only: QtGui.QAction

    def __init__(self):
        super().__init__()

        # Create QActions (QAction is in QtGui in PySide6, not QtWidgets)
        self.action_close = QtGui.QAction('Close')
        self.action_save = QtGui.QAction('Save')
        self.action_show_fit_configuration = QtGui.QAction('Fit Configuration')
        self.action_start_stop_record = QtGui.QAction('Start/Stop Laser Recording')
        self.action_start_stop_scan = QtGui.QAction('Start/Stop Laser Scanning')
        self.action_clear_data = QtGui.QAction('Clear trace data')
        self.action_show_frequency = QtGui.QAction('Frequency Mode')
        self.action_autoscale_histogram = QtGui.QAction('Autoscale histogram')
        self.action_show_histogram_region = QtGui.QAction('Show histogram region')
        self.action_restore_view = QtGui.QAction('Restore default')
        self.action_laser_only = QtGui.QAction('Laser-Only Mode')

        # Create and set icons
        icon_path = os.path.join(get_artwork_dir(), 'icons')
        exit_icon = QtGui.QIcon(os.path.join(icon_path, 'application-exit'))
        save_icon = QtGui.QIcon(os.path.join(icon_path, 'document-save'))
        configure_icon = QtGui.QIcon(os.path.join(icon_path, 'configure'))
        clear_icon = QtGui.QIcon(os.path.join(icon_path, 'edit-clear'))
        scale_icon = QtGui.QIcon(os.path.join(icon_path, 'zoom-fit-best'))
        record_icon = QtGui.QIcon(os.path.join(icon_path, 'record-counter'))
        record_icon.addFile(os.path.join(icon_path, 'stop-counter'), state=QtGui.QIcon.State.On)
        play_icon = QtGui.QIcon(os.path.join(icon_path, 'start-counter'))
        play_icon.addFile(os.path.join(icon_path, 'stop-counter'), state=QtGui.QIcon.State.On)

        self.action_close.setIcon(exit_icon)
        self.action_save.setIcon(save_icon)
        self.action_show_fit_configuration.setIcon(configure_icon)
        self.action_start_stop_record.setIcon(record_icon)
        self.action_start_stop_scan.setIcon(play_icon)
        self.action_clear_data.setIcon(clear_icon)
        self.action_autoscale_histogram.setIcon(scale_icon)

        # Set tooltips
        self.action_close.setToolTip('Close window. Does NOT deactivate module.')
        self.action_save.setToolTip('Save all data')
        self.action_show_fit_configuration.setToolTip(
            'Open a dialog to edit data fitting configurations.'
        )
        self.action_start_stop_record.setToolTip(
            'Start/Stop data acquisition only without scanning the laser'
        )
        self.action_start_stop_scan.setToolTip('Start/Stop laser scanning and data recording')
        self.action_clear_data.setToolTip('Deletes all acquired data up until now')
        self.action_show_frequency.setToolTip('Toggle between wavelength and frequency laser data')
        self.action_autoscale_histogram.setToolTip(
            'Automatically set boundaries of histogram with min/max x value'
        )
        self.action_show_histogram_region.setToolTip('Show visual overlay of the histogram span.')
        self.action_restore_view.setToolTip('Restores default view of the window')
        self.action_laser_only.setToolTip('If checked, the measurement will record laser data only')

        # Configure checkable flags
        self.action_close.setCheckable(False)
        self.action_save.setCheckable(False)
        self.action_show_fit_configuration.setCheckable(False)
        self.action_start_stop_record.setCheckable(True)
        self.action_start_stop_scan.setCheckable(True)
        self.action_clear_data.setCheckable(False)
        self.action_show_frequency.setCheckable(True)
        self.action_autoscale_histogram.setCheckable(False)
        self.action_show_histogram_region.setCheckable(True)
        self.action_restore_view.setCheckable(False)
        self.action_laser_only.setCheckable(True)