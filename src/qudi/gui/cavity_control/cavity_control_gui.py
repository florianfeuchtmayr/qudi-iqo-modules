# -*- coding: utf-8 -*-
"""
Minimal GUI for cavity control:
- Parameter inputs (alpha, AO1 range, ramp speed, osc decimation)
- Full scan controls (start/stop, progress, live plot)
- Apply constant coarse voltage
- Drift tracker controls (enable, speed, deadband, bounds)
"""

from PySide2 import QtWidgets, QtCore
from qudi.core.connector import Connector
from qudi.core.module import GuiBase


class CavityControlGui(GuiBase):
    # Use the logic name as configured in your .cfg (string interface is fine in Qudi)
    logic = Connector(name='logic', interface='CavityControlLogic')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mw = None  # QMainWindow instance
        # Widgets we need to reference later
        self.alpha_edit = None
        self.ao1_low_edit = None
        self.ao1_high_edit = None
        self.ao1_speed_edit = None
        self.dec_edit = None
        self.coarse_start = None
        self.coarse_end = None
        self.btn_start = None
        self.btn_stop = None
        self.progress = None
        self.apply_val = None
        self.btn_apply = None
        self.chk_drift = None
        self.center_pct = None
        self.drift_speed = None
        self.deadband_uv = None

    def on_activate(self):
        # Build and wire the main window
        self._mw = QtWidgets.QMainWindow()
        self._mw.setWindowTitle('Cavity Control')
        self._build_ui()
        self._connect_signals()
        self.show()

    def on_deactivate(self):
        # Close the main window if still open
        if self._mw is not None:
            try:
                self._mw.close()
            except Exception:
                pass
            self._mw = None

    def show(self):
        """
        Required by Qudi: show the top-level GUI and bring it to front.
        """
        if self._mw is None:
            # If somehow not built, build now
            self.on_activate()
        try:
            self._mw.show()
            self._mw.raise_()
            self._mw.activateWindow()
        except Exception:
            pass

    def _build_ui(self):
        # Central widget and layout
        central = QtWidgets.QWidget(self._mw)
        lay = QtWidgets.QVBoxLayout(central)

        # Params
        params = QtWidgets.QGroupBox('Parameters', central)
        p_lay = QtWidgets.QFormLayout(params)
        self.alpha_edit = QtWidgets.QDoubleSpinBox(params)
        self.alpha_edit.setRange(0.1, 1000.0)
        self.alpha_edit.setValue(self.logic().alpha)

        self.ao1_low_edit = QtWidgets.QDoubleSpinBox(params)
        self.ao1_low_edit.setRange(-100.0, 100.0)
        self.ao1_low_edit.setValue(self.logic().ao1_low_v)

        self.ao1_high_edit = QtWidgets.QDoubleSpinBox(params)
        self.ao1_high_edit.setRange(-100.0, 100.0)
        self.ao1_high_edit.setValue(self.logic().ao1_high_v)

        self.ao1_speed_edit = QtWidgets.QDoubleSpinBox(params)
        self.ao1_speed_edit.setRange(1e-3, 100.0)
        self.ao1_speed_edit.setValue(self.logic().ao1_speed_vps)

        self.dec_edit = QtWidgets.QSpinBox(params)
        self.dec_edit.setRange(1, 65536)
        self.dec_edit.setValue(self.logic().osc_decimation)

        p_lay.addRow('Alpha', self.alpha_edit)
        p_lay.addRow('AO1 low (V)', self.ao1_low_edit)
        p_lay.addRow('AO1 high (V)', self.ao1_high_edit)
        p_lay.addRow('AO1 speed (V/s)', self.ao1_speed_edit)
        p_lay.addRow('Osc decimation', self.dec_edit)

        # Full scan
        fs = QtWidgets.QGroupBox('Full Scan', central)
        fs_lay = QtWidgets.QHBoxLayout(fs)
        self.coarse_start = QtWidgets.QDoubleSpinBox(fs)
        self.coarse_start.setRange(-1000.0, 1000.0)
        self.coarse_end = QtWidgets.QDoubleSpinBox(fs)
        self.coarse_end.setRange(-1000.0, 1000.0)
        # Use the window's style to fetch standard icons
        play_icon = self._mw.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay)
        stop_icon = self._mw.style().standardIcon(QtWidgets.QStyle.SP_MediaStop)
        self.btn_start = QtWidgets.QPushButton(play_icon, 'Start', fs)
        self.btn_stop = QtWidgets.QPushButton(stop_icon, 'Stop', fs)
        self.progress = QtWidgets.QProgressBar(fs)
        fs_lay.addWidget(QtWidgets.QLabel('Coarse start (V)', fs))
        fs_lay.addWidget(self.coarse_start)
        fs_lay.addWidget(QtWidgets.QLabel('Coarse end (V)', fs))
        fs_lay.addWidget(self.coarse_end)
        fs_lay.addWidget(self.btn_start)
        fs_lay.addWidget(self.btn_stop)
        fs_lay.addWidget(self.progress)

        # Apply coarse voltage
        ac = QtWidgets.QGroupBox('Apply Coarse Voltage', central)
        ac_lay = QtWidgets.QHBoxLayout(ac)
        self.apply_val = QtWidgets.QDoubleSpinBox(ac)
        self.apply_val.setRange(-1000.0, 1000.0)
        self.btn_apply = QtWidgets.QPushButton('Apply', ac)
        ac_lay.addWidget(QtWidgets.QLabel('Coarse (V)', ac))
        ac_lay.addWidget(self.apply_val)
        ac_lay.addWidget(self.btn_apply)

        # Drift tracker
        dt = QtWidgets.QGroupBox('Drift Tracker', central)
        dt_lay = QtWidgets.QFormLayout(dt)
        self.chk_drift = QtWidgets.QCheckBox('Enable', dt)
        self.center_pct = QtWidgets.QDoubleSpinBox(dt)
        self.center_pct.setRange(0.0, 100.0)
        self.center_pct.setValue(self.logic().drift_center_pct)
        self.drift_speed = QtWidgets.QDoubleSpinBox(dt)
        self.drift_speed.setRange(1e-4, 1.0)
        self.drift_speed.setValue(self.logic().drift_speed_vps)
        self.deadband_uv = QtWidgets.QDoubleSpinBox(dt)
        self.deadband_uv.setRange(0.0, 100000.0)
        self.deadband_uv.setValue(self.logic().drift_deadband_uv)
        dt_lay.addRow(self.chk_drift)
        dt_lay.addRow('Center (%)', self.center_pct)
        dt_lay.addRow('Speed (V/s)', self.drift_speed)
        dt_lay.addRow('Deadband (µV)', self.deadband_uv)

        # Assemble
        lay.addWidget(params)
        lay.addWidget(fs)
        lay.addWidget(ac)
        lay.addWidget(dt)
        self._mw.setCentralWidget(central)

    def _connect_signals(self):
        # Param changes
        self.alpha_edit.valueChanged.connect(lambda v: self.logic().set_alpha(float(v)))

        def apply_ao1_params():
            self.logic().set_ao1_params(float(self.ao1_low_edit.value()),
                                        float(self.ao1_high_edit.value()),
                                        float(self.ao1_speed_edit.value()))
        self.ao1_low_edit.editingFinished.connect(apply_ao1_params)
        self.ao1_high_edit.editingFinished.connect(apply_ao1_params)
        self.ao1_speed_edit.editingFinished.connect(apply_ao1_params)

        self.dec_edit.valueChanged.connect(lambda v: self.logic().set_osc_params(int(v), int(self.logic().osc_trigger_source)))

        # Actions
        self.btn_apply.clicked.connect(lambda: self.logic().apply_coarse_voltage(float(self.apply_val.value())))
        self.btn_start.clicked.connect(lambda: self.logic().full_scan(float(self.coarse_start.value()), float(self.coarse_end.value())))
        self.chk_drift.toggled.connect(self.logic().enable_drift)

        # Logic signals
        self.logic().sigFullScanProgress.connect(self._on_scan_progress)
        self.logic().sigFullScanDone.connect(self._on_scan_done)
        self.logic().sigCoarseVoltageSet.connect(lambda v: None)
        self.logic().sigDriftStatus.connect(lambda en, e: None)

    @QtCore.Slot(int, int)
    def _on_scan_progress(self, k: int, n: int):
        self.progress.setMaximum(n)
        self.progress.setValue(k)

    @QtCore.Slot()
    def _on_scan_done(self):
        self.progress.setValue(self.progress.maximum())