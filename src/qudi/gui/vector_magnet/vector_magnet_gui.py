# -*- coding: utf-8 -*-
"""
This file contains a gui for the vector magnet logic.

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
from PySide2 import QtWidgets, QtCore
from qudi.core.module import GuiBase
from qudi.core.connector import Connector
import math


class VectorMagnetGui(GuiBase):
    """Gui module to monitor and control the vector magnet.

    Example Config:

    vector_magnet_gui:
        module.Class: 'vector_magnet.vector_magnet_gui.VectorMagnetGui'
        connect:
            logic: vector_magnet_logic

    """

    logic = Connector(name='logic', interface='VectorMagnetLogic')
    _widget: QtWidgets.QWidget | None = None
    _fast_sweep_enabled = False

    # ------------- Lifecycle -------------

    def on_activate(self):
        builder = _QtUiBuilder()
        self._widget = builder.build()
        self._connect_logic()
        self._widget.show()
        self._status_message("Ready.")

    def on_deactivate(self):
        if self._widget:
            try:
                self._widget.close()
            except Exception:
                pass

    def show(self):
        if self._widget:
            self._widget.show()
            self._widget.raise_()
            self._widget.activateWindow()

    def hide(self):
        if self._widget:
            self._widget.hide()

    def close(self):
        if self._widget:
            self._widget.close()

    # ------------- Logic Connections -------------

    def _connect_logic(self):
        L = self.logic()
        w = self._widget

        # Signals
        L.sigFieldReadback.connect(self._update_field_readback)
        L.sigCurrentReadback.connect(self._update_currents)
        L.sigSetpointRejected.connect(self._status_message)
        L.sigRampProgress.connect(self._on_ramp_progress)
        L.sigQuenchState.connect(self._on_quench_state)
        L.sigLogEvent.connect(self._append_log)
        L.sigHeaterState.connect(self._on_heater_state)
        L.sigStatusText.connect(self._status_message)

        # Actions
        w.applyCartesianButton.clicked.connect(self._apply_cartesian)
        w.applySphericalButton.clicked.connect(self._apply_spherical)
        w.zeroAllButton.clicked.connect(self._zero_all)
        w.fastSweepCheckBox.stateChanged.connect(self._fast_sweep_toggled)
        w.setRampRatesButton.clicked.connect(self._set_ramp_rates)
        w.emergencyStopButton.clicked.connect(L.emergency_stop)
        w.resetQuenchButton.clicked.connect(L.reset_quench)
        w.persistentCheckBox.toggled.connect(L.set_persistent_mode)
        w.optionBHoldLeadsRadio.toggled.connect(self._idle_behavior_changed)

    # ------------- Helpers -------------

    @staticmethod
    def _val_or_zero(edit: QtWidgets.QLineEdit) -> float:
        txt = edit.text().strip()
        if not txt:
            return 0.0
        try:
            return float(txt.replace(',', '.'))
        except ValueError:
            raise ValueError(f"Invalid number: {txt}")

    def _status_message(self, msg: str):
        self._widget.statusLabel.setText(msg)

    # ------------- Actions -------------

    def _apply_cartesian(self):
        w = self._widget
        try:
            bx = self._val_or_zero(w.bxEdit)
            by = self._val_or_zero(w.byEdit)
            bz = self._val_or_zero(w.bzEdit)
        except ValueError:
            self._status_message("Invalid Cartesian input.")
            return
        self.logic().request_set_field_cartesian(bx, by, bz, fast=self._fast_sweep_enabled)
        #Status handled by logic via sigStatusText

    def _apply_spherical(self):
        w = self._widget
        try:
            bmag = self._val_or_zero(w.bmagEdit)
            theta = self._val_or_zero(w.thetaEdit)
            phi = self._val_or_zero(w.phiEdit)
        except ValueError:
            self._status_message("Invalid spherical input.")
            return
        self.logic().request_set_field_spherical(bmag, theta, phi, fast=self._fast_sweep_enabled)
        # Status handled by logic via sigStatusText

    def _zero_all(self):
        self.logic().zero_all(fast=False)
        self._status_message("Zero-all sweep issued.")

    def _fast_sweep_toggled(self, state: int):
        self._fast_sweep_enabled = (state == QtCore.Qt.Checked)
        self._status_message(f"Fast sweep {'ON' if self._fast_sweep_enabled else 'OFF'}")

    def _idle_behavior_changed(self):
        if self._widget.optionBHoldLeadsRadio.isChecked():
            self.logic().set_persistent_idle_behavior('hold_leads')
        else:
            self.logic().set_persistent_idle_behavior('zero_leads')

    def _set_ramp_rates(self):
        w = self._widget
        try:
            rx = float(w.rampXEdit.text() or 0.0)
            ry = float(w.rampYEdit.text() or 0.0)
            rz = float(w.rampZEdit.text() or 0.0)
        except ValueError:
            self._status_message("Invalid ramp rates.")
            return
        L = self.logic()
        L.set_axis_ramp_rate('x', rx)
        L.set_axis_ramp_rate('y', ry)
        L.set_axis_ramp_rate('z', rz)
        self._status_message("Ramp rates updated.")

    # ------------- Slots / Updates -------------

    def _update_field_readback(self, d: dict):
        w = self._widget
        w.readBxLabel.setText(f"{d['Bx_mT']:.3f}")
        w.readByLabel.setText(f"{d['By_mT']:.3f}")
        w.readBzLabel.setText(f"{d['Bz_mT']:.3f}")
        w.readBmagLabel.setText(f"{d['Bmag_mT']:.3f}")

        # Derive spherical angles
        Bx = d['Bx_mT'] / 1000.0
        By = d['By_mT'] / 1000.0
        Bz = d['Bz_mT'] / 1000.0
        Bmag = d['Bmag_mT'] / 1000.0
        if Bmag > 1e-12:
            theta_conv = math.acos(Bz / Bmag)
            theta_user = math.degrees(math.pi - theta_conv)
            phi = (math.degrees(math.atan2(By, Bx)) + 360.0) % 360.0
            w.readThetaLabel.setText(f"{theta_user:.2f}")
            w.readPhiLabel.setText(f"{phi:.2f}")
        else:
            w.readThetaLabel.setText("--")
            w.readPhiLabel.setText("--")

    def _update_currents(self, d: dict):
        w = self._widget
        w.readIxLabel.setText(f"{d['Ix']:.4f}")
        w.readIyLabel.setText(f"{d['Iy']:.4f}")
        w.readIzLabel.setText(f"{d['Iz']:.4f}")

    def _on_ramp_progress(self, val: float):
        self._widget.rampProgressBar.setValue(int(round(val * 100)))

    def _on_quench_state(self, info: dict):
        if info['quench']:
            self._status_message(f"QUENCH DETECTED axes={info['axes']}")
            self._widget.resetQuenchButton.setEnabled(True)
        else:
            self._status_message("Quench reset.")
            self._widget.resetQuenchButton.setEnabled(False)

    def _on_heater_state(self, states: dict):
        w = self._widget
        w.hxStatusLabel.setText("ON" if states.get('x') else "OFF")
        w.hyStatusLabel.setText("ON" if states.get('y') else "OFF")
        w.hzStatusLabel.setText("ON" if states.get('z') else "OFF")

    def _append_log(self, line: str):
        self._widget.logPlain.appendPlainText(line)


class _QtUiBuilder:
    """Programmatic GUI builder (3-column layout)."""

    def build(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        w.setWindowTitle("Vector Magnet")
        outer = QtWidgets.QVBoxLayout(w)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(10)

        # Set Field Group
        set_group = QtWidgets.QGroupBox("Set Field")
        sg = QtWidgets.QGridLayout(set_group)
        row = 0

        # Cartesian header
        cart_header = QtWidgets.QLabel("Cartesian (B_x, B_y, B_z) [mT]")
        w.applyCartesianButton = QtWidgets.QPushButton("Apply")
        sg.addWidget(cart_header, row, 0, 1, 2)
        sg.addWidget(w.applyCartesianButton, row, 2); row += 1

        w.bxEdit = QtWidgets.QLineEdit(); w.bxEdit.setPlaceholderText("B_x")
        w.byEdit = QtWidgets.QLineEdit(); w.byEdit.setPlaceholderText("B_y")
        w.bzEdit = QtWidgets.QLineEdit(); w.bzEdit.setPlaceholderText("B_z")
        sg.addWidget(w.bxEdit, row, 0); sg.addWidget(w.byEdit, row, 1); sg.addWidget(w.bzEdit, row, 2); row += 1

        # Spherical header
        sph_header = QtWidgets.QLabel("Spherical (|B|, θ, φ) [mT, °, °]")
        w.applySphericalButton = QtWidgets.QPushButton("Apply")
        sg.addWidget(sph_header, row, 0, 1, 2)
        sg.addWidget(w.applySphericalButton, row, 2); row += 1

        w.bmagEdit = QtWidgets.QLineEdit(); w.bmagEdit.setPlaceholderText("|B|")
        w.thetaEdit = QtWidgets.QLineEdit(); w.thetaEdit.setPlaceholderText("θ (deg)")
        w.phiEdit = QtWidgets.QLineEdit(); w.phiEdit.setPlaceholderText("φ (deg)")
        sg.addWidget(w.bmagEdit, row, 0); sg.addWidget(w.thetaEdit, row, 1); sg.addWidget(w.phiEdit, row, 2); row += 1

        # Zero All
        w.zeroAllButton = QtWidgets.QPushButton("Zero All")
        sg.addWidget(w.zeroAllButton, row, 0, 1, 3); row += 1

        outer.addWidget(set_group)

        # Readback Group
        rb_group = QtWidgets.QGroupBox("Readback")
        rgb = QtWidgets.QGridLayout(rb_group)
        row = 0

        rgb.addWidget(QtWidgets.QLabel("Cartesian (B_x, B_y, B_z) [mT]:"), row, 0, 1, 3); row += 1
        w.readBxLabel = self._val_label(); w.readByLabel = self._val_label(); w.readBzLabel = self._val_label()
        rgb.addWidget(w.readBxLabel, row, 0); rgb.addWidget(w.readByLabel, row, 1); rgb.addWidget(w.readBzLabel, row, 2); row += 1

        rgb.addWidget(QtWidgets.QLabel("Spherical (|B|, θ, φ) [mT, °, °]:"), row, 0, 1, 3); row += 1
        w.readBmagLabel = self._val_label(); w.readThetaLabel = self._val_label(); w.readPhiLabel = self._val_label()
        rgb.addWidget(w.readBmagLabel, row, 0); rgb.addWidget(w.readThetaLabel, row, 1); rgb.addWidget(w.readPhiLabel, row, 2); row += 1

        rgb.addWidget(QtWidgets.QLabel("Currents (I_x, I_y, I_z) [A]:"), row, 0, 1, 3); row += 1
        w.readIxLabel = self._val_label(); w.readIyLabel = self._val_label(); w.readIzLabel = self._val_label()
        rgb.addWidget(w.readIxLabel, row, 0); rgb.addWidget(w.readIyLabel, row, 1); rgb.addWidget(w.readIzLabel, row, 2); row += 1

        outer.addWidget(rb_group)

        # Persistent Mode Group
        pm_group = QtWidgets.QGroupBox("Persistent Mode")
        pmg = QtWidgets.QGridLayout(pm_group)
        row = 0
        w.persistentCheckBox = QtWidgets.QCheckBox("Persistent")
        w.optionAZeroLeadsRadio = QtWidgets.QRadioButton("Idle Zero Leads")
        w.optionBHoldLeadsRadio = QtWidgets.QRadioButton("Idle Hold Leads")
        w.optionAZeroLeadsRadio.setChecked(True)
        pmg.addWidget(w.persistentCheckBox, row, 0)
        pmg.addWidget(w.optionAZeroLeadsRadio, row, 1)
        pmg.addWidget(w.optionBHoldLeadsRadio, row, 2); row += 1

        pmg.addWidget(QtWidgets.QLabel("Heater Status (H_x, H_y, H_z):"), row, 0, 1, 3); row += 1
        w.hxStatusLabel = self._val_label(); w.hyStatusLabel = self._val_label(); w.hzStatusLabel = self._val_label()
        pmg.addWidget(w.hxStatusLabel, row, 0); pmg.addWidget(w.hyStatusLabel, row, 1); pmg.addWidget(w.hzStatusLabel, row, 2); row += 1

        outer.addWidget(pm_group)

        # Ramp / Sweep Group
        rs_group = QtWidgets.QGroupBox("Ramp / Sweep")
        rsg = QtWidgets.QGridLayout(rs_group)
        row = 0

        header = QtWidgets.QLabel("Ramp Rates [A/s]")
        w.setRampRatesButton = QtWidgets.QPushButton("Set")
        rsg.addWidget(header, row, 0, 1, 2)
        rsg.addWidget(w.setRampRatesButton, row, 2); row += 1

        w.rampXEdit = QtWidgets.QLineEdit(); w.rampXEdit.setPlaceholderText("X")
        w.rampYEdit = QtWidgets.QLineEdit(); w.rampYEdit.setPlaceholderText("Y")
        w.rampZEdit = QtWidgets.QLineEdit(); w.rampZEdit.setPlaceholderText("Z")
        rsg.addWidget(w.rampXEdit, row, 0); rsg.addWidget(w.rampYEdit, row, 1); rsg.addWidget(w.rampZEdit, row, 2); row += 1

        w.fastSweepCheckBox = QtWidgets.QCheckBox("Fast Sweep")
        rsg.addWidget(w.fastSweepCheckBox, row, 0, 1, 3); row += 1

        w.rampProgressBar = QtWidgets.QProgressBar(); w.rampProgressBar.setRange(0, 100)
        rsg.addWidget(QtWidgets.QLabel("Progress:"), row, 0)
        rsg.addWidget(w.rampProgressBar, row, 1, 1, 2); row += 1

        w.statusLabel = QtWidgets.QLabel("")
        rsg.addWidget(QtWidgets.QLabel("Status:"), row, 0)
        rsg.addWidget(w.statusLabel, row, 1, 1, 2); row += 1

        w.emergencyStopButton = QtWidgets.QPushButton("Emergency Stop")
        w.resetQuenchButton = QtWidgets.QPushButton("Reset Quench"); w.resetQuenchButton.setEnabled(False)
        rsg.addWidget(w.emergencyStopButton, row, 0, 1, 2)
        rsg.addWidget(w.resetQuenchButton, row, 2); row += 1

        outer.addWidget(rs_group)

        # Log Group
        log_group = QtWidgets.QGroupBox("Log")
        lg = QtWidgets.QVBoxLayout(log_group)
        w.logPlain = QtWidgets.QPlainTextEdit(); w.logPlain.setReadOnly(True)
        w.logPlain.setMinimumHeight(200)
        lg.addWidget(w.logPlain)
        outer.addWidget(log_group, stretch=1)

        return w

    @staticmethod
    def _val_label() -> QtWidgets.QLabel:
        lbl = QtWidgets.QLabel("--")
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        lbl.setMinimumWidth(70)
        return lbl