# -*- coding: utf-8 -*-
"""
GUI controller for the Vector Magnet.
Loads vector_magnet_gui.ui, wires user actions to logic, shows status & logs.
"""
from __future__ import annotations
from PySide2 import QtWidgets, QtCore, QtGui
from qudi.core.module import Base
from qudi.core.connector import Connector
from qudi.core.module import GuiBase
import math


class VectorMagnetGui(GuiBase):

    logic = Connector(name='logic', interface='VectorMagnetLogic')
    # Internal top-level widget (created on activation)
    _widget: QtWidgets.QWidget | None = None
    _fast_sweep_enabled: bool = False

    # ------------- Required GUI API -------------
    def show(self):
        """
        Required by GuiBase. Show (and raise) the main widget.
        Safe to call multiple times. If activation has not yet produced
        the widget, this method just returns (or could lazily activate).
        """
        if self._widget is None:
            # Either not activated yet or activation failed.
            # Do not call on_activate() here (framework controls lifecycle).
            return
        self._widget.show()
        self._widget.raise_()
        self._widget.activateWindow()

    def hide(self):
        if self._widget is not None:
            self._widget.hide()

    def close(self):
        """
        Optional convenience: close underlying widget.
        """
        if self._widget is not None:
            self._widget.close()

    def on_activate(self):
        # Load the .ui
        loader = QtUiLoaderCompat()
        self._widget = loader.load('vector_magnet_gui.ui')
        self._setup()
        self._widget.show()

    def on_deactivate(self):
        try:
            self._widget.close()
        except Exception:
            pass

    # ---------------- Internal Setup ----------------
    def _setup(self):
        L = self.logic()
        # Connect logic signals
        L.sigFieldReadback.connect(self._update_field_readback)
        L.sigCurrentReadback.connect(self._update_currents)
        L.sigSetpointRejected.connect(self._status_message)
        L.sigSetpointAccepted.connect(self._on_setpoint_accepted)
        L.sigRampProgress.connect(self._on_ramp_progress)
        L.sigModeUpdate.connect(self._on_mode_update)
        L.sigQuenchState.connect(self._on_quench_state)
        L.sigLogEvent.connect(self._append_log)
        L.sigHeaterState.connect(self._on_heater_state)

        # Wire GUI events
        self._widget.applyCartesianButton.clicked.connect(self._apply_cartesian)
        self._widget.applySphericalButton.clicked.connect(self._apply_spherical)
        self._widget.persistentCheckBox.toggled.connect(self._persistent_toggled)
        self._widget.optionBHoldLeadsRadio.toggled.connect(self._idle_behavior_changed)
        self._widget.emergencyStopButton.clicked.connect(L.emergency_stop)
        self._widget.resetQuenchButton.clicked.connect(L.reset_quench)
        self._widget.fastSweepCheckBox.stateChanged.connect(self._fast_sweep_toggled)

        self._widget.setRampRatesButton.clicked.connect(self._set_ramp_rates)

        # Defaults
        self._status_message("Ready.")
        self._widget.resetQuenchButton.setEnabled(False)
        self._fast_sweep_enabled = False

    # ---------------- Actions ----------------
    # Add inside VectorMagnetGui class (e.g. above _apply_cartesian)
    def _val_or_zero(self, line_edit: QtWidgets.QLineEdit) -> float:
        txt = line_edit.text().strip()
        if not txt:
            return 0.0
        try:
            return float(txt.replace(',', '.'))
        except ValueError:
            raise ValueError(f'Invalid number: "{txt}"')

    def _status_message(self, msg: str):
        self._widget.statusLabel.setText(msg)
        print("VectorMagnetGui:", msg)  # DEBUG — remove later

    # Replace the existing _apply_cartesian with this:
    def _apply_cartesian(self):
        try:
            Bx = self._val_or_zero(self._widget.bxEdit)
            By = self._val_or_zero(self._widget.byEdit)
            Bz = self._val_or_zero(self._widget.bzEdit)
        except ValueError:
            self._status_message("Invalid Cartesian field input.")
            return
        # If all three empty -> no-op
        if Bx == By == Bz == 0.0 and not any(
                e.text().strip() for e in (self._widget.bxEdit,
                                           self._widget.byEdit,
                                           self._widget.bzEdit)):
            self._status_message("No values entered.")
            return
        self._status_message(f"Requesting field (mT): {Bx},{By},{Bz}")
        self.logic().request_set_field_cartesian(Bx, By, Bz, fast=self._fast_sweep_enabled)

    def _apply_spherical(self):
        try:
            Bmag = self._val_or_zero(self._widget.bmagEdit)
            theta = self._val_or_zero(self._widget.thetaEdit)
            phi = self._val_or_zero(self._widget.phiEdit)
        except ValueError:
            self._status_message("Invalid spherical field input.")
            return
        self._status_message(f"Requesting spherical |B|={Bmag}mT θ={theta}° φ={phi}°")
        self.logic().request_set_field_spherical(Bmag, theta, phi, fast=self._fast_sweep_enabled)

    def _persistent_toggled(self, checked: bool):
        self.logic().set_persistent_mode(checked)

    def _idle_behavior_changed(self):
        # Option B selection
        if self._widget.optionBHoldLeadsRadio.isChecked():
            self.logic().set_persistent_idle_behavior('hold_leads')
        else:
            self.logic().set_persistent_idle_behavior('zero_leads')

    def _fast_sweep_toggled(self, state: int):
        self._fast_sweep_enabled = (state == QtCore.Qt.Checked)
        self._status_message(f"Fast sweep {'enabled' if self._fast_sweep_enabled else 'disabled'} "
                             "(no automatic heater toggling).")

    def _set_ramp_rates(self):
        try:
            rx = float(self._widget.rampXEdit.text())
            ry = float(self._widget.rampYEdit.text())
            rz = float(self._widget.rampZEdit.text())
        except ValueError:
            self._status_message("Invalid ramp rates.")
            return
        self.logic().set_axis_ramp_rate('x', rx)
        self.logic().set_axis_ramp_rate('y', ry)
        self.logic().set_axis_ramp_rate('z', rz)
        self._status_message("Ramp rates updated.")

    # ---------------- GUI Update Slots ----------------
    def _update_field_readback(self, d):
        self._widget.readBxLabel.setText(f"{d['Bx_mT']:.3f}")
        self._widget.readByLabel.setText(f"{d['By_mT']:.3f}")
        self._widget.readBzLabel.setText(f"{d['Bz_mT']:.3f}")
        self._widget.readBmagLabel.setText(f"{d['Bmag_mT']:.3f}")
        # Update spherical angles from Cartesian
        Bx = d['Bx_mT'] / 1000.0
        By = d['By_mT'] / 1000.0
        Bz = d['Bz_mT'] / 1000.0
        Bmag = d['Bmag_mT'] / 1000.0
        if Bmag > 1e-12:
            # Reconstruct user θ = (π - θ_conv)
            theta_conv = math.acos(Bz / Bmag)
            theta_user = math.degrees(math.pi - theta_conv)
            phi = (math.degrees(math.atan2(By, Bx)) + 360.0) % 360.0
            self._widget.readThetaLabel.setText(f"{theta_user:.2f}")
            self._widget.readPhiLabel.setText(f"{phi:.2f}")
        else:
            self._widget.readThetaLabel.setText("--")
            self._widget.readPhiLabel.setText("--")

    def _update_currents(self, d):
        self._widget.readIxLabel.setText(f"{d['Ix']:.3f}")
        self._widget.readIyLabel.setText(f"{d['Iy']:.3f}")
        self._widget.readIzLabel.setText(f"{d['Iz']:.3f}")

    def _on_setpoint_accepted(self, info: dict):
        self._status_message("Setpoint accepted; ramping...")

    def _on_ramp_progress(self, val: float):
        self._widget.rampProgressBar.setValue(int(val * 100))

    def _on_mode_update(self, info: dict):
        # Could display persistent mode state or idle behavior updates
        pass

    def _on_quench_state(self, info: dict):
        if info['quench']:
            self._status_message(f"QUENCH DETECTED (axes {info['axes']}) – press Reset after safe.")
            self._widget.resetQuenchButton.setEnabled(True)
        else:
            self._status_message("Quench state reset.")
            self._widget.resetQuenchButton.setEnabled(False)

    def _on_heater_state(self, states: dict):
        xy_state = "ON" if states.get('xy', False) else "OFF"
        z_state = "ON" if states.get('z', False) else "OFF"
        self._widget.heaterStatusLabel.setText(f"Heater XY: {xy_state} | Heater Z: {z_state}")

    def _append_log(self, line: str):
        self._widget.logPlain.appendPlainText(line)

    def _status_message(self, msg: str):
        self._widget.statusLabel.setText(msg)

class QtUiLoaderCompat:
    """
    Minimalistic UI loader; in Qudi you can swap with QUiLoader or use uic.loadUiType.
    Ensures objectNames used in code exist in .ui.
    """
    def load(self, ui_path: str):
        # For an actual deployment, replace with QUiLoader logic; here we load via QtWidgets for brevity.
        from PySide2 import QtWidgets
        w = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(w)

        # --- Input Controls (Cartesian) ---
        w.bxEdit = QtWidgets.QLineEdit(); w.bxEdit.setPlaceholderText("Bx (mT)")
        w.byEdit = QtWidgets.QLineEdit(); w.byEdit.setPlaceholderText("By (mT)")
        w.bzEdit = QtWidgets.QLineEdit(); w.bzEdit.setPlaceholderText("Bz (mT)")
        w.applyCartesianButton = QtWidgets.QPushButton("Apply Cartesian")

        # --- Input Controls (Spherical) ---
        w.bmagEdit = QtWidgets.QLineEdit(); w.bmagEdit.setPlaceholderText("|B| (mT)")
        w.thetaEdit = QtWidgets.QLineEdit(); w.thetaEdit.setPlaceholderText("θ (deg, 0=-Z ->180=+Z)")
        w.phiEdit = QtWidgets.QLineEdit(); w.phiEdit.setPlaceholderText("φ (deg)")
        w.applySphericalButton = QtWidgets.QPushButton("Apply Spherical")

        # --- Ramp Rates ---
        w.rampXEdit = QtWidgets.QLineEdit(); w.rampXEdit.setPlaceholderText("Ramp X (A/s)")
        w.rampYEdit = QtWidgets.QLineEdit(); w.rampYEdit.setPlaceholderText("Ramp Y (A/s)")
        w.rampZEdit = QtWidgets.QLineEdit(); w.rampZEdit.setPlaceholderText("Ramp Z (A/s)")
        w.setRampRatesButton = QtWidgets.QPushButton("Set Ramp Rates")
        w.fastSweepCheckBox = QtWidgets.QCheckBox("Fast Sweep (no auto heater toggle)")

        # --- Persistent Mode ---
        w.persistentCheckBox = QtWidgets.QCheckBox("Persistent Mode")
        w.optionAZeroLeadsRadio = QtWidgets.QRadioButton("Idle: Zero Leads")
        w.optionBHoldLeadsRadio = QtWidgets.QRadioButton("Idle: Hold Leads")
        w.optionAZeroLeadsRadio.setChecked(True)

        # --- Readback Labels ---
        w.readBxLabel = QtWidgets.QLabel("--")
        w.readByLabel = QtWidgets.QLabel("--")
        w.readBzLabel = QtWidgets.QLabel("--")
        w.readBmagLabel = QtWidgets.QLabel("--")
        w.readThetaLabel = QtWidgets.QLabel("--")
        w.readPhiLabel = QtWidgets.QLabel("--")
        w.readIxLabel = QtWidgets.QLabel("--")
        w.readIyLabel = QtWidgets.QLabel("--")
        w.readIzLabel = QtWidgets.QLabel("--")

        # --- Heater status ---
        w.heaterStatusLabel = QtWidgets.QLabel("Heater XY: -- | Heater Z: --")

        # --- Ramp Progress / Status ---
        w.rampProgressBar = QtWidgets.QProgressBar()
        w.rampProgressBar.setRange(0, 100)
        w.statusLabel = QtWidgets.QLabel("")

        # --- Quench / Emergency ---
        w.emergencyStopButton = QtWidgets.QPushButton("Emergency Stop")
        w.resetQuenchButton = QtWidgets.QPushButton("Reset Quench")
        w.resetQuenchButton.setEnabled(False)

        # --- Log Pane ---
        w.logPlain = QtWidgets.QPlainTextEdit()
        w.logPlain.setReadOnly(True)

        row = 0
        layout.addWidget(QtWidgets.QLabel("Cartesian (mT):"), row, 0, 1, 2)
        row += 1
        layout.addWidget(w.bxEdit, row, 0); layout.addWidget(w.byEdit, row, 1); layout.addWidget(w.bzEdit, row, 2)
        layout.addWidget(w.applyCartesianButton, row, 3)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Spherical:"), row, 0, 1, 2)
        row += 1
        layout.addWidget(w.bmagEdit, row, 0); layout.addWidget(w.thetaEdit, row, 1); layout.addWidget(w.phiEdit, row, 2)
        layout.addWidget(w.applySphericalButton, row, 3)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Ramp Rates (A/s):"), row, 0)
        row += 1
        layout.addWidget(w.rampXEdit, row, 0); layout.addWidget(w.rampYEdit, row, 1); layout.addWidget(w.rampZEdit, row, 2)
        layout.addWidget(w.setRampRatesButton, row, 3)
        row += 1
        layout.addWidget(w.fastSweepCheckBox, row, 0, 1, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Persistent Mode:"), row, 0)
        row += 1
        layout.addWidget(w.persistentCheckBox, row, 0)
        layout.addWidget(w.optionAZeroLeadsRadio, row, 1)
        layout.addWidget(w.optionBHoldLeadsRadio, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Field Readback (mT): Bx By Bz |B|"), row, 0, 1, 4)
        row += 1
        layout.addWidget(w.readBxLabel, row, 0); layout.addWidget(w.readByLabel, row, 1)
        layout.addWidget(w.readBzLabel, row, 2); layout.addWidget(w.readBmagLabel, row, 3)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Angles (deg): θ φ"), row, 0, 1, 2)
        row += 1
        layout.addWidget(w.readThetaLabel, row, 0); layout.addWidget(w.readPhiLabel, row, 1)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Currents (A): Ix Iy Iz"), row, 0, 1, 3)
        row += 1
        layout.addWidget(w.readIxLabel, row, 0); layout.addWidget(w.readIyLabel, row, 1); layout.addWidget(w.readIzLabel, row, 2)
        row += 1
        layout.addWidget(w.heaterStatusLabel, row, 0, 1, 3)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Ramp Progress:"), row, 0)
        layout.addWidget(w.rampProgressBar, row, 1, 1, 3)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Status:"), row, 0)
        layout.addWidget(w.statusLabel, row, 1, 1, 3)
        row += 1
        layout.addWidget(w.emergencyStopButton, row, 0)
        layout.addWidget(w.resetQuenchButton, row, 1)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Log:"), row, 0)
        row += 1
        layout.addWidget(w.logPlain, row, 0, 1, 4)

        return w