# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Tuple, Optional


from qudi.core.connector import Connector
from PySide2 import QtCore, QtWidgets
from qudi.core.module import GuiBase, LogicBase


class RPLockInPIDHMainWindow(QtWidgets.QMainWindow):
    """
    Sehr einfache Haupt-GUI für den RedPitaya Lock-In PID (H):
    Drei Spinboxen direkt im MainWindow (keine Subwidgets).
    """

    sig_ramp_values_changed = QtCore.Signal(float)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)

        self.setWindowTitle('RedPitaya Lock-In PID (H)')

        # Zentrales Widget + möglichst einfaches Layout
        central_widget = QtWidgets.QWidget(self)
        layout = QtWidgets.QFormLayout(central_widget)
        layout.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        self.setCentralWidget(central_widget)

        # Scan-Frequenz
        self.scan_frequency_spin = QtWidgets.QDoubleSpinBox(self)
        self.scan_frequency_spin.setRange(0.1, 100.0)
        self.scan_frequency_spin.setSingleStep(0.1)
        self.scan_frequency_spin.setDecimals(1)
        self.scan_frequency_spin.setValue(30.0)
        self.scan_frequency_spin.setSuffix(' Hz')
        layout.addRow('Scan Frequency', self.scan_frequency_spin)

        # Scan-Offset (V)
        self.scan_offset_spin = QtWidgets.QDoubleSpinBox(self)
        self.scan_offset_spin.setRange(-1.0, 1.0)
        self.scan_offset_spin.setSingleStep(0.025)
        self.scan_offset_spin.setDecimals(3)
        self.scan_offset_spin.setValue(0.0)
        self.scan_offset_spin.setSuffix(' V')
        layout.addRow('Scan Offset', self.scan_offset_spin)

        # Scan-Amplitude (V)
        self.scan_amplitude_spin = QtWidgets.QDoubleSpinBox(self)
        self.scan_amplitude_spin.setRange(0.01, 2.0)
        self.scan_amplitude_spin.setSingleStep(0.025)
        self.scan_amplitude_spin.setDecimals(3)
        self.scan_amplitude_spin.setValue(2.0)
        self.scan_amplitude_spin.setSuffix(' V')
        layout.addRow('Scan Amplitude', self.scan_amplitude_spin)

        # full range button
        self.full_range_button = QtWidgets.QPushButton('Full Range', self)
        layout.addRow(self.full_range_button)
        self.full_range_button.clicked.connect(self._set_full_range)

        # Signale nach außen
        self.scan_frequency_spin.valueChanged.connect(
            self.sig_ramp_values_changed.emit
        )
        self.scan_offset_spin.valueChanged.connect(
            self.sig_ramp_values_changed.emit
        )
        self.scan_amplitude_spin.valueChanged.connect(
            self.sig_ramp_values_changed.emit
        )

    def _set_full_range(self) -> None:
        """Set offset and amplitude to use the full range of -1V to +1V."""
        self.scan_offset_spin.setValue(0.0)
        self.scan_amplitude_spin.setValue(2.0)
        self.sig_ramp_values_changed.emit(69.0)

    # def closeEvent(self, event: QtGui.QCloseEvent) -> None:
    #     self.sig_closed.emit()
    #     super().closeEvent(event)


class RPLockInPIDHGui(GuiBase):
    """
    Qudi-GUI-Modul für den RedPitaya Lock-In PID (H).
    Bindet das MainWindow an die Logik an und schreibt Register
    über _set_lock_register.
    """

    rp_logic = Connector(name = 'rp_logic', interface = LogicBase)

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self._logic = None
        self._mw = RPLockInPIDHMainWindow()

        # Verbindungen der MainWindow-Signale zu den Handlern
        self._mw.sig_ramp_values_changed.connect(
            self._handle_ramp_values_changed
        )
        
    def show(self):
        """Make window visible and put it above all other windows.
        """
        self._mw.show()
        self._mw.raise_()
        self._mw.activateWindow()

    def on_activate(self) -> None:
        """Beim Aktivieren Logic auflösen und Fenster anzeigen."""
        self._logic = self.rp_logic()
        self._handle_ramp_values_changed(69.0)
        self._mw.show()

    def on_deactivate(self) -> None:
        """Beim Deaktivieren Fenster verstecken und Logic-Referenz löschen."""
        self._mw.hide()
        self._logic = None

    def _handle_ramp_values_changed(self, flo) -> None:

        freq = self._mw.scan_frequency_spin.value()
        offset = self._mw.scan_offset_spin.value()
        amplitude = self._mw.scan_amplitude_spin.value()

        ramp_hig_lim_value = int(min((offset + amplitude / 2) * 8192, 8191))
        ramp_low_lim_value = int(max((offset - amplitude / 2) * 8192, -8191))
        ramp_delta = ramp_hig_lim_value - ramp_low_lim_value
        ramp_step_value = int(1/(ramp_delta * freq * 16e-9) - 1)

        if self._logic is None:
            return

        self._logic.set_lock_register('ramp_low_lim', ramp_low_lim_value)
        self._logic.set_lock_register('ramp_hig_lim', ramp_hig_lim_value)
        self._logic.set_lock_register('ramp_step', ramp_step_value)

