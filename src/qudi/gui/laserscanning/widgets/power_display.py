# -*- coding: utf-8 -*-
"""
Dock widget for displaying laser power (from an optional powermeter).
"""

__all__ = ["LaserPowerDisplayDockWidget", "LaserPowerDisplayWidget"]

import math
from PySide6 import QtCore, QtWidgets


def _format_power_auto_unit(power_w: float | None, digits: int = 4) -> tuple[str, str]:
    """
    Format power given in W into a human-friendly value + unit.
    Chooses among W, mW, µW, nW, pW dynamically.
    """
    if power_w is None:
        return "—", "W"
    try:
        power_w = float(power_w)
    except Exception:
        return "—", "W"

    if math.isnan(power_w) or math.isinf(power_w):
        return "—", "W"

    abs_w = abs(power_w)
    if abs_w == 0:
        return f"{0:.{digits}g}", "W"

    units = [
        ("W", 1.0),
        ("mW", 1e3),
        ("µW", 1e6),
        ("nW", 1e9),
        ("pW", 1e12),
    ]

    chosen_unit, chosen_factor = units[0]
    for unit, factor in units:
        val = abs_w * factor
        chosen_unit, chosen_factor = unit, factor
        if 1.0 <= val < 1000.0:
            break

    disp = power_w * chosen_factor
    return f"{disp:.{digits}g}", chosen_unit


class LaserPowerDisplayWidget(QtWidgets.QWidget):
    """A tiny widget that only displays the current laser power."""

    sigToggleAcquisitionRequested = QtCore.Signal()

    def __init__(self, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(10)

        self._label_name = QtWidgets.QLabel("Power:")
        self._label_name.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

        # Large numeric value
        self._lcd_value = QtWidgets.QLCDNumber()
        self._lcd_value.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self._lcd_value.setSmallDecimalPoint(True)
        self._lcd_value.setDigitCount(6)  # e.g. "999.9999"
        self._lcd_value.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                      QtWidgets.QSizePolicy.Expanding)
        self._lcd_value.setMinimumHeight(40)

        self._label_unit = QtWidgets.QLabel("--")
        self._label_unit.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        # NEW: start/stop button
        self._btn_toggle = QtWidgets.QPushButton("Start")
        self._btn_toggle.setCheckable(True)
        self._btn_toggle.clicked.connect(self._toggle_clicked)
        self._btn_toggle.setSizePolicy(QtWidgets.QSizePolicy.Minimum,
                                       QtWidgets.QSizePolicy.Preferred)

        layout.addWidget(self._label_name)
        layout.addWidget(self._lcd_value, stretch = 1)
        layout.addWidget(self._label_unit)
        layout.addWidget(self._btn_toggle)

        self.setLayout(layout)

        # reflect initial state
        self.set_running(False)
        self.set_power_w(None)

    def _toggle_clicked(self) -> None:
        self.sigToggleAcquisitionRequested.emit()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_unit_font()

    def _update_unit_font(self) -> None:
        # Scale unit font with LCD height
        pt = max(10, min(48, int(self._lcd_value.height() * 0.35)))
        f = self._label_unit.font()
        if f.pointSize() != pt:
            f.setPointSize(pt)
            self._label_unit.setFont(f)

    @QtCore.Slot(object)
    def set_power_w(self, power_w: float | None) -> None:
        if power_w is None:
            self._lcd_value.display("OFF")
            self._label_unit.setText("--")
            self._update_unit_font()
            return

        val_txt, unit = _format_power_auto_unit(power_w, digits=4)
        unit = unit.replace("µW", "uW")

        try:
            val = float(val_txt)
        except Exception:
            self._lcd_value.display("ERR")
            self._label_unit.setText("--")
            self._update_unit_font()
            return

        disp = f"{val:.4f}".rstrip("0").rstrip(".")
        if disp == "":
            disp = "0"

        self._lcd_value.display(disp)
        self._label_unit.setText(unit)
        self._update_unit_font()


    @QtCore.Slot(bool)
    def set_running(self, running: bool) -> None:
        running = bool(running)
        self._btn_toggle.blockSignals(True)
        self._btn_toggle.setChecked(running)
        self._btn_toggle.setText("Stop" if running else "Start")
        if not running:
            self._lcd_value.display('OFF')
            self._label_unit.setText("--")
        self._btn_toggle.blockSignals(False)

class LaserPowerDisplayDockWidget(QtWidgets.QDockWidget):
    """Dock widget wrapper so it can be dragged/floated like the others."""

    def __init__(self, title: str, parent: QtWidgets.QWidget | None = None):
        super().__init__(title, parent)
        self.display_widget = LaserPowerDisplayWidget()
        self.setWidget(self.display_widget)

        # Optional: make it behave like the other docks
        self.setAllowedAreas(
            QtCore.Qt.LeftDockWidgetArea
            | QtCore.Qt.RightDockWidgetArea
            | QtCore.Qt.TopDockWidgetArea
            | QtCore.Qt.BottomDockWidgetArea
        )