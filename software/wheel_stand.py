import sys
import math
import random
import csv
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports
import json

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QFileDialog, 
    QMessageBox, QSizePolicy, QLabel, QWidget, QPushButton
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont
import pyqtgraph as pg

from ui_main_window import Ui_MainWindow


# Global modern dark industrial stylesheet (QSS)
DARK_STYLE = """
QMainWindow {
    background-color: #1a1a24;
}

QWidget {
    color: #e0e0e0;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
}

/* Top container: Actual data box */
QGroupBox {
    border: 1px solid #33334d;
    border-radius: 8px;
    margin-top: 12px;
    font-weight: bold;
    color: #94a3b8;
    background-color: #20202e;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 4px;
}

/* Plot container frames */
#plotForces, #plotDistance {
    background-color: #20202e;
    border: 1px solid #33334d;
    border-radius: 8px;
}

/* Dropdown menu */
QComboBox {
    background-color: #2b2b3d;
    border: 1px solid #454566;
    border-radius: 6px;
    padding: 6px 10px;
    min-width: 140px;
    min-height: 22px;
    font-weight: 500;
}
QComboBox:hover {
    border-color: #6272a4;
}
QComboBox::drop-down {
    border: none;
}
QComboBox QAbstractItemView {
    background-color: #2b2b3d;
    selection-background-color: #44475a;
    border: 1px solid #454566;
}

/* General push button styling */
QPushButton {
    border-radius: 6px;
    padding: 6px 12px;
    font-weight: bold;
    min-width: 90px;
    min-height: 28px;
}

/* Specific button styling */
#connectBtn {
    background-color: #3b82f6;
    color: white;
}
#connectBtn:hover {
    background-color: #2563eb;
}

#startBtn {
    background-color: #10b981;
    color: white;
}
#startBtn:hover {
    background-color: #059669;
}
#startBtn:disabled {
    background-color: #252836;
    color: #64748b;
}

#stopBtn {
    background-color: #ef4444;
    color: white;
}
#stopBtn:hover {
    background-color: #dc2626;
}
#stopBtn:disabled {
    background-color: #252836;
    color: #64748b;
}

#exportBtn {
    background-color: #0284c7;
    color: white;
    min-width: 110px;
}
#exportBtn:hover {
    background-color: #0369a1;
}

#clearBtn {
    background-color: #4b5563;
    color: white;
    min-width: 110px;
}
#clearBtn:hover {
    background-color: #374151;
}

#clearMarkersBtn {
    background-color: #374151;
    color: #e2e8f0;
    min-width: 105px;
}
#clearMarkersBtn:hover {
    background-color: #4b5563;
}

/* LCD Displays */
QLCDNumber {
    background-color: #12121a;
    border: 1px solid #3b3b54;
    border-radius: 6px;
    color: #38bdf8;
    min-height: 32px;
    min-width: 75px;
}

/* Status Bar */
QStatusBar {
    background-color: #14141c;
    color: #a0aec0;
    border-top: 1px solid #2d3748;
}

/* Dialog and Message Box styling */
QMessageBox {
    background-color: #20202e;
}

QMessageBox QLabel {
    color: #f1f5f9;
    font-size: 13px;
    font-weight: 500;
}

QMessageBox QPushButton {
    background-color: #3b82f6;
    color: white;
    border-radius: 6px;
    padding: 6px 18px;
    min-width: 70px;
}

QMessageBox QPushButton:hover {
    background-color: #2563eb;
}
"""


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        if self.centralwidget.layout() is None:
            main_layout = QVBoxLayout(self.centralwidget)
            main_layout.setContentsMargins(16, 12, 16, 12)
            main_layout.setSpacing(12)
            
            # 1. Top container: Actual data box
            if hasattr(self, 'groupBox'):
                self.groupBox.setMinimumHeight(120)
                main_layout.addWidget(self.groupBox)
            
            # 2. Center layout: Real-time dynamic plots
            plots_layout = QHBoxLayout()
            plots_layout.setSpacing(14)
            plots_layout.addWidget(self.plotForces)
            plots_layout.addWidget(self.plotDistance)
            main_layout.addLayout(plots_layout, stretch=1)
            
            # 3. Bottom controls bar
            controls_layout = QHBoxLayout()
            controls_layout.setSpacing(10)
            
            controls_layout.addWidget(self.portComboBox)
            controls_layout.addWidget(self.connectBtn)
            controls_layout.addSpacing(15)
            
            controls_layout.addWidget(self.startBtn)
            controls_layout.addWidget(self.stopBtn)
            controls_layout.addSpacing(25)
            
            controls_layout.addWidget(self.exportBtn)
            controls_layout.addWidget(self.clearBtn)

            # Dedicated button for clearing pinned plot markers
            self.clearMarkersBtn = QPushButton("clear markers")
            self.clearMarkersBtn.setObjectName("clearMarkersBtn")
            self.clearMarkersBtn.clicked.connect(self.clear_markers)
            controls_layout.addWidget(self.clearMarkersBtn)

            controls_layout.addStretch()
            
            main_layout.addLayout(controls_layout)

        self.setWindowTitle("Suspension Test Stand - DAQ System")

        # Apply dark theme styling
        self.setStyleSheet(DARK_STYLE)

        # Status Bar configuration
        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: #ef4444; font-weight: bold; padding-left: 8px;")
        self.statusBar().addWidget(self.status_label)

        # Ports and connection state
        self.serial_connection = None
        self.is_connected = False
        self.is_simulation = True

        # Circular buffers for real-time plotting
        self.buffer_size = 100
        self.time_buffer = deque(maxlen=self.buffer_size)
        self.load1_buffer = deque(maxlen=self.buffer_size)
        self.load2_buffer = deque(maxlen=self.buffer_size)
        self.dist1_buffer = deque(maxlen=self.buffer_size)
        self.dist2_buffer = deque(maxlen=self.buffer_size)
        self.dist3_buffer = deque(maxlen=self.buffer_size)

        # Measurement log for CSV export
        self.recorded_data = []
        self.is_recording = False
        self.sample_index = 0

        # Pinned data markers and current hover match references
        self.pinned_markers_forces = []
        self.pinned_markers_dist = []
        self.current_hover_forces = None
        self.current_hover_dist = None

        # Initialize plot graphs and interactive cursors
        self._init_plots()

        # Set digit count for LCD displays
        for lcd_name in ['Load1', 'Load2', 'Dist1', 'Dist2', 'Dist3']:
            if hasattr(self, lcd_name):
                getattr(self, lcd_name).setDigitCount(8)

        # Symmetrical and centered layout for LCD displays over respective graphs
        if hasattr(self, 'groupBox'):
            self.groupBox.setMinimumHeight(130)

            # Helper function to encapsulate LCD display with centered title label
            def create_lcd_card(lcd_name, title, color):
                container = QWidget()
                vbox = QVBoxLayout(container)
                vbox.setContentsMargins(0, 0, 0, 0)
                vbox.setSpacing(4)

                lbl = QLabel(title)
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold; background: transparent;")
                vbox.addWidget(lbl)

                if hasattr(self, lcd_name):
                    lcd = getattr(self, lcd_name)
                    lcd.setParent(container)
                    lcd.setMinimumSize(95, 32)
                    vbox.addWidget(lcd)

                return container

            # Main horizontal layout inside the top group box
            box_layout = QHBoxLayout(self.groupBox)
            box_layout.setContentsMargins(16, 20, 16, 12)

            # 1. Left container: Load Cells (centered over the force plot)
            forces_widget = QWidget()
            forces_layout = QHBoxLayout(forces_widget)
            forces_layout.setContentsMargins(0, 0, 0, 0)
            forces_layout.setSpacing(25)
            forces_layout.addStretch()
            forces_layout.addWidget(create_lcd_card('Load1', 'Load Cell 1 (N)', '#f87171'))
            forces_layout.addWidget(create_lcd_card('Load2', 'Load Cell 2 (N)', '#fbbf24'))
            forces_layout.addStretch()

            # 2. Right container: Distance Sensors (centered over the displacement plot)
            dist_widget = QWidget()
            dist_layout = QHBoxLayout(dist_widget)
            dist_layout.setContentsMargins(0, 0, 0, 0)
            dist_layout.setSpacing(25)
            dist_layout.addStretch()
            dist_layout.addWidget(create_lcd_card('Dist1', 'Laser 1 (mm)', '#38bdf8'))
            dist_layout.addWidget(create_lcd_card('Dist2', 'Laser 2 (mm)', '#34d399'))
            dist_layout.addWidget(create_lcd_card('Dist3', 'Laser 3 (mm)', '#c084fc'))
            dist_layout.addStretch()

            # Mount both groups symmetrically (1:1 ratio, matching plot layouts)
            box_layout.addWidget(forces_widget, stretch=1)
            box_layout.addSpacing(14)
            box_layout.addWidget(dist_widget, stretch=1)

        # Populate available COM ports
        self.populate_ports()

        # Connect button signals to slots
        self.connectBtn.clicked.connect(self.toggle_connection)
        self.startBtn.clicked.connect(self.start_measurement)
        self.stopBtn.clicked.connect(self.stop_measurement)
        self.exportBtn.clicked.connect(self.export_csv)
        self.clearBtn.clicked.connect(self.clear_data)

        # Set initial button availability
        self.startBtn.setEnabled(False)
        self.stopBtn.setEnabled(False)

        # Periodic timer for real-time sampling and simulation updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_data_stream)

    def _init_plots(self):
        pg.setConfigOption('background', '#181822')
        pg.setConfigOption('foreground', '#cbd5e1')

        # --- Load Cells Plot Setup ---
        self.plot_forces = pg.PlotWidget(title="<b>Load Cells (N)</b>")
        self.plot_forces.plotItem.vb.setMenuEnabled(False)
        self.plot_forces.showGrid(x=True, y=True, alpha=0.25)
        self.plot_forces.addLegend(offset=(10, 10))
        self.curve_load1 = self.plot_forces.plot(pen=pg.mkPen(color='#f87171', width=2.5), name="Load Cell 1")
        self.curve_load2 = self.plot_forces.plot(pen=pg.mkPen(color='#fbbf24', width=2.5), name="Load Cell 2")

        layout_forces = QVBoxLayout(self.plotForces)
        layout_forces.setContentsMargins(4, 4, 4, 4)
        layout_forces.addWidget(self.plot_forces)

        # --- Distance Sensors Plot Setup ---
        self.plot_dist = pg.PlotWidget(title="<b>Distance Sensors (mm)</b>")
        self.plot_dist.plotItem.vb.setMenuEnabled(False)
        self.plot_dist.showGrid(x=True, y=True, alpha=0.25)
        self.plot_dist.addLegend(offset=(10, 10))
        self.curve_dist1 = self.plot_dist.plot(pen=pg.mkPen(color='#38bdf8', width=2.5), name="Laser 1")
        self.curve_dist2 = self.plot_dist.plot(pen=pg.mkPen(color='#34d399', width=2.5), name="Laser 2")
        self.curve_dist3 = self.plot_dist.plot(pen=pg.mkPen(color='#c084fc', width=2.5), name="Laser 3")

        layout_dist = QVBoxLayout(self.plotDistance)
        layout_dist.setContentsMargins(4, 4, 4, 4)
        layout_dist.addWidget(self.plot_dist)

        # --- Crosshair Cursor and Tooltip: Load Cells Plot ---
        self.vLine_forces = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('#64748b', style=pg.QtCore.Qt.DashLine))
        self.hLine_forces = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('#64748b', style=pg.QtCore.Qt.DashLine))
        self.plot_forces.addItem(self.vLine_forces, ignoreBounds=True)
        self.plot_forces.addItem(self.hLine_forces, ignoreBounds=True)
        self.cursor_text_forces = pg.TextItem(anchor=(0, 1), fill=(24, 24, 34, 220))
        self.plot_forces.addItem(self.cursor_text_forces, ignoreBounds=True)
        self.plot_forces.scene().sigMouseMoved.connect(self.on_mouse_moved_forces)

        # --- Crosshair Cursor and Tooltip: Distance Sensors Plot ---
        self.vLine_dist = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('#64748b', style=pg.QtCore.Qt.DashLine))
        self.hLine_dist = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('#64748b', style=pg.QtCore.Qt.DashLine))
        self.plot_dist.addItem(self.vLine_dist, ignoreBounds=True)
        self.plot_dist.addItem(self.hLine_dist, ignoreBounds=True)
        self.cursor_text_dist = pg.TextItem(anchor=(0, 1), fill=(24, 24, 34, 220))
        self.plot_dist.addItem(self.cursor_text_dist, ignoreBounds=True)
        self.plot_dist.scene().sigMouseMoved.connect(self.on_mouse_moved_dist)

        # Bind mouse click events for pinning data tips
        self.plot_forces.scene().sigMouseClicked.connect(self.on_plot_clicked_forces)
        self.plot_dist.scene().sigMouseClicked.connect(self.on_plot_clicked_dist)

    def on_mouse_moved_forces(self, evt):
        pos = evt
        if not self.plot_forces.sceneBoundingRect().contains(pos) or len(self.time_buffer) < 2:
            self.vLine_forces.hide()
            self.hLine_forces.hide()
            self.cursor_text_forces.hide()
            self.current_hover_forces = None
            return

        vb = self.plot_forces.plotItem.vb
        times = list(self.time_buffer)
        l1 = list(self.load1_buffer)
        l2 = list(self.load2_buffer)

        min_dist_px = float('inf')
        best_match = None

        for i in range(len(times)):
            t_val = times[i]
            for name, val, col in [("Load Cell 1", l1[i], "#f87171"), ("Load Cell 2", l2[i], "#fbbf24")]:
                pt_scene = vb.mapViewToScene(pg.Point(t_val, val))
                dist_px = math.hypot(pt_scene.x() - pos.x(), pt_scene.y() - pos.y())
                if dist_px < min_dist_px:
                    min_dist_px = dist_px
                    best_match = (name, t_val, val, col)

        if best_match and min_dist_px < 30.0:
            self.current_hover_forces = best_match
            name, snap_x, snap_y, color = best_match
            self.vLine_forces.setPos(snap_x)
            self.hLine_forces.setPos(snap_y)
            self.cursor_text_forces.setPos(snap_x, snap_y)
            self.cursor_text_forces.setHtml(
                f"<div style='font-size: 11px; padding: 3px; font-family: Segoe UI;'>"
                f"<span style='color:{color}; font-weight:bold;'>{name}</span><br>"
                f"t: {snap_x:.2f} s<br>"
                f"F: {snap_y:.2f} N"
                f"</div>"
            )
            self.vLine_forces.show()
            self.hLine_forces.show()
            self.cursor_text_forces.show()
        else:
            self.current_hover_forces = None
            self.vLine_forces.hide()
            self.hLine_forces.hide()
            self.cursor_text_forces.hide()

    def on_plot_clicked_forces(self, evt):
        # 1. LEFT CLICK: Pin marker onto plot curve
        if evt.button() == Qt.LeftButton and self.current_hover_forces:
            name, snap_x, snap_y, color = self.current_hover_forces

            pt_item = pg.ScatterPlotItem(x=[snap_x], y=[snap_y], size=8, brush=pg.mkBrush(color), pen=pg.mkPen('w', width=1))
            self.plot_forces.addItem(pt_item)

            txt_item = pg.TextItem(anchor=(0, 1), fill=(24, 24, 34, 220))
            txt_item.setPos(snap_x, snap_y)
            txt_item.setHtml(
                f"<div style='font-size: 10px; padding: 3px; font-family: Segoe UI; border: 1px solid {color}; border-radius: 4px;'>"
                f"<span style='color:{color}; font-weight:bold;'>{name}</span><br>"
                f"t: {snap_x:.2f}s | <b>{snap_y:.2f} N</b>"
                f"</div>"
            )
            self.plot_forces.addItem(txt_item)
            
            self.pinned_markers_forces.append({
                'x': snap_x,
                'y': snap_y,
                'pt': pt_item,
                'txt': txt_item
            })
            evt.accept()

        # 2. RIGHT CLICK: Delete specific clicked marker or text box
        elif evt.button() == Qt.RightButton and self.pinned_markers_forces:
            pos = evt.scenePos()
            vb = self.plot_forces.plotItem.vb

            for marker in list(self.pinned_markers_forces):
                clicked_on_text = marker['txt'].sceneBoundingRect().contains(pos)
                pt_scene = vb.mapViewToScene(pg.Point(marker['x'], marker['y']))
                dist_px = math.hypot(pt_scene.x() - pos.x(), pt_scene.y() - pos.y())

                if clicked_on_text or dist_px < 25.0:
                    self.plot_forces.removeItem(marker['pt'])
                    self.plot_forces.removeItem(marker['txt'])
                    self.pinned_markers_forces.remove(marker)
                    evt.accept()
                    break

    def on_mouse_moved_dist(self, evt):
        pos = evt
        if not self.plot_dist.sceneBoundingRect().contains(pos) or len(self.time_buffer) < 2:
            self.vLine_dist.hide()
            self.hLine_dist.hide()
            self.cursor_text_dist.hide()
            self.current_hover_dist = None
            return

        vb = self.plot_dist.plotItem.vb
        times = list(self.time_buffer)
        d1 = list(self.dist1_buffer)
        d2 = list(self.dist2_buffer)
        d3 = list(self.dist3_buffer)

        min_dist_px = float('inf')
        best_match = None

        for i in range(len(times)):
            t_val = times[i]
            for name, val, col in [("Laser 1", d1[i], "#38bdf8"), ("Laser 2", d2[i], "#34d399"), ("Laser 3", d3[i], "#c084fc")]:
                pt_scene = vb.mapViewToScene(pg.Point(t_val, val))
                dist_px = math.hypot(pt_scene.x() - pos.x(), pt_scene.y() - pos.y())
                if dist_px < min_dist_px:
                    min_dist_px = dist_px
                    best_match = (name, t_val, val, col)

        if best_match and min_dist_px < 30.0:
            self.current_hover_dist = best_match
            name, snap_x, snap_y, color = best_match
            self.vLine_dist.setPos(snap_x)
            self.hLine_dist.setPos(snap_y)
            self.cursor_text_dist.setPos(snap_x, snap_y)
            self.cursor_text_dist.setHtml(
                f"<div style='font-size: 11px; padding: 3px; font-family: Segoe UI;'>"
                f"<span style='color:{color}; font-weight:bold;'>{name}</span><br>"
                f"t: {snap_x:.2f} s<br>"
                f"d: {snap_y:.2f} mm"
                f"</div>"
            )
            self.vLine_dist.show()
            self.hLine_dist.show()
            self.cursor_text_dist.show()
        else:
            self.current_hover_dist = None
            self.vLine_dist.hide()
            self.hLine_dist.hide()
            self.cursor_text_dist.hide()

    def on_plot_clicked_dist(self, evt):
        # 1. LEFT CLICK: Pin marker onto plot curve
        if evt.button() == Qt.LeftButton and self.current_hover_dist:
            name, snap_x, snap_y, color = self.current_hover_dist

            pt_item = pg.ScatterPlotItem(x=[snap_x], y=[snap_y], size=8, brush=pg.mkBrush(color), pen=pg.mkPen('w', width=1))
            self.plot_dist.addItem(pt_item)

            txt_item = pg.TextItem(anchor=(0, 1), fill=(24, 24, 34, 220))
            txt_item.setPos(snap_x, snap_y)
            txt_item.setHtml(
                f"<div style='font-size: 10px; padding: 3px; font-family: Segoe UI; border: 1px solid {color}; border-radius: 4px;'>"
                f"<span style='color:{color}; font-weight:bold;'>{name}</span><br>"
                f"t: {snap_x:.2f}s | <b>{snap_y:.2f} mm</b>"
                f"</div>"
            )
            self.plot_dist.addItem(txt_item)
            
            self.pinned_markers_dist.append({
                'x': snap_x,
                'y': snap_y,
                'pt': pt_item,
                'txt': txt_item
            })
            evt.accept()

        # 2. RIGHT CLICK: Delete specific clicked marker or text box
        elif evt.button() == Qt.RightButton and self.pinned_markers_dist:
            pos = evt.scenePos()
            vb = self.plot_dist.plotItem.vb

            for marker in list(self.pinned_markers_dist):
                clicked_on_text = marker['txt'].sceneBoundingRect().contains(pos)
                pt_scene = vb.mapViewToScene(pg.Point(marker['x'], marker['y']))
                dist_px = math.hypot(pt_scene.x() - pos.x(), pt_scene.y() - pos.y())

                if clicked_on_text or dist_px < 25.0:
                    self.plot_dist.removeItem(marker['pt'])
                    self.plot_dist.removeItem(marker['txt'])
                    self.pinned_markers_dist.remove(marker)
                    evt.accept()
                    break

    def populate_ports(self):
        self.portComboBox.clear()
        self.portComboBox.addItem("Simulation Mode (Virtual)", "SIM")

        available_ports = serial.tools.list_ports.comports()
        for p in available_ports:
            display_text = f"{p.device} - {p.description}"
            self.portComboBox.addItem(display_text, p.device)

    def toggle_connection(self):
        if not self.is_connected:
            selected_port = self.portComboBox.currentData()

            if selected_port == "SIM":
                self.is_simulation = True
                self._set_connected_state(True)
                self.status_label.setText("● Connected: Virtual Simulation Mode")
                self.status_label.setStyleSheet("color: #38bdf8; font-weight: bold; padding-left: 8px;")
                self.timer.start(33)
            else:
                try:
                    self.serial_connection = serial.Serial(
                        port=selected_port,
                        baudrate=115200,
                        timeout=0.1
                    )
                    self.is_simulation = False
                    self._set_connected_state(True)
                    self.status_label.setText(f"● Connected: {selected_port} @ 115200 baud")
                    self.status_label.setStyleSheet("color: #10b981; font-weight: bold; padding-left: 8px;")
                    self.timer.start(10)
                except Exception as ex:
                    QMessageBox.critical(self, "Connection Error", f"Failed to open port {selected_port}:\n{str(ex)}")
                    return
        else:
            self.timer.stop()
            if self.is_recording:
                self.stop_measurement()

            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
                self.serial_connection = None

            self._set_connected_state(False)
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: #ef4444; font-weight: bold; padding-left: 8px;")

    def _set_connected_state(self, connected: bool):
        self.is_connected = connected
        if connected:
            self.connectBtn.setText("Disconnect")
            self.connectBtn.setStyleSheet("background-color: #ef4444; color: white;")
            self.portComboBox.setEnabled(False)
            self.startBtn.setEnabled(True)
        else:
            self.connectBtn.setText("Connect")
            self.connectBtn.setStyleSheet("background-color: #3b82f6; color: white;")
            self.portComboBox.setEnabled(True)
            self.startBtn.setEnabled(False)
            self.stopBtn.setEnabled(False)
            self.populate_ports()

    def process_data_stream(self):
        if self.is_simulation:
            self.sample_index += 1
            t = self.sample_index * 0.033
            load1 = 500 + 150 * math.sin(2 * math.pi * 0.5 * t) + random.uniform(-10, 10)
            load2 = 450 + 120 * math.cos(2 * math.pi * 0.5 * t) + random.uniform(-10, 10)
            dist1 = 30 + 10 * math.sin(2 * math.pi * 1.0 * t) + random.uniform(-0.5, 0.5)
            dist2 = 28 + 8 * math.sin(2 * math.pi * 1.0 * t + 0.5) + random.uniform(-0.5, 0.5)
            dist3 = 32 + 12 * math.cos(2 * math.pi * 1.0 * t) + random.uniform(-0.5, 0.5)
            self.update_ui(t, load1, load2, dist1, dist2, dist3)
        else:
            if self.serial_connection and self.serial_connection.in_waiting > 0:
                try:
                    raw_line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line.startswith('{') and raw_line.endswith('}'):
                        data = json.loads(raw_line)
                        
                        dist1 = float(data.get("sensor_1", 0.0))
                        dist2 = float(data.get("sensor_2", 0.0))
                        dist3 = float(data.get("sensor_3", 0.0))
                        
                        load1 = float(data.get("load_1", 0.0))
                        load2 = float(data.get("load_2", 0.0))
                        
                        self.sample_index += 1
                        t = self.sample_index * 0.033
                        self.update_ui(t, load1, load2, dist1, dist2, dist3)
                except (ValueError, json.JSONDecodeError):
                    pass

    def update_ui(self, t, load1, load2, dist1, dist2, dist3):
        self.time_buffer.append(t)
        self.load1_buffer.append(load1)
        self.load2_buffer.append(load2)
        self.dist1_buffer.append(dist1)
        self.dist2_buffer.append(dist2)
        self.dist3_buffer.append(dist3)

        times = list(self.time_buffer)
        self.curve_load1.setData(times, list(self.load1_buffer))
        self.curve_load2.setData(times, list(self.load2_buffer))
        self.curve_dist1.setData(times, list(self.dist1_buffer))
        self.curve_dist2.setData(times, list(self.dist2_buffer))
        self.curve_dist3.setData(times, list(self.dist3_buffer))

        if hasattr(self, 'Load1'): self.Load1.display(f"{load1:.2f}")
        if hasattr(self, 'Load2'): self.Load2.display(f"{load2:.2f}")
        if hasattr(self, 'Dist1'): self.Dist1.display(f"{dist1:.2f}")
        if hasattr(self, 'Dist2'): self.Dist2.display(f"{dist2:.2f}")
        if hasattr(self, 'Dist3'): self.Dist3.display(f"{dist3:.2f}")

        if self.is_recording:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.recorded_data.append([
                timestamp,
                f"{t:.3f}",
                f"{load1:.2f}",
                f"{load2:.2f}",
                f"{dist1:.2f}",
                f"{dist2:.2f}",
                f"{dist3:.2f}"
            ])

    def clear_data(self):
        if self.is_recording:
            return

        self.recorded_data.clear()
        self.sample_index = 0

        self.time_buffer.clear()
        self.load1_buffer.clear()
        self.load2_buffer.clear()
        self.dist1_buffer.clear()
        self.dist2_buffer.clear()
        self.dist3_buffer.clear()

        self.curve_load1.clear()
        self.curve_load2.clear()
        self.curve_dist1.clear()
        self.curve_dist2.clear()
        self.curve_dist3.clear()

        if hasattr(self, 'Load1'): self.Load1.display("0.0")
        if hasattr(self, 'Load2'): self.Load2.display("0.0")
        if hasattr(self, 'Dist1'): self.Dist1.display("0.00")
        if hasattr(self, 'Dist2'): self.Dist2.display("0.00")
        if hasattr(self, 'Dist3'): self.Dist3.display("0.00")

        if self.is_connected:
            self.status_label.setText("● Ready (Data cleared - 0 samples)")
            self.status_label.setStyleSheet("color: #38bdf8; font-weight: bold; padding-left: 8px;")
        else:
            self.status_label.setText("● Disconnected")
            self.status_label.setStyleSheet("color: #ef4444; font-weight: bold; padding-left: 8px;")

    def clear_markers(self):
        """Remove all pinned markers and data tips from both plots."""
        for marker in self.pinned_markers_forces:
            self.plot_forces.removeItem(marker['pt'])
            self.plot_forces.removeItem(marker['txt'])
        self.pinned_markers_forces.clear()

        for marker in self.pinned_markers_dist:
            self.plot_dist.removeItem(marker['pt'])
            self.plot_dist.removeItem(marker['txt'])
        self.pinned_markers_dist.clear()

    def start_measurement(self):
        self.recorded_data.clear()
        self.is_recording = True
        self.startBtn.setEnabled(False)
        self.stopBtn.setEnabled(True)
        if self.is_simulation:
            self.status_label.setText("● RECORDING (Simulation Mode)...")
        else:
            self.status_label.setText("● RECORDING DATA...")
        self.status_label.setStyleSheet("color: #f59e0b; font-weight: bold; padding-left: 8px;")

    def stop_measurement(self):
        self.is_recording = False
        self.startBtn.setEnabled(True)
        self.stopBtn.setEnabled(False)
        self.status_label.setText(f"● Idle (Recorded {len(self.recorded_data)} samples)")
        self.status_label.setStyleSheet("color: #10b981; font-weight: bold; padding-left: 8px;")

    def export_csv(self):
        if not self.recorded_data:
            QMessageBox.information(self, "Export Info", "No recorded data to export.")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Measurement Data to CSV",
            "test_stand_export.csv",
            "CSV Files (*.csv)"
        )
        if file_path:
            with open(file_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Timestamp",
                    "RelativeTime_s",
                    "LoadCell1_N",
                    "LoadCell2_N",
                    "Distance1_mm",
                    "Distance2_mm",
                    "Distance3_mm"
                ])
                writer.writerows(self.recorded_data)
            self.status_label.setText(f"● Exported successfully to {file_path.split('/')[-1]}")
            self.status_label.setStyleSheet("color: #38bdf8; font-weight: bold; padding-left: 8px;")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())
