import sys
import os
import cv2
import numpy as np
import time
import csv
import threading
import serial
import pynmea2
import pyzed.sl as sl

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QFileDialog, QMessageBox,
    QVBoxLayout, QHBoxLayout, QLineEdit, QCheckBox, QFormLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

# check the gps serial port
gps_port = "COM1"
gps_baudrate = 9600


# ---------------- GPS Thread ----------------
class GPSThread(threading.Thread):
    def __init__(self, port=gps_port, baudrate=gps_baudrate):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.lat = None
        self.lon = None
        self.alt = None
        self.fix_quality = 0
        self.lock = threading.Lock()
        self.running = True
        self.connected = False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✅ GPS connected on {self.port} at {self.baudrate} baud")
            self.connected = True
        except Exception as e:
            print(f"❌ GPS not connected or error: {e}")
            return

        while self.running:
            try:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    if line.startswith("$GPGGA") or line.startswith("$GPRMC") or line.startswith("$GPGLL"):
                        try:
                            msg = pynmea2.parse(line)
                            with self.lock:
                                if hasattr(msg, "latitude") and hasattr(msg, "longitude"):
                                    self.lat = msg.latitude
                                    self.lon = msg.longitude
                                self.alt = getattr(msg, "altitude", self.alt if self.alt else 0.0)
                                self.fix_quality = getattr(msg, "gps_qual", self.fix_quality)
                        except pynmea2.ParseError:
                            continue
            except Exception as e:
                print("GPS reading error:", e)
                continue

    def get_position(self):
        with self.lock:
            return self.lat, self.lon, self.alt, self.fix_quality

    def stop(self):
        self.running = False
 

# ---------------- ZED Capture Tool ----------------
class ZEDCapturePro(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZED 2i Capture Tool (SDK 5.1)")
        self.setGeometry(100, 80, 1600, 900)
        self.setStyleSheet("""
            QWidget { background-color: #1e1e1e; color: white; }
            QLabel { color: white; font-size: 14px; }
            QPushButton {
                background-color: #0078D7; color: white; border-radius: 6px;
                padding: 6px 12px; font-weight: bold;
            }
            QPushButton:hover { background-color: #1493FF; }
            QPushButton:disabled { background-color: #555; }
        """)

        # ------------------- Initialize ZED -------------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080 #
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            QMessageBox.critical(self, "Error", "Failed to open ZED camera.")
            sys.exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        self.depth_mat = sl.Mat()

        # Capture state
        self.output_dir = None
        self.recording = False
        self.frame_id = 0
        self.csv_writer = None
        self.csv_file_handle = None
        self.video_writer = None

        # ------------------- GPS -------------------
        self.gps = GPSThread(port=gps_port, baudrate=gps_baudrate)
        self.gps.start()

        # ------------------- UI -------------------
        self.left_label = self._make_view_label("Left Camera")
        self.right_label = self._make_view_label("Right Camera")
        self.depth_label = self._make_view_label("Depth View")
        self.gps_label = QLabel("GPS: Waiting for fix...")
        self.gps_label.setAlignment(Qt.AlignCenter)
        self.gps_label.setStyleSheet("color: #00FF00; font-size: 16px; font-weight: bold;")

        # Control Panel
        self.width_edit = QLineEdit("1920")
        self.height_edit = QLineEdit("1080")
        self.fps_edit = QLineEdit("30")
        self.save_image_check = QCheckBox("Save Images")
        self.save_image_check.setChecked(True)
        self.save_video_check = QCheckBox("Save Video")
        self.save_video_check.setChecked(True)

        # Depth min/max controls
        self.depth_min_edit = QLineEdit("500")
        self.depth_max_edit = QLineEdit("10000")
        self.depth_min_edit.setToolTip("Minimum depth in mm for normalization")
        self.depth_max_edit.setToolTip("Maximum depth in mm for normalization")

        self.btn_select = QPushButton("📁 Select Output Directory")
        self.btn_select.clicked.connect(self.select_output_dir)
        self.btn_start = QPushButton("▶ Start Capture")
        self.btn_start.clicked.connect(self.start_capture)
        self.btn_stop = QPushButton("⏹ Stop Capture")
        self.btn_stop.clicked.connect(self.stop_capture)
        self.btn_stop.setEnabled(False)
        self.btn_close = QPushButton("❌ Close Camera")
        self.btn_close.clicked.connect(self.close_camera)

        # Layouts
        control_form = QFormLayout()
        control_form.addRow("Width:", self.width_edit)
        control_form.addRow("Height:", self.height_edit)
        control_form.addRow("FPS:", self.fps_edit)
        control_form.addRow(self.save_image_check)
        control_form.addRow(self.save_video_check)
        control_form.addRow("Depth Min (mm):", self.depth_min_edit)
        control_form.addRow("Depth Max (mm):", self.depth_max_edit)
        control_form.addRow(self.btn_select)
        control_form.addRow(self.btn_start)
        control_form.addRow(self.btn_stop)
        control_form.addRow(self.btn_close)

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.left_label)
        top_layout.addWidget(self.right_label)
        top_layout.addWidget(self.depth_label)

        right_layout = QVBoxLayout()
        right_layout.addLayout(control_form)
        right_layout.addStretch(1)

        main_layout = QVBoxLayout()
        main_top = QHBoxLayout()
        main_top.addLayout(top_layout, stretch=3)
        main_top.addLayout(right_layout, stretch=1)
        main_layout.addLayout(main_top)
        main_layout.addWidget(self.gps_label)
        self.setLayout(main_layout)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frames)

        # Read FPS from the UI to set the timer's speed
        try:
            fps = float(self.fps_edit.text())
        except ValueError:
            fps = 30.0  # Fallback in case of bad text in the box

        # Calculate interval (e.g., 1000 ms / 30fps = 33 milliseconds)
        timer_interval_ms = int(1000.0 / fps)

        # Start the timer with the correct interval
        self.timer.start(timer_interval_ms)

    def _make_view_label(self, title):
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setFixedSize(480, 270)
        label.setStyleSheet("background-color: #000; border: 2px solid #333;")
        return label