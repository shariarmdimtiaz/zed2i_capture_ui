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

    def update_frames(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
        self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
        self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)

        left_img = self.image_left.get_data()
        right_img = self.image_right.get_data()
        depth_raw = self.depth_mat.get_data().astype(np.float32)

        # Replace NaNs/Infs with 0
        depth_clean = np.nan_to_num(depth_raw, nan=0.0, posinf=0.0, neginf=0.0)

        # Get depth min/max from user input
        try:
            depth_min = float(self.depth_min_edit.text())
            depth_max = float(self.depth_max_edit.text())
        except ValueError:
            depth_min = 500.0
            depth_max = 10000.0

        # Clip and normalize depth for display and saving
        depth_clipped = np.clip(depth_clean, depth_min, depth_max)
        depth_norm = ((depth_clipped - depth_min) / (depth_max - depth_min) * 255.0).astype(np.uint8)

        # ******** THIS IS THE FIX *********
        # Invert the normalized map: 0 (close) becomes 255 (red), 255 (far) becomes 0 (blue)
        depth_inv_norm = 255 - depth_norm
        # Apply colormap to the INVERTED map
        depth_color = cv2.applyColorMap(depth_inv_norm, cv2.COLORMAP_JET)
        depth_color_save = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        # **********************************

        left_rgb = cv2.cvtColor(left_img, cv2.COLOR_RGBA2RGB)
        right_rgb = cv2.cvtColor(right_img, cv2.COLOR_RGBA2RGB)

        # GPS info
        lat, lon, alt, fix = self.gps.get_position()
        lat = lat if lat is not None else 0.0
        lon = lon if lon is not None else 0.0
        alt = alt if alt is not None else 0.0
        status = "GPS connected. [✔]" if fix == 2 else "GPS not connected. ⚠"
        self.gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}, Alt: {alt:.1f}m, Status: {status}")

        # Display images
        self._display_qimage(left_rgb, self.left_label)
        self._display_qimage(right_rgb, self.right_label)
        self._display_qimage(depth_color, self.depth_label)

        # Save frames
        if self.recording and self.output_dir:
            self.save_frames(left_rgb, right_rgb, depth_color_save, depth_clean, lat, lon, alt)

    def _display_qimage(self, frame, widget):
        height, width, _ = frame.shape
        qimg = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(widget.width(), widget.height(), Qt.KeepAspectRatio)
        widget.setPixmap(pix)
        
    def select_output_dir(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Output Directory")
        if folder:
            self.output_dir = folder
            os.makedirs(os.path.join(folder, "Left"), exist_ok=True)
            os.makedirs(os.path.join(folder, "Right"), exist_ok=True)
            os.makedirs(os.path.join(folder, "Depth"), exist_ok=True)
            os.makedirs(os.path.join(folder, "Video"), exist_ok=True)
            self.csv_file = os.path.join(folder, "gps_log.csv")
            self.csv_file_handle = open(self.csv_file, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file_handle)
            self.csv_writer.writerow(["frame_id", "timestamp", "latitude", "longitude", "altitude"])
            print(f"[INFO] Output folder: {self.output_dir}")

    def start_capture(self):
        if not self.output_dir:
            QMessageBox.warning(self, "Warning", "Select an output directory first!")
            return
        self.recording = True
        self.frame_id = 0
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        if self.save_video_check.isChecked():
            video_path = os.path.join(self.output_dir, "Video", "output.avi")
            width = int(self.width_edit.text())
            height = int(self.height_edit.text())
            fps = float(self.fps_edit.text())
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            print(f"[INFO] Video recording started: {video_path}")

        print("[INFO] Capture started...")

    def stop_capture(self):
        self.recording = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        if self.csv_file_handle:
            self.csv_file_handle.close()
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            print("[INFO] Video recording stopped.")
        print("[INFO] Capture stopped.")

    def save_frames(self, left_img, right_img, depth_color, depth_raw, lat, lon, alt):
        ts = str(self.frame_id).zfill(5)
        if self.save_image_check.isChecked():
            cv2.imwrite(os.path.join(self.output_dir, "Left", f"left_{ts}.png"), left_img)
            cv2.imwrite(os.path.join(self.output_dir, "Right", f"right_{ts}.png"), right_img)
            # This 'depth_color' is now the correctly inverted one
            cv2.imwrite(os.path.join(self.output_dir, "Depth", f"depth_{ts}.png"), depth_color)
            np.save(os.path.join(self.output_dir, "Depth", f"depth_{ts}.npy"), depth_raw)

        if self.video_writer:
            resized = cv2.resize(left_img, (int(self.width_edit.text()), int(self.height_edit.text())))
            self.video_writer.write(resized)

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.csv_writer:
            self.csv_writer.writerow([ts, timestamp, lat, lon, alt])
            self.csv_file_handle.flush()
        self.frame_id += 1

    def close_camera(self):
        self.timer.stop()
        self.zed.close()
        self.gps.stop()
        if self.csv_file_handle:
            self.csv_file_handle.close()
        if self.video_writer:
            self.video_writer.release()
        self.close()


# ---------------- Main ----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ZEDCapturePro()
    win.show()
    sys.exit(app.exec_())