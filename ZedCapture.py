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
 
 

