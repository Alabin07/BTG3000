#!/usr/bin/env python3
"""
Rocket Telemetry Ground Station
================================
Receives UDP JSON packets from ESP32 and renders a live dashboard.

Requirements:
    pip install pyqtgraph PyQt6 numpy PyOpenGL

Run:
    python ground_station.py
"""

import sys
import json
import socket
import threading
import math
import time
from collections import deque

import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout,
    QVBoxLayout, QHBoxLayout, QLabel, QFrame, QSizePolicy,
    QSplitter
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QColor, QPalette, QFontDatabase
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtCore import Qt
import pyqtgraph as pg
from OpenGL.GL import *
from OpenGL.GLU import *

# ── Config ──────────────────────────────────────────────────────────────────
UDP_PORT      = 5005
HISTORY_LEN   = 500   # samples kept in rolling window (~5 s at 100 Hz)
UPDATE_HZ     = 30    # dashboard refresh rate

# ── Colour palette ───────────────────────────────────────────────────────────
BG          = "#0a0c10"
PANEL       = "#111620"
BORDER      = "#1e2840"
ACCENT      = "#00e5ff"
ACCENT2     = "#ff4b6e"
ACCENT3     = "#39ff14"
TEXT        = "#c8d8f0"
TEXT_DIM    = "#4a5a70"
WARN        = "#ffaa00"

COLORS = {
    "alt":   ACCENT,
    "va":    ACCENT2,
    "ax":    "#7b61ff",
    "ay":    "#00e5ff",
    "az":    ACCENT3,
    "gx":    "#ff6b35",
    "gy":    "#f7c59f",
    "gz":    "#efefd0",
    "temp":  WARN,
    "pres":  "#a0c4ff",
    "amag":  "#ffffff",
}

pg.setConfigOptions(antialias=True, background=BG, foreground=TEXT)


# ── UDP receiver (runs in a background thread) ───────────────────────────────
class Receiver(QObject):
    packet_received = pyqtSignal(dict)

    def __init__(self, port: int):
        super().__init__()
        self.port = port
        self._running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self.port))
        sock.settimeout(1.0)
        while self._running:
            try:
                data, _ = sock.recvfrom(512)
                pkt = json.loads(data.decode())
                self.packet_received.emit(pkt)
            except (socket.timeout, json.JSONDecodeError):
                pass
        sock.close()

    def stop(self):
        self._running = False


# ── Rolling data store ────────────────────────────────────────────────────────
class DataStore:
    def __init__(self, maxlen: int):
        self.t    = deque(maxlen=maxlen)
        self.alt  = deque(maxlen=maxlen)
        self.va   = deque(maxlen=maxlen)
        self.ax   = deque(maxlen=maxlen)
        self.ay   = deque(maxlen=maxlen)
        self.az   = deque(maxlen=maxlen)
        self.gx   = deque(maxlen=maxlen)
        self.gy   = deque(maxlen=maxlen)
        self.gz   = deque(maxlen=maxlen)
        self.temp = deque(maxlen=maxlen)
        self.pres = deque(maxlen=maxlen)
        self.latest: dict = {}
        self.pkt_count = 0
        self.last_pkt_time = time.time()
        self._lock = threading.Lock()

        # Packet-rate tracking: store timestamps of recent packets
        self._recent_times: deque = deque(maxlen=200)

    def ingest(self, pkt: dict):
        with self._lock:
            now = time.time()
            t0 = pkt["t"] / 1000.0  # ms → s
            self.t.append(t0)
            self.alt.append(pkt.get("alt", 0))
            self.va.append(pkt.get("va", 0))
            self.ax.append(pkt.get("ax", 0))
            self.ay.append(pkt.get("ay", 0))
            self.az.append(pkt.get("az", 0))
            self.gx.append(pkt.get("gx", 0))
            self.gy.append(pkt.get("gy", 0))
            self.gz.append(pkt.get("gz", 0))
            self.temp.append(pkt.get("temp", 0))
            self.pres.append(pkt.get("pres", 0))
            self.latest = pkt
            self.pkt_count += 1
            self.last_pkt_time = now
            self._recent_times.append(now)

    def packets_per_second(self) -> float:
        """Compute packet rate over the last 1 second."""
        with self._lock:
            now = time.time()
            cutoff = now - 1.0
            count = sum(1 for t in self._recent_times if t >= cutoff)
            return float(count)

    def arrays(self, *keys):
        with self._lock:
            t = np.array(self.t)
            if len(t) == 0:
                return (np.array([]),) * (len(keys) + 1)
            t = t - t[0]   # relative seconds from window start
            arrs = [np.array(getattr(self, k)) for k in keys]
            return (t, *arrs)


# ── 3D Quaternion Visualizer ──────────────────────────────────────────────────
class QuaternionWidget(QOpenGLWidget):
    """OpenGL widget that renders a simple rocket body oriented by a quaternion."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self._last_quat = (1.0, 0.0, 0.0, 0.0)
        self.setMinimumSize(200, 220)

    def set_quaternion(self, qw, qx, qy, qz):
        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.update()

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glClearColor(0.067, 0.086, 0.063, 1.0)  # matches PANEL hex ~#111620

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45.0, aspect, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        # Clear with panel background colour
        r, g, b = 0x11/255, 0x16/255, 0x20/255
        glClearColor(r, g, b, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        gluLookAt(0, 0, 4,   0, 0, 0,   0, 1, 0)

        # Build rotation matrix directly from quaternion (avoids axis-angle instability)
        qw = self.qw; qx = self.qx; qy = self.qy; qz = self.qz
        # Normalise to guard against drift
        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if n > 1e-6:
            qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n

        # Column-major 4×4 for glMultMatrixf
        m = [
            1-2*(qy*qy+qz*qz),   2*(qx*qy+qz*qw),   2*(qx*qz-qy*qw),  0,
              2*(qx*qy-qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz+qx*qw),  0,
              2*(qx*qz+qy*qw),   2*(qy*qz-qx*qw), 1-2*(qx*qx+qy*qy),  0,
            0, 0, 0, 1,
        ]

        glPushMatrix()
        glMultMatrixf(m)

        # ── Draw body axis reference rings ──────────────────────────────────
        self._draw_axis_rings()

        # ── Draw rocket body (elongated cylinder) ───────────────────────────
        self._draw_rocket_body()

        glPopMatrix()

        # ── Draw world-frame axis labels (fixed) ────────────────────────────
        self._draw_world_axes()

    def _draw_axis_rings(self):
        """Draw three thin rings around the rocket's local axes."""
        segs = 48
        r = 0.55

        axes = [
            ((1, 0, 0), 0,   (1.0, 0.24, 0.43)),   # X  red-pink
            ((0, 1, 0), 90,  (0.0, 0.9,  1.0)),     # Y  cyan
            ((0, 0, 1), 90,  (0.22, 1.0, 0.08)),    # Z  green
        ]
        for (rx, ry, rz), extra_rot, (cr, cg, cb) in axes:
            glPushMatrix()
            glRotatef(90 * (rx != 0), 0, 1, 0)
            glRotatef(90 * (rz != 0), 1, 0, 0)
            glColor4f(cr, cg, cb, 0.5)
            glLineWidth(1.2)
            glBegin(GL_LINE_LOOP)
            for i in range(segs):
                a = 2 * math.pi * i / segs
                glVertex3f(r * math.cos(a), r * math.sin(a), 0)
            glEnd()
            glPopMatrix()

    def _draw_rocket_body(self):
        """Draw a simple rocket silhouette: cylinder body + cone nose + fins."""
        slices = 24
        body_r = 0.12
        body_h = 0.8
        nose_h = 0.35

        # Body cylinder
        glColor4f(0.6, 0.75, 0.95, 0.95)
        self._cylinder(body_r, body_h, slices)

        # Nose cone (sits on top of body)
        glPushMatrix()
        glTranslatef(0, body_h / 2, 0)
        glColor4f(0.0, 0.9, 1.0, 0.95)
        self._cone(body_r, nose_h, slices)
        glPopMatrix()

        # Fins (3 × thin quads at bottom)
        glColor4f(1.0, 0.29, 0.43, 0.9)
        fin_spread = 0.28
        fin_h = 0.22
        for i in range(3):
            angle = 120 * i
            glPushMatrix()
            glRotatef(angle, 0, 1, 0)
            glBegin(GL_QUADS)
            y_base = -body_h / 2
            glVertex3f(body_r, y_base, 0)
            glVertex3f(body_r + fin_spread, y_base - fin_h, 0)
            glVertex3f(body_r + fin_spread, y_base - fin_h + 0.04, 0)
            glVertex3f(body_r, y_base + 0.04, 0)
            glEnd()
            glPopMatrix()

        # Up-arrow direction indicator (thin line along +Y body axis)
        glColor4f(1.0, 1.0, 0.2, 0.8)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glVertex3f(0, -body_h/2, 0)
        glVertex3f(0,  body_h/2 + nose_h, 0)
        glEnd()

    def _cylinder(self, radius, height, slices):
        half = height / 2
        # Side wall
        glBegin(GL_QUAD_STRIP)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            x = radius * math.cos(a)
            z = radius * math.sin(a)
            glVertex3f(x, -half, z)
            glVertex3f(x,  half, z)
        glEnd()
        # Top / bottom caps
        for y in (-half, half):
            glBegin(GL_TRIANGLE_FAN)
            glVertex3f(0, y, 0)
            for i in range(slices + 1):
                a = 2 * math.pi * i / slices
                glVertex3f(radius * math.cos(a), y, radius * math.sin(a))
            glEnd()

    def _cone(self, radius, height, slices):
        # Side
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, height, 0)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), 0, radius * math.sin(a))
        glEnd()
        # Base cap
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, 0)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), 0, radius * math.sin(a))
        glEnd()

    def _draw_world_axes(self):
        """Draw fixed XYZ world-frame arrows (not rotated with rocket)."""
        glLineWidth(2.0)
        length = 0.9
        axes = [
            ((1,0,0), (1.0, 0.3, 0.3)),
            ((0,1,0), (0.3, 1.0, 0.3)),
            ((0,0,1), (0.3, 0.5, 1.0)),
        ]
        glTranslatef(-1.4, -1.3, 0)
        glScalef(0.3, 0.3, 0.3)
        for (dx, dy, dz), (cr, cg, cb) in axes:
            glColor3f(cr, cg, cb)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(dx*length, dy*length, dz*length)
            glEnd()


# ── Styled helpers ────────────────────────────────────────────────────────────
def make_panel(title: str = "") -> QFrame:
    f = QFrame()
    f.setStyleSheet(f"""
        QFrame {{
            background: {PANEL};
            border: 1px solid {BORDER};
            border-radius: 6px;
        }}
    """)
    return f


def make_label(text: str, size: int = 11, color: str = TEXT,
               bold: bool = False) -> QLabel:
    lbl = QLabel(text)
    weight = "bold" if bold else "normal"
    lbl.setStyleSheet(f"color: {color}; font-size: {size}px; font-weight: {weight}; background: transparent; border: none;")
    return lbl


def make_value_label(text: str = "—", size: int = 28,
                     color: str = ACCENT) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(f"""
        color: {color};
        font-size: {size}px;
        font-weight: bold;
        font-family: 'Courier New', monospace;
        background: transparent;
        border: none;
    """)
    lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    return lbl


def styled_plot(title: str, y_label: str, height: int = None) -> pg.PlotWidget:
    pw = pg.PlotWidget(title=title)
    pw.setLabel("left", y_label)
    pw.showGrid(x=True, y=True, alpha=0.15)
    pw.getAxis("bottom").setTextPen(TEXT_DIM)
    pw.getAxis("left").setTextPen(TEXT_DIM)
    pw.setMenuEnabled(False)
    pw.getPlotItem().titleLabel.setAttr("color", TEXT_DIM)
    pw.setBackground(PANEL)
    for axis in ("left", "bottom", "right", "top"):
        ax = pw.getAxis(axis)
        if ax:
            ax.setPen(pg.mkPen(BORDER))
    if height:
        pw.setMaximumHeight(height)
    return pw


# ── Stacked accel panel ───────────────────────────────────────────────────────
class AccelPanel(QWidget):
    """
    Four stacked mini-plots: X / Y / Z (individual axes) + magnitude.
    Much easier to read than a single overlapping 3-axis chart.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px;")

        vl = QVBoxLayout(self)
        vl.setContentsMargins(4, 4, 4, 4)
        vl.setSpacing(2)

        title = make_label("LINEAR ACCELERATION  (gravity-free)", 9, TEXT_DIM, bold=True)
        title.setContentsMargins(6, 4, 0, 2)
        vl.addWidget(title)

        ROW_H = 90

        self.pw_x   = styled_plot("", "X  m/s²", ROW_H)
        self.pw_y   = styled_plot("", "Y  m/s²", ROW_H)
        self.pw_z   = styled_plot("", "Z  m/s²", ROW_H)
        self.pw_mag = styled_plot("", "|a| m/s²", ROW_H)

        # Hide bottom axis labels on all but the bottom plot to save space
        for pw in (self.pw_x, self.pw_y, self.pw_z):
            pw.getAxis("bottom").setStyle(showValues=False)
            pw.getAxis("bottom").setHeight(0)

        # Link X axes so they scroll together
        self.pw_y.setXLink(self.pw_x)
        self.pw_z.setXLink(self.pw_x)
        self.pw_mag.setXLink(self.pw_x)

        self.curve_ax  = self.pw_x.plot(pen=pg.mkPen(COLORS["ax"],   width=1.5))
        self.curve_ay  = self.pw_y.plot(pen=pg.mkPen(COLORS["ay"],   width=1.5))
        self.curve_az  = self.pw_z.plot(pen=pg.mkPen(COLORS["az"],   width=1.5))
        self.curve_mag = self.pw_mag.plot(pen=pg.mkPen(COLORS["amag"], width=1.8))

        # Horizontal zero-lines
        for pw in (self.pw_x, self.pw_y, self.pw_z):
            pw.addLine(y=0, pen=pg.mkPen(BORDER, style=Qt.PenStyle.DashLine))

        for pw in (self.pw_x, self.pw_y, self.pw_z, self.pw_mag):
            vl.addWidget(pw)

    def update_data(self, t, ax, ay, az):
        if len(t) <= 1:
            return
        mag = np.sqrt(ax**2 + ay**2 + az**2)
        self.curve_ax.setData(t, ax)
        self.curve_ay.setData(t, ay)
        self.curve_az.setData(t, az)
        self.curve_mag.setData(t, mag)


# ── Main window ───────────────────────────────────────────────────────────────
class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🚀  Rocket Telemetry Ground Station")
        self.resize(1440, 920)
        self.setStyleSheet(f"QMainWindow {{ background: {BG}; }}")

        self.store = DataStore(HISTORY_LEN)
        self._last_quat = (1.0, 0.0, 0.0, 0.0)
        self._setup_ui()
        self._start_receiver()

        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(1000 // UPDATE_HZ)

        # For pkt/s smoothing
        self._hz_samples: deque = deque(maxlen=5)

    # ── UI layout ────────────────────────────────────────────────────────────
    def _setup_ui(self):
        root = QWidget()
        root.setStyleSheet(f"background: {BG};")
        self.setCentralWidget(root)
        layout = QGridLayout(root)
        layout.setSpacing(8)
        layout.setContentsMargins(10, 10, 10, 10)

        # ── Header ──────────────────────────────────────────────────────────
        header = QWidget()
        header.setStyleSheet(f"background: {PANEL}; border-radius: 6px;")
        hl = QHBoxLayout(header)
        hl.setContentsMargins(16, 8, 16, 8)

        title_lbl = make_label("ROCKET TELEMETRY", 18, ACCENT, bold=True)
        title_lbl.setStyleSheet(title_lbl.styleSheet() +
            "letter-spacing: 4px; font-family: 'Courier New', monospace;")
        hl.addWidget(title_lbl)
        hl.addStretch()

        self.lbl_status = make_label("● NO SIGNAL", 11, ACCENT2)
        self.lbl_pkts   = make_label("0 pkts", 11, TEXT_DIM)
        self.lbl_hz     = make_label("0.0 pkt/s", 11, TEXT_DIM)  # pkt/s display
        for w in (self.lbl_status, self.lbl_pkts, self.lbl_hz):
            hl.addWidget(w)
            hl.addSpacing(20)

        layout.addWidget(header, 0, 0, 1, 3)

        # ── Big stat cards ───────────────────────────────────────────────────
        cards_widget = QWidget()
        cards_widget.setStyleSheet("background: transparent;")
        cards_layout = QHBoxLayout(cards_widget)
        cards_layout.setSpacing(8)
        cards_layout.setContentsMargins(0, 0, 0, 0)

        self.card_alt  = self._make_stat_card("ALTITUDE",      "m",    ACCENT)
        self.card_va   = self._make_stat_card("VERT ACCEL",    "m/s²", ACCENT2)
        self.card_temp = self._make_stat_card("TEMPERATURE",   "°C",   WARN)
        self.card_pres = self._make_stat_card("PRESSURE",      "hPa",  "#a0c4ff")
        self.card_gmax = self._make_stat_card("PEAK |ACCEL|",  "m/s²", ACCENT3)

        for c in (self.card_alt, self.card_va, self.card_temp,
                  self.card_pres, self.card_gmax):
            cards_layout.addWidget(c)

        layout.addWidget(cards_widget, 1, 0, 1, 3)

        # ── Altitude plot ────────────────────────────────────────────────────
        self.plot_alt = styled_plot("Altitude", "m")
        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen(ACCENT, width=2))
        self.curve_va  = self.plot_alt.plot(pen=pg.mkPen(ACCENT2, width=1.5))
        layout.addWidget(self.plot_alt, 2, 0, 1, 2)

        # ── Stacked accel panel ──────────────────────────────────────────────
        self.accel_panel = AccelPanel()
        layout.addWidget(self.accel_panel, 3, 0)

        # ── Gyro 3-axis plot ─────────────────────────────────────────────────
        self.plot_gyro = styled_plot("Gyroscope", "rad/s")
        self.curve_gx = self.plot_gyro.plot(pen=pg.mkPen(COLORS["gx"], width=1.5), name="X")
        self.curve_gy = self.plot_gyro.plot(pen=pg.mkPen(COLORS["gy"], width=1.5), name="Y")
        self.curve_gz = self.plot_gyro.plot(pen=pg.mkPen(COLORS["gz"], width=1.5), name="Z")
        layout.addWidget(self.plot_gyro, 3, 1)

        # ── Right panel: orientation ─────────────────────────────────────────
        orient_panel = make_panel()
        ol = QVBoxLayout(orient_panel)
        ol.setContentsMargins(12, 12, 12, 12)
        ol.setSpacing(4)

        ol.addWidget(make_label("ORIENTATION", 10, TEXT_DIM, bold=True))
        ol.addSpacing(4)

        self.lbl_roll  = self._orient_row(ol, "Roll")
        self.lbl_pitch = self._orient_row(ol, "Pitch")
        self.lbl_yaw   = self._orient_row(ol, "Yaw")

        ol.addSpacing(6)
        ol.addWidget(make_label("QUATERNION", 9, TEXT_DIM))
        self.lbl_qw = self._orient_row(ol, "W", size=11)
        self.lbl_qx = self._orient_row(ol, "X", size=11)
        self.lbl_qy = self._orient_row(ol, "Y", size=11)
        self.lbl_qz = self._orient_row(ol, "Z", size=11)

        ol.addSpacing(8)
        ol.addWidget(make_label("3D VIEW", 9, TEXT_DIM, bold=True))

        # OpenGL 3D widget
        self.quat_gl = QuaternionWidget()
        ol.addWidget(self.quat_gl, stretch=1)

        layout.addWidget(orient_panel, 2, 2, 2, 1)

        # Row/col stretch
        layout.setRowStretch(2, 2)
        layout.setRowStretch(3, 3)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 3)
        layout.setColumnStretch(2, 1)

        self._peak_accel = 0.0

    def _make_stat_card(self, title: str, unit: str, color: str) -> QWidget:
        panel = make_panel()
        vl = QVBoxLayout(panel)
        vl.setContentsMargins(14, 10, 14, 10)
        vl.setSpacing(2)
        lbl_title = make_label(title, 9, TEXT_DIM, bold=True)
        lbl_val   = make_value_label("—", 26, color)
        lbl_unit  = make_label(unit, 10, TEXT_DIM)
        vl.addWidget(lbl_title)
        vl.addWidget(lbl_val)
        vl.addWidget(lbl_unit)
        panel._val_label = lbl_val
        return panel

    def _orient_row(self, layout, name: str, size: int = 13) -> QLabel:
        row = QWidget()
        row.setStyleSheet("background: transparent;")
        rl = QHBoxLayout(row)
        rl.setContentsMargins(0, 1, 0, 1)
        rl.addWidget(make_label(name, 10, TEXT_DIM))
        lbl = make_value_label("—", size, TEXT)
        rl.addWidget(lbl)
        layout.addWidget(row)
        return lbl

    # ── Receiver ─────────────────────────────────────────────────────────────
    def _start_receiver(self):
        self.receiver = Receiver(UDP_PORT)
        self.receiver.packet_received.connect(self.store.ingest)
        t = threading.Thread(target=self.receiver.run, daemon=True)
        t.start()

    # ── Refresh ──────────────────────────────────────────────────────────────
    def _refresh(self):
        age = time.time() - self.store.last_pkt_time
        connected = age < 2.0

        # ── Packet rate ──────────────────────────────────────────────────────
        pps = self.store.packets_per_second()
        self._hz_samples.append(pps)
        pps_avg = sum(self._hz_samples) / len(self._hz_samples)

        # Color-code pkt/s: green ≥50, yellow ≥15, red <15
        if pps_avg >= 50:
            hz_color = ACCENT3
        elif pps_avg >= 15:
            hz_color = WARN
        else:
            hz_color = ACCENT2

        self.lbl_hz.setText(f"{pps_avg:.0f} pkt/s")
        self.lbl_hz.setStyleSheet(f"color: {hz_color}; font-size:11px; font-family:'Courier New',monospace;")

        if connected:
            self.lbl_status.setText("● LIVE")
            self.lbl_status.setStyleSheet(f"color: {ACCENT3}; font-size:11px;")
        else:
            self.lbl_status.setText("● NO SIGNAL")
            self.lbl_status.setStyleSheet(f"color: {ACCENT2}; font-size:11px;")

        pkt = self.store.latest
        if not pkt:
            return

        # ── Stat cards ───────────────────────────────────────────────────────
        self.card_alt._val_label.setText(f"{pkt.get('alt', 0):.1f}")
        self.card_va._val_label.setText(f"{pkt.get('va', 0):+.2f}")
        self.card_temp._val_label.setText(f"{pkt.get('temp', 0):.1f}")
        self.card_pres._val_label.setText(f"{pkt.get('pres', 0)/100:.1f}")

        accel_mag = math.sqrt(pkt.get("ax",0)**2 + pkt.get("ay",0)**2 + pkt.get("az",0)**2)
        self._peak_accel = max(self._peak_accel, accel_mag)
        self.card_gmax._val_label.setText(f"{self._peak_accel:.1f}")

        # ── Packet count ─────────────────────────────────────────────────────
        self.lbl_pkts.setText(f"{self.store.pkt_count} pkts")

        if all(k in pkt for k in ("qw","qx","qy","qz")):
            qw, qx, qy, qz = pkt["qw"], pkt["qx"], pkt["qy"], pkt["qz"]

            norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

            if norm < 1e-3:
                # reject bad data
                qw, qx, qy, qz = self._last_quat
            else:
                self._last_quat = (qw, qx, qy, qz)
        else:
            qw, qx, qy, qz = self._last_quat
        # Always update stored values on the GL widget directly
        # so Qt-driven repaints never render stale defaults
        self.quat_gl.qw = qw
        self.quat_gl.qx = qx
        self.quat_gl.qy = qy
        self.quat_gl.qz = qz

        sinr = 2*(qw*qx + qy*qz)
        cosr = 1 - 2*(qx*qx + qy*qy)
        roll = math.degrees(math.atan2(sinr, cosr))

        sinp = 2*(qw*qy - qz*qx)
        pitch = math.degrees(math.asin(max(-1, min(1, sinp))))

        siny = 2*(qw*qz + qx*qy)
        cosy = 1 - 2*(qy*qy + qz*qz)
        yaw  = math.degrees(math.atan2(siny, cosy))

        self.lbl_roll.setText(f"{roll:+7.1f}°")
        self.lbl_pitch.setText(f"{pitch:+7.1f}°")
        self.lbl_yaw.setText(f"{yaw:+7.1f}°")
        self.lbl_qw.setText(f"{qw:.4f}")
        self.lbl_qx.setText(f"{qx:.4f}")
        self.lbl_qy.setText(f"{qy:.4f}")
        self.lbl_qz.setText(f"{qz:.4f}")

        # Update 3D view
        self.quat_gl.set_quaternion(qw, qx, qy, qz)

        # ── Plot data ────────────────────────────────────────────────────────
        t, alt, va = self.store.arrays("alt", "va")
        if len(t) > 1:
            self.curve_alt.setData(t, alt)
            self.curve_va.setData(t, va)

        t, ax, ay, az = self.store.arrays("ax", "ay", "az")
        self.accel_panel.update_data(t, ax, ay, az)

        t, gx, gy, gz = self.store.arrays("gx", "gy", "gz")
        if len(t) > 1:
            self.curve_gx.setData(t, gx)
            self.curve_gy.setData(t, gy)
            self.curve_gz.setData(t, gz)

    def closeEvent(self, event):
        self.receiver.stop()
        super().closeEvent(event)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window, QColor(BG))
    pal.setColor(QPalette.ColorRole.WindowText, QColor(TEXT))
    app.setPalette(pal)

    win = GroundStation()
    win.show()
    sys.exit(app.exec())