"""
is-robotic-arm - DH / FK interactive debugger & Serial Controller
-----------------------------------------------------------------
PyQt5 + pyqtgraph application (OpenGL rendering) to visualize and debug
the DH model of a 5DOF robotic arm in real-time.
Now includes asynchronous serial communication to teleoperate the physical 
Arduino Mega without blocking the UI or overloading the hardware.

Convention used: Standard DH, A_i = Rz(theta_i+offset_i)·Tz(d_i)·Tx(a_i)·Rx(alpha_i)
World frame: X=forward, Y=left, Z=up (ROS REP-103 convention)
"""

import sys
import numpy as np
import serial
from PyQt5 import QtWidgets, QtCore
import pyqtgraph.opengl as gl

# ----------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------

def dh_matrix(alpha, a, d, theta):
    """Builds the 4x4 homogeneous transformation matrix of a single joint,
    from its 4 DH parameters (standard convention)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])


def forward_kinematics(dh_rows, thetas_deg):
    """Calculates the complete FK: builds each A_i matrix then chains them
    (T0->T1->...->T5) by multiplication. Used theta_i = physical 
    slider angle + constant offset of the corresponding DH row."""
    T = np.eye(4)
    frames = [T.copy()]
    for row, th_deg in zip(dh_rows, thetas_deg):
        A = dh_matrix(np.radians(row['alpha_deg']), row['a'], row['d'],
                      np.radians(th_deg + row['offset_deg']))
        T = T @ A
        frames.append(T.copy())
    return frames


def joint_axis_world(frames, joint_index_1based):
    """Actual direction, in the world frame, of the rotation axis of
    joint i (1..5)."""
    return frames[joint_index_1based - 1][:3, 2]


def rotation_to_rpy_deg(R):
    """Extracts roll/pitch/yaw (ZYX convention) in degrees from a 3x3
    rotation matrix, for readable display in the readout panel."""
    pitch = np.degrees(np.arcsin(-np.clip(R[2, 0], -1.0, 1.0)))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


JOINT_NAMES = ["J1 base (yaw)", "J2 epaule (pitch)", "J3 coude (pitch)",
               "J4 poignet1 (pitch)", "J5 poignet2 (roll)"]

DEFAULT_DH = [
    {'alpha_deg': 90.0, 'a': 0.0,  'd': 0.20, 'offset_deg':  0.0},   # J1 base  (d1=0.20m)
    {'alpha_deg':  0.0, 'a': 0.20, 'd': 0.0,  'offset_deg': 90.0},   # J2 épaule (a2=0.20m, 0°=haut)
    {'alpha_deg':  0.0, 'a': 0.20, 'd': 0.0,  'offset_deg':  0.0},   # J3 coude  (a3=0.20m)
    {'alpha_deg': 90.0, 'a': 0.0,  'd': 0.0,  'offset_deg': 90.0},   # J4 poignet pitch
    {'alpha_deg':  0.0, 'a': 0.0,  'd': 0.05, 'offset_deg':  0.0},   # J5 roll   (d5=0.05m)
]


# ----------------------------------------------------------------------
# Dark theme palette & UI Components
# ----------------------------------------------------------------------
BG = '#1a1d23'
PANEL = '#22262e'
BORDER = '#343a46'
TEXT = '#d8dbe0'
TEXT_DIM = '#8a8f99'
ACCENT = '#4fa3ff'
OK_COLOR = '#4caf6f'
WARN_COLOR = '#e0724a'
AXIS_X = (0.95, 0.35, 0.35, 1.0)
AXIS_Y = (0.40, 0.85, 0.55, 1.0)
AXIS_Z = (0.35, 0.65, 0.98, 1.0)
LINK_COLOR = (0.95, 0.65, 0.20, 1.0)
JOINT_COLOR = (1.0, 0.85, 0.4, 1.0)

STYLESHEET = f"""
QWidget {{ background-color: {BG}; color: {TEXT}; font-family: 'Segoe UI', 'Helvetica Neue', sans-serif; font-size: 12px; }}
QMainWindow::separator {{ background: {BORDER}; width: 1px; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 6px; top: -1px; }}
QTabBar::tab {{ background: {PANEL}; color: {TEXT_DIM}; padding: 7px 16px; border: 1px solid {BORDER}; border-bottom: none; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }}
QTabBar::tab:selected {{ background: #2a2f38; color: {ACCENT}; font-weight: 600; }}
QGroupBox {{ background-color: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px; margin-top: 14px; padding: 10px 8px 8px 8px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {ACCENT}; }}
QLabel {{ background: transparent; color: {TEXT_DIM}; }}
QLabel[role="value"] {{ color: {TEXT}; }}
QDoubleSpinBox {{ background-color: #14161b; border: 1px solid {BORDER}; border-radius: 4px; padding: 2px 4px; color: {TEXT}; min-width: 62px; }}
QDoubleSpinBox:focus {{ border: 1px solid {ACCENT}; }}
QSlider::groove:horizontal {{ height: 4px; background: {BORDER}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: {ACCENT}; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 2px; }}
QPushButton {{ background-color: #2b3038; border: 1px solid {BORDER}; border-radius: 4px; padding: 6px 10px; color: {TEXT}; }}
QPushButton:hover {{ background-color: #343a46; border: 1px solid {ACCENT}; }}
QScrollArea {{ border: none; }}
QTextEdit {{ background-color: #14161b; border: 1px solid {BORDER}; border-radius: 4px; color: {TEXT}; font-family: 'Consolas', 'Menlo', monospace; font-size: 11px; }}
"""


class ThetaSlider(QtWidgets.QWidget):
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, name, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        self.label = QtWidgets.QLabel(name)
        self.label.setFixedWidth(130)
        
        # Le Slider (fonctionne en dixièmes de degrés pour la précision)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(-1800)
        self.slider.setMaximum(1800)
        self.slider.setValue(0)
        
        # Le Champ de texte numérique précis (SpinBox)
        self.spinbox = QtWidgets.QDoubleSpinBox()
        self.spinbox.setRange(-180.0, 180.0)
        self.spinbox.setSingleStep(1.0)
        self.spinbox.setDecimals(1)
        self.spinbox.setSuffix(" °")
        self.spinbox.setFixedWidth(75)

        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.spinbox)

        # Synchronisation bidirectionnelle
        self.slider.valueChanged.connect(self._slider_changed)
        self.spinbox.valueChanged.connect(self._spinbox_changed)

    def _slider_changed(self, v):
        deg = v / 10.0
        # On bloque temporairement le signal de la spinbox pour éviter une boucle infinie
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(deg)
        self.spinbox.blockSignals(False)
        self.valueChanged.emit(deg)

    def _spinbox_changed(self, deg):
        # On bloque temporairement le signal du slider
        self.slider.blockSignals(True)
        self.slider.setValue(int(deg * 10))
        self.slider.blockSignals(False)
        self.valueChanged.emit(deg)

    def set_value(self, deg):
        self.spinbox.setValue(deg) # Ceci déclenchera automatiquement _spinbox_changed


class DHRow(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()

    def __init__(self, name, row, parent=None):
        super().__init__(parent)
        self.row = row
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        label = QtWidgets.QLabel(name)
        label.setFixedWidth(130)
        layout.addWidget(label)

        self.boxes = {}
        specs = [('alpha_deg', -180, 180, 1), ('a', -2.0, 2.0, 0.001),
                 ('d', -2.0, 2.0, 0.001), ('offset_deg', -360, 360, 1)]
        for key, lo, hi, step in specs:
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(lo, hi)
            box.setSingleStep(step)
            box.setDecimals(3 if key in ('a', 'd') else 1)
            box.setValue(row[key])
            box.valueChanged.connect(self._make_cb(key))
            layout.addWidget(box)
            self.boxes[key] = box

    def _make_cb(self, key):
        def cb(val):
            self.row[key] = val
            self.changed.emit()
        return cb


class AxisLegend(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(14)
        for name, color in [("X", AXIS_X), ("Y", AXIS_Y), ("Z", AXIS_Z), ("liens", LINK_COLOR)]:
            dot = QtWidgets.QLabel("●")
            rgb = tuple(int(c * 255) for c in color[:3])
            dot.setStyleSheet(f"color: rgb{rgb}; font-size: 14px;")
            txt = QtWidgets.QLabel(name)
            txt.setStyleSheet(f"color: {TEXT_DIM};")
            layout.addWidget(dot)
            layout.addWidget(txt)
        layout.addStretch()


class GeometryCheckPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout_ = QtWidgets.QVBoxLayout(self)
        self.layout_.setSpacing(4)
        self.rows = {}
        checks = [
            "J1 (base) axe vertical",
            "J2 / J3 paralleles (epaule-coude)",
            "J3 / J4 paralleles (coude-poignet1)",
            "J5 perpendiculaire a J4 (roll vs dernier pitch)",
        ]
        for key in checks:
            row = QtWidgets.QHBoxLayout()
            dot = QtWidgets.QLabel("●")
            dot.setFixedWidth(16)
            lbl = QtWidgets.QLabel(key)
            row.addWidget(dot)
            row.addWidget(lbl)
            row.addStretch()
            self.layout_.addLayout(row)
            self.rows[key] = dot

    def update_checks(self, frames):
        def axis(i): return joint_axis_world(frames, i)
        def is_parallel(u, v, tol=1e-2): return abs(abs(np.dot(u, v)) - 1.0) < tol
        def is_perp(u, v, tol=1e-2): return abs(np.dot(u, v)) < tol

        results = {
            "J1 (base) axe vertical": is_parallel(axis(1), np.array([0, 0, 1])),
            "J2 / J3 paralleles (epaule-coude)": is_parallel(axis(2), axis(3)),
            "J3 / J4 paralleles (coude-poignet1)": is_parallel(axis(3), axis(4)),
            "J5 perpendiculaire a J4 (roll vs dernier pitch)": is_perp(axis(4), axis(5)),
        }
        for key, ok in results.items():
            color = OK_COLOR if ok else WARN_COLOR
            self.rows[key].setStyleSheet(f"color: {color}; font-size: 14px;")


class CustomTitleBar(QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.setFixedHeight(35)
        self.setStyleSheet(f"""
            QWidget {{ background-color: #14161b; border-bottom: 1px solid {BORDER}; }}
            QLabel {{ color: {TEXT_DIM}; font-weight: 600; font-size: 13px; padding-left: 14px; border: none; }}
            QPushButton {{ background: transparent; border: none; font-size: 14px; color: {TEXT_DIM}; }}
            QPushButton:hover {{ background-color: {PANEL}; color: {TEXT}; }}
            QPushButton#closeBtn:hover {{ background-color: #e81123; color: white; }}
        """)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.title_label = QtWidgets.QLabel("is-robotic-arm — DH / FK debugger & Teleoperation")
        layout.addWidget(self.title_label)
        layout.addStretch()
        
        self.btn_min = QtWidgets.QPushButton("—")
        self.btn_min.setFixedSize(45, 35)
        self.btn_min.clicked.connect(self.parent.showMinimized)
        layout.addWidget(self.btn_min)
        
        self.btn_close = QtWidgets.QPushButton("✕")
        self.btn_close.setObjectName("closeBtn")
        self.btn_close.setFixedSize(45, 35)
        self.btn_close.clicked.connect(self.parent.close)
        layout.addWidget(self.btn_close)
        self.offset = None

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton: self.offset = event.pos()

    def mouseMoveEvent(self, event):
        if self.offset is not None: self.parent.move(event.globalPos() - self.offset)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton: self.offset = None


# ----------------------------------------------------------------------
# Main Application
# ----------------------------------------------------------------------

class DHFKApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.resize(1480, 860)
        self.setStyleSheet(STYLESHEET + f" #MainContainer {{ border: 1px solid {BORDER}; background-color: {BG}; }}")

        self.dh = [row.copy() for row in DEFAULT_DH]
        self.thetas_deg = [0.0] * 5
        self.axis_len = 0.15

        main_container = QtWidgets.QWidget()
        main_container.setObjectName("MainContainer")
        self.setCentralWidget(main_container)

        global_layout = QtWidgets.QVBoxLayout(main_container)
        global_layout.setContentsMargins(0, 0, 0, 0)
        global_layout.setSpacing(0)

        self.title_bar = CustomTitleBar(self)
        global_layout.addWidget(self.title_bar)

        content_widget = QtWidgets.QWidget()
        content_layout = QtWidgets.QHBoxLayout(content_widget)
        content_layout.setContentsMargins(10, 10, 10, 10)
        global_layout.addWidget(content_widget, stretch=1)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._build_3d_view(), stretch=1)
        self.legend = AxisLegend()
        left.addWidget(self.legend)
        content_layout.addLayout(left, stretch=3)

        content_layout.addWidget(self._build_side_panel(), stretch=2)

        self._init_gl_items()
        self._rescale_view()
        self.update_scene()

        # ------------------------------------------------------------------
        # Serial Communication Setup (Smart Buffer / Teleoperation)
        # ------------------------------------------------------------------
        try:
            # Changez COM9 par votre port actuel si besoin
            self.ser = serial.Serial('COM8', 115200, timeout=0) 
            self.robot_ready = True
            print("Connected to Arduino Mega. Teleoperation active.")
        except Exception as e:
            self.ser = None
            self.robot_ready = False
            print(f"Simulation mode only (Arduino not detected: {e})")

        self.last_sent_thetas = [0.0] * 5
        # Résolutions ajustées empiriquement
        # J1 (1:16) x2, J2 (1:64) x1, J3 (1:64) x1, J4 (1:16) x2, J5 (Roll) x2
        self.steps_per_deg = [142.22, 568.888, 568.888, 142.22, 142.22]

        # Timer to handle non-blocking communication (Tick at 50Hz / 20ms)
        self.com_timer = QtCore.QTimer()
        self.com_timer.timeout.connect(self._serial_loop)
        self.com_timer.start(20)

    # ------------------------------------------------------------------
    # UI Builders
    # ------------------------------------------------------------------
    def _build_3d_view(self):
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor(BG)
        self.gl_view.setCameraPosition(distance=1.6, elevation=22, azimuth=35)
        grid = gl.GLGridItem()
        grid.setColor((60, 65, 75, 80))
        grid.setSize(2, 2)
        grid.setSpacing(0.1, 0.1)
        self.gl_view.addItem(grid)
        return self.gl_view

    def _init_gl_items(self):
        self.link_item = gl.GLLinePlotItem(color=LINK_COLOR, width=5, antialias=True)
        self.gl_view.addItem(self.link_item)
        self.joint_markers = gl.GLScatterPlotItem(color=JOINT_COLOR, size=14)
        self.gl_view.addItem(self.joint_markers)
        self.axis_x = gl.GLLinePlotItem(color=AXIS_X, width=2, mode='lines')
        self.axis_y = gl.GLLinePlotItem(color=AXIS_Y, width=2, mode='lines')
        self.axis_z = gl.GLLinePlotItem(color=AXIS_Z, width=2, mode='lines')
        for it in (self.axis_x, self.axis_y, self.axis_z): self.gl_view.addItem(it)

    def _rescale_view(self):
        reach = sum(r['a'] for r in self.dh) + sum(r['d'] for r in self.dh) + 0.05
        self.gl_view.setCameraPosition(distance=max(reach, 0.3) * 2.6)

    def _build_side_panel(self):
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(500)
        v = QtWidgets.QVBoxLayout(panel)
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_controls_tab(), "Controls")
        tabs.addTab(self._build_debug_tab(), "Debug / Validation")
        v.addWidget(tabs, stretch=1)
        reset_btn = QtWidgets.QPushButton("Reset angles (home)")
        reset_btn.clicked.connect(self._on_reset)
        v.addWidget(reset_btn)
        return panel

    def _build_controls_tab(self):
        tab = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(tab)
        
        gb_theta = QtWidgets.QGroupBox("Joint Angles (theta)")
        gb_theta_l = QtWidgets.QVBoxLayout(gb_theta)
        self.theta_sliders = []
        for i, name in enumerate(JOINT_NAMES):
            s = ThetaSlider(name)
            s.valueChanged.connect(self._make_theta_cb(i))
            gb_theta_l.addWidget(s)
            self.theta_sliders.append(s)
        v.addWidget(gb_theta)

        gb_dh = QtWidgets.QGroupBox("DH Table (alpha°, a[m], d[m], offset°)")
        gb_dh_l = QtWidgets.QVBoxLayout(gb_dh)
        self.dh_rows_widgets = []
        for i, name in enumerate(JOINT_NAMES):
            row_w = DHRow(name, self.dh[i])
            row_w.changed.connect(self._on_dh_changed)
            gb_dh_l.addWidget(row_w)
            self.dh_rows_widgets.append(row_w)
        v.addWidget(gb_dh)
        
        v.addStretch()
        return tab

    def _build_debug_tab(self):
        tab = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(tab)
        gb_check = QtWidgets.QGroupBox("Geometric Validation (live)")
        gb_check_l = QtWidgets.QVBoxLayout(gb_check)
        self.geo_panel = GeometryCheckPanel()
        gb_check_l.addWidget(self.geo_panel)
        v.addWidget(gb_check)

        gb_read = QtWidgets.QGroupBox("FK Readout")
        gb_read_l = QtWidgets.QVBoxLayout(gb_read)
        self.readout = QtWidgets.QTextEdit()
        self.readout.setReadOnly(True)
        gb_read_l.addWidget(self.readout)
        v.addWidget(gb_read)
        return tab

    def _make_theta_cb(self, idx):
        def cb(deg):
            self.thetas_deg[idx] = deg
            self.update_scene()
        return cb

    def _on_dh_changed(self):
        self._rescale_view()
        self.update_scene()

    def _on_reset(self):
        for s in self.theta_sliders: s.set_value(0.0)

    # ------------------------------------------------------------------
    # Serial Communication Logic (Smart Buffer)
    # ------------------------------------------------------------------
    def _serial_loop(self):
        """Background loop reading Arduino ACKs and sending new targets."""
        if not self.ser:
            return

        # 1. Listen to Arduino
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('ascii').strip()
                if "ACK:IDLE" in line:
                    self.robot_ready = True
            except:
                pass

        # 2. Send target if Arduino is ready and sliders have moved
        if self.robot_ready and self.thetas_deg != self.last_sent_thetas:
            
            # Convert degrees to hardware steps
            steps = [int(self.thetas_deg[i] * self.steps_per_deg[i]) for i in range(5)]
            
            # Format the robust transmission frame
            frame = f"<T,{steps[0]},{steps[1]},{steps[2]},{steps[3]},{steps[4]}>\n"
            
            self.ser.write(frame.encode('ascii'))
            self.last_sent_thetas = list(self.thetas_deg)
            self.robot_ready = False # Lock transmission until next ACK:IDLE

    # ------------------------------------------------------------------
    # Rendering & Calculations
    # ------------------------------------------------------------------
    def update_scene(self):
        dh_frames = forward_kinematics(self.dh, self.thetas_deg)
        T_world = np.eye(4); T_world[0, 3] = -0.4; T_world[1, 3] = -0.4
        display_frames = [T_world, dh_frames[0], dh_frames[1], dh_frames[2], dh_frames[3], dh_frames[5]]
        
        self._update_3d(display_frames)
        self._update_readout(dh_frames)
        self.geo_panel.update_checks(dh_frames)

    def _update_3d(self, display_frames):
        arm_origins = np.array([T[:3, 3] for T in display_frames[1:]])
        self.link_item.setData(pos=arm_origins)
        self.joint_markers.setData(pos=arm_origins)
        
        xs, ys, zs = [], [], []
        for T in display_frames:
            o = T[:3, 3]
            R = T[:3, :3]
            xs += [o, o + R[:, 0] * self.axis_len]
            ys += [o, o + R[:, 1] * self.axis_len]
            zs += [o, o + R[:, 2] * self.axis_len]

        self.axis_x.setData(pos=np.array(xs))
        self.axis_y.setData(pos=np.array(ys))
        self.axis_z.setData(pos=np.array(zs))

    def _update_readout(self, frames):
        T05 = frames[-1]
        pos = T05[:3, 3]
        roll, pitch, yaw = rotation_to_rpy_deg(T05[:3, :3])
        lines = ["EFFECTEUR (Par rapport au Repère 1 Base)",
                 f"  pos  x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}  [m]",
                 f"  rpy  r={roll:+7.2f}  p={pitch:+7.2f}  y={yaw:+7.2f}  [deg]",
                 "", "AXES DE ROTATION REELS DES JOINTS"]
        for i in range(1, 6):
            z_i = joint_axis_world(frames, i)
            lines.append(f"  J{i}  [{z_i[0]:+.2f}, {z_i[1]:+.2f}, {z_i[2]:+.2f}]")
        self.readout.setPlainText("\n".join(lines))


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = DHFKApp()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()