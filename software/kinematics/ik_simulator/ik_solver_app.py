"""
is-robotic-arm - IK (Inverse Kinematics) interactive debugger
------------------------------------------------
PyQt5 + pyqtgraph (OpenGL rendering) application to visualize and debug
the analytical inverse kinematics model of a 5DOF robotic arm in real time.

Convention used: standard DH.
Controlled via spatial position (X, Y, Z) and tool orientation (Pitch, Roll).
"""

import sys
import math
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph.opengl as gl


# ----------------------------------------------------------------------
# Cinematique & Mathematiques
# ----------------------------------------------------------------------

def dh_matrix(alpha, a, d, theta):
    """Standard DH homogeneous transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

def forward_kinematics(dh_rows, thetas_deg):
    """FK used here only to render the IK results in 3D."""
    T = np.eye(4)
    frames = [T.copy()]
    for row, th_deg in zip(dh_rows, thetas_deg):
        A = dh_matrix(np.radians(row['alpha_deg']), row['a'], row['d'],
                      np.radians(th_deg + row['offset_deg']))
        T = T @ A
        frames.append(T.copy())
    return frames

def compute_ik_5dof(x, y, z, pitch_deg, roll_deg, d1, a2, a3, elbow_down=True):
    """
    Analytical 5-DOF IK solver.
    Returns (success_bool, thetas_deg_list_or_error_message)
    """
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    
    # 1. Base orientation
    theta_1 = math.atan2(y, x)
    
    # 2. 2D projection
    r_c = math.sqrt(x**2 + y**2)
    z_c = z - d1
    D_sq = r_c**2 + z_c**2
    D = math.sqrt(D_sq)
    
    # 3. Reachability test
    if D > (a2 + a3):
        return False, f"Out of reach: too far (D={D:.3f}m > Max={a2+a3:.3f}m)"
    if D < abs(a2 - a3):
        return False, f"Out of reach: too close (D={D:.3f}m < Min={abs(a2-a3):.3f}m)"
        
    # 4. Joint 3 (Elbow)
    C_3 = (D_sq - a2**2 - a3**2) / (2 * a2 * a3)
    C_3 = max(min(C_3, 1.0), -1.0) 
    S_3_mag = math.sqrt(1 - C_3**2)
    S_3 = S_3_mag if elbow_down else -S_3_mag 
    theta_3 = math.atan2(S_3, C_3)
    
    # 5. Joint 2 (Shoulder)
    beta = math.atan2(z_c, r_c)
    C_psi = (D_sq + a2**2 - a3**2) / (2 * D * a2)
    C_psi = max(min(C_psi, 1.0), -1.0)
    psi = math.atan2(math.sqrt(1 - C_psi**2), C_psi)
    theta_2 = beta - psi if elbow_down else beta + psi
        
    # 6. Joints 4 & 5 (Orientation)
    theta_4 = pitch - theta_2 - theta_3
    theta_5 = roll
    
    return True, [math.degrees(theta_1), math.degrees(theta_2), math.degrees(theta_3), 
                  math.degrees(theta_4), math.degrees(theta_5)]


# ----------------------------------------------------------------------
# Palette theme sombre (Identique au FK)
# ----------------------------------------------------------------------
BG = '#1a1d23'
PANEL = '#22262e'
BORDER = '#343a46'
TEXT = '#d8dbe0'
TEXT_DIM = '#8a8f99'
ACCENT = '#4fa3ff'
OK_COLOR = '#4caf6f'
WARN_COLOR = '#e81123'
LINK_COLOR = (0.95, 0.65, 0.20, 1.0)
JOINT_COLOR = (1.0, 0.85, 0.4, 1.0)
TARGET_OK_COLOR = (0.3, 0.8, 0.4, 0.8)
TARGET_ERR_COLOR = (0.9, 0.2, 0.2, 0.8)

STYLESHEET = f"""
QWidget {{ background-color: {BG}; color: {TEXT}; font-family: 'Segoe UI', sans-serif; font-size: 12px; }}
QMainWindow::separator {{ background: {BORDER}; width: 1px; }}
QGroupBox {{ background-color: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px; margin-top: 14px; padding: 10px 8px 8px 8px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {ACCENT}; }}
QLabel {{ background: transparent; color: {TEXT_DIM}; }}
QLabel[role="value"] {{ color: {TEXT}; font-family: 'Consolas', monospace; }}
QSlider::groove:horizontal {{ height: 4px; background: {BORDER}; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: {ACCENT}; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }}
QDoubleSpinBox {{ background-color: #14161b; border: 1px solid {BORDER}; border-radius: 4px; padding: 2px 4px; color: {TEXT}; }}
QPushButton {{ background-color: #2b3038; border: 1px solid {BORDER}; border-radius: 4px; padding: 6px 10px; color: {TEXT}; }}
QPushButton:hover {{ background-color: #343a46; border: 1px solid {ACCENT}; }}
QTextEdit {{ background-color: #14161b; border: 1px solid {BORDER}; border-radius: 4px; color: {TEXT}; font-family: 'Consolas', monospace; font-size: 13px; }}
QCheckBox {{ color: {TEXT}; font-weight: bold; }}
"""

# ----------------------------------------------------------------------
# Composants UI
# ----------------------------------------------------------------------

class TargetSlider(QtWidgets.QWidget):
    """Generic slider for coordinates (m) or angles (deg)"""
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, name, min_v, max_v, init_v, is_angle=False):
        super().__init__()
        self.is_angle = is_angle
        self.scale = 10.0 if is_angle else 1000.0 # Precision: 0.1 deg or 1mm
        self.unit = "°" if is_angle else " m"
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        
        self.label = QtWidgets.QLabel(name)
        self.label.setFixedWidth(100)
        
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(int(min_v * self.scale))
        self.slider.setMaximum(int(max_v * self.scale))
        self.slider.setValue(int(init_v * self.scale))
        
        self.val_lbl = QtWidgets.QLabel()
        self.val_lbl.setProperty("role", "value")
        self.val_lbl.setFixedWidth(60)
        
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.val_lbl)
        
        self.slider.valueChanged.connect(self._on_change)
        self._on_change(self.slider.value()) # init label

    def _on_change(self, v):
        real_v = v / self.scale
        fmt = f"{real_v:+.1f}{self.unit}" if self.is_angle else f"{real_v:+.3f}{self.unit}"
        self.val_lbl.setText(fmt)
        self.valueChanged.emit(real_v)

    def set_value(self, v):
        self.slider.setValue(int(v * self.scale))

class CustomTitleBar(QtWidgets.QWidget):
    # (Same as the FK script)
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.setFixedHeight(35)
        self.setStyleSheet(f"QWidget {{ background-color: #14161b; border-bottom: 1px solid {BORDER}; }} QLabel {{ color: {TEXT_DIM}; font-weight: 600; padding-left: 14px; border: none; }} QPushButton {{ background: transparent; border: none; font-size: 14px; color: {TEXT_DIM}; }} QPushButton:hover {{ background-color: {PANEL}; color: {TEXT}; }} QPushButton#closeBtn:hover {{ background-color: #e81123; color: white; }}")
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QtWidgets.QLabel("is-robotic-arm — IK Solver Debugger"))
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
# Application Principale
# ----------------------------------------------------------------------

class IKApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.resize(1300, 800)
        self.setStyleSheet(STYLESHEET + f" #Main {{ border: 1px solid {BORDER}; }}")
        
        # Default geometry (physical arm parameters)
        self.geom = {'d1': 0.25, 'a2': 0.30, 'a3': 0.25}
        # Current state
        self.target = {'x': 0.20, 'y': 0.15, 'z': 0.15, 'pitch': -90.0, 'roll': 0.0}
        self.elbow_down = True
        self.last_valid_thetas = [0, 0, 0, 0, 0]

        main_w = QtWidgets.QWidget(); main_w.setObjectName("Main")
        self.setCentralWidget(main_w)
        layout = QtWidgets.QVBoxLayout(main_w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        layout.addWidget(CustomTitleBar(self))
        
        content = QtWidgets.QWidget()
        c_layout = QtWidgets.QHBoxLayout(content)
        c_layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(content, stretch=1)
        
        c_layout.addWidget(self._build_3d_view(), stretch=2)
        c_layout.addWidget(self._build_panel(), stretch=1)
        
        self._init_gl_items()
        self.update_ik()

    def _build_3d_view(self):
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor(BG)
        self.gl_view.setCameraPosition(distance=1.6, elevation=25, azimuth=45)
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
        
        # Marqueur spherique representant la cible spatiale (x,y,z)
        self.target_marker = gl.GLScatterPlotItem(size=20)
        self.gl_view.addItem(self.target_marker)

    def _build_panel(self):
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(450)
        v = QtWidgets.QVBoxLayout(panel)
        v.setSpacing(10)
        
        # Target parameters
        gb_target = QtWidgets.QGroupBox("End-Effector Target (IK Target)")
        vt = QtWidgets.QVBoxLayout(gb_target)
        
        self.s_x = TargetSlider("X (Avant)", -0.6, 0.6, self.target['x'])
        self.s_y = TargetSlider("Y (Gauche)", -0.6, 0.6, self.target['y'])
        self.s_z = TargetSlider("Z (Haut)", 0.0, 0.8, self.target['z'])
        self.s_p = TargetSlider("Pitch Outil", -180, 180, self.target['pitch'], True)
        self.s_r = TargetSlider("Roll Outil", -180, 180, self.target['roll'], True)
        
        for s, k in [(self.s_x,'x'), (self.s_y,'y'), (self.s_z,'z'), (self.s_p,'pitch'), (self.s_r,'roll')]:
            s.valueChanged.connect(self._make_tgt_cb(k))
            vt.addWidget(s)
            
        self.cb_posture = QtWidgets.QCheckBox("Posture: Elbow Down")
        self.cb_posture.setChecked(self.elbow_down)
        self.cb_posture.stateChanged.connect(self._on_posture_changed)
        vt.addWidget(self.cb_posture)
        v.addWidget(gb_target)
        
        # Physical dimensions
        gb_geom = QtWidgets.QGroupBox("Arm Geometry (m)")
        vg = QtWidgets.QHBoxLayout(gb_geom)
        self.boxes = {}
        for lbl, key in [("Base (d1)", "d1"), ("Upper arm (a2)", "a2"), ("Forearm (a3)", "a3")]:
            lv = QtWidgets.QVBoxLayout()
            lv.addWidget(QtWidgets.QLabel(lbl))
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(0.01, 1.0)
            box.setSingleStep(0.01)
            box.setDecimals(3)
            box.setValue(self.geom[key])
            box.valueChanged.connect(self._make_geom_cb(key))
            lv.addWidget(box)
            vg.addLayout(lv)
        v.addWidget(gb_geom)
        
        # Readout (output)
        gb_out = QtWidgets.QGroupBox("Inverse Kinematics Result")
        vo = QtWidgets.QVBoxLayout(gb_out)
        self.readout = QtWidgets.QTextEdit()
        self.readout.setReadOnly(True)
        vo.addWidget(self.readout)
        v.addWidget(gb_out)
        
        btn_reset = QtWidgets.QPushButton("Reset Target")
        btn_reset.clicked.connect(self._reset_target)
        v.addWidget(btn_reset)
        
        v.addStretch()
        return panel

    def _make_tgt_cb(self, key):
        def cb(val):
            self.target[key] = val
            self.update_ik()
        return cb
        
    def _make_geom_cb(self, key):
        def cb(val):
            self.geom[key] = val
            self.update_ik()
        return cb

    def _on_posture_changed(self, state):
        self.elbow_down = bool(state)
        self.update_ik()

    def _reset_target(self):
        self.s_x.set_value(0.20); self.s_y.set_value(0.15); self.s_z.set_value(0.15)
        self.s_p.set_value(-90.0); self.s_r.set_value(0.0)

    def update_ik(self):
        """Computes IK, updates the text and the 3D model."""
        # 1. Update the target marker (ghost)
        tx, ty, tz = self.target['x'], self.target['y'], self.target['z']
        self.target_marker.setData(pos=np.array([[tx, ty, tz]]))
        
        # 2. Call the IK math solver
        success, result = compute_ik_5dof(
            tx, ty, tz, self.target['pitch'], self.target['roll'],
            self.geom['d1'], self.geom['a2'], self.geom['a3'],
            self.elbow_down
        )
        
        if success:
            self.last_valid_thetas = result
            self.target_marker.setData(color=TARGET_OK_COLOR)
            
            txt = "<span style='color:#4caf6f; font-weight:bold;'>[SUCCESS] Target reached.</span><br><br>"
            txt += "<b>Angles generated for the motors:</b><br>"
            for i, th in enumerate(result):
                txt += f"&nbsp;&nbsp;Joint {i+1} : <span style='color:#4fa3ff'>{th:>+7.2f}°</span><br>"
                
            self.readout.setHtml(txt)
        else:
            self.target_marker.setData(color=TARGET_ERR_COLOR)
            txt = f"<span style='color:#e81123; font-weight:bold;'>[ERROR] {result}</span><br><br>"
            txt += "<span style='color:#8a8f99'>The arm stays at its last valid position.</span>"
            self.readout.setHtml(txt)

        # 3. 3D rendering using the FK model with the valid thetas
        dh = [
            {'alpha_deg': 90.0, 'a': 0.0,  'd': self.geom['d1'], 'offset_deg': 0.0},
            {'alpha_deg': 0.0,  'a': self.geom['a2'], 'd': 0.0,  'offset_deg': 0.0},
            {'alpha_deg': 0.0,  'a': self.geom['a3'], 'd': 0.0,  'offset_deg': 0.0},
            {'alpha_deg': 90.0, 'a': 0.0,  'd': 0.0,  'offset_deg': 90.0},
            {'alpha_deg': 0.0,  'a': 0.0,  'd': 0.0,  'offset_deg': 0.0},
        ]
        
        # Compute the frames from the valid thetas
        frames = forward_kinematics(dh, self.last_valid_thetas)
        
        # Extract the origins to draw the lines
        arm_origins = np.array([T[:3, 3] for T in frames])
        self.link_item.setData(pos=arm_origins)
        self.joint_markers.setData(pos=arm_origins)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = IKApp()
    win.show()
    sys.exit(app.exec_())
