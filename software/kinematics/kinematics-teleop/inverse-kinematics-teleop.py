"""
is-robotic-arm - IK (Inverse Kinematics) interactive debugger & Teleoperation
------------------------------------------------
Application PyQt5 + pyqtgraph (rendu OpenGL) pour visualiser et debugger
en temps reel le modele Analytique Inverse d'un bras robotique 5DOF.
Intègre la téléopération série vers l'Arduino Mega avec Homing vertical.

Convention utilisee : DH standard.
Controle par position spatiale (X, Y, Z) et orientation d'outil (Pitch, Roll).

Géométrie bras réel :
  d1 = 0.20 m  (base  → épaule J2, axe vertical)
  a2 = 0.20 m  (épaule → coude J3)
  a3 = 0.20 m  (coude  → poignet J4)
  d5 = 0.05 m  (poignet J4 → outil J5, le long de l'axe outil)

Le solveur IK cible J5 (extrémité outil), pas J4.
"""

import sys
import math
import serial
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph.opengl as gl


# ----------------------------------------------------------------------
# Cinematique & Mathematiques
# ----------------------------------------------------------------------

def dh_matrix(alpha, a, d, theta):
    """Matrice de transformation homogene DH standard."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

def forward_kinematics(dh_rows, thetas_deg):
    """FK utilisé ici uniquement pour l'affichage 3D des resultats de l'IK."""
    T = np.eye(4)
    frames = [T.copy()]
    for row, th_deg in zip(dh_rows, thetas_deg):
        A = dh_matrix(np.radians(row['alpha_deg']), row['a'], row['d'],
                      np.radians(th_deg + row['offset_deg']))
        T = T @ A
        frames.append(T.copy())
    return frames

def compute_ik_5dof(x, y, z, pitch_deg, roll_deg, d1, a2, a3, d5, elbow_down=True):
    """
    Solveur Analytique IK 5-DOF.
    (x, y, z) = position de la POINTE OUTIL (J5), pas du centre poignet (J4).
    d5 = distance J4 → J5 le long de l'axe outil (pitch).
    Retourne (succes_bool, thetas_deg_list_ou_message_erreur)
    """
    pitch = math.radians(pitch_deg)

    # 1. Estimation initiale de theta_1 depuis la cible outil
    theta_1 = math.atan2(y, x)

    # 2. Calcul du centre poignet (J4) depuis la cible outil (J5)
    #    L'axe outil pointe à l'angle 'pitch' par rapport à l'horizontale,
    #    dans le plan vertical du bras (défini par theta_1).
    cos_t1 = math.cos(theta_1)
    sin_t1 = math.sin(theta_1)
    x_w = x - d5 * math.cos(pitch) * cos_t1
    y_w = y - d5 * math.cos(pitch) * sin_t1
    z_w = z - d5 * math.sin(pitch)

    # 2b. Recalculer theta_1 depuis le CENTRE POIGNET (plus robuste).
    #     Si le poignet est sur l'axe Z (bras vertical), atan2(0,0) est
    #     indéfini → garder theta_1 = 0 pour éviter un saut aléatoire.
    r_w = math.sqrt(x_w**2 + y_w**2)
    if r_w > 1e-4:   # poignet hors de l'axe Z → recalculer
        theta_1 = math.atan2(y_w, x_w)

    # 3. Projection 2D dans le plan du bras (vers le centre poignet)
    r_c = r_w  # déjà calculé en 2b
    z_c = z_w - d1
    D_sq = r_c**2 + z_c**2
    D = math.sqrt(D_sq)

    # 4. Test d'accessibilite (sur le centre poignet, pas l'outil)
    reach_max = a2 + a3
    reach_min = abs(a2 - a3)
    if D > reach_max + 1e-3:   # tolérance 1mm pour les erreurs flottantes
        return False, f"Hors de portée : Trop éloigné (D_poignet={D:.3f}m > Max={reach_max:.3f}m)"
    if D < reach_min:
        return False, f"Hors de portée : Trop proche (D_poignet={D:.3f}m < Min={reach_min:.3f}m)"

    # 5. Joint 3 (Coude)
    C_3 = (D_sq - a2**2 - a3**2) / (2 * a2 * a3)
    C_3 = max(min(C_3, 1.0), -1.0)
    S_3_mag = math.sqrt(1 - C_3**2)
    S_3 = S_3_mag if elbow_down else -S_3_mag
    theta_3 = math.atan2(S_3, C_3)

    # 6. Joint 2 (Epaule)
    beta = math.atan2(z_c, r_c)
    C_psi = (D_sq + a2**2 - a3**2) / (2 * D * a2)
    C_psi = max(min(C_psi, 1.0), -1.0)
    psi = math.atan2(math.sqrt(1 - C_psi**2), C_psi)
    theta_2 = beta - psi if elbow_down else beta + psi

    # 7. Joints 4 & 5 (Orientation)
    theta_4 = pitch - theta_2 - theta_3
    theta_5 = math.radians(roll_deg)   # roll direct

    # Convertir theta_2 en convention "0° = vertical haut" (au lieu de 0° = horizontal)
    return True, [math.degrees(theta_1), math.degrees(theta_2) - 90.0, math.degrees(theta_3),
                  math.degrees(theta_4), math.degrees(theta_5)]


# ----------------------------------------------------------------------
# Palette theme sombre
# ----------------------------------------------------------------------
BG = '#1a1d23'
PANEL = '#22262e'
BORDER = '#343a46'
TEXT = '#d8dbe0'
TEXT_DIM = '#8a8f99'
ACCENT = '#4fa3ff'
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
    """Slider generique pour coordonnees (m) ou angles (deg)"""
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, name, min_v, max_v, init_v, is_angle=False):
        super().__init__()
        self.is_angle = is_angle
        self.scale = 10.0 if is_angle else 1000.0
        self.min_v = min_v
        self.max_v = max_v

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)

        self.label = QtWidgets.QLabel(name)
        self.label.setFixedWidth(100)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(int(min_v * self.scale))
        self.slider.setMaximum(int(max_v * self.scale))
        self.slider.setValue(int(init_v * self.scale))

        # SpinBox with up/down arrows (like DH_Interactive)
        self.spinbox = QtWidgets.QDoubleSpinBox()
        self.spinbox.setRange(min_v, max_v)
        self.spinbox.setSingleStep(1.0 if is_angle else 0.01)
        self.spinbox.setDecimals(1 if is_angle else 3)
        self.spinbox.setSuffix(" °" if is_angle else " m")
        self.spinbox.setFixedWidth(80)
        self.spinbox.setValue(init_v)

        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.spinbox)

        # Bidirectional sync (block signals to avoid infinite loop)
        self.slider.valueChanged.connect(self._slider_changed)
        self.spinbox.valueChanged.connect(self._spinbox_changed)

    def _slider_changed(self, v):
        real_v = v / self.scale
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(real_v)
        self.spinbox.blockSignals(False)
        self.valueChanged.emit(real_v)

    def _spinbox_changed(self, val):
        self.slider.blockSignals(True)
        self.slider.setValue(int(val * self.scale))
        self.slider.blockSignals(False)
        self.valueChanged.emit(val)

    def set_value(self, v):
        self.spinbox.setValue(v)  # triggers _spinbox_changed -> slider sync


class CustomTitleBar(QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.setFixedHeight(35)
        self.setStyleSheet(
            f"QWidget {{ background-color: #14161b; border-bottom: 1px solid {BORDER}; }}"
            f"QLabel {{ color: {TEXT_DIM}; font-weight: 600; padding-left: 14px; border: none; }}"
            f"QPushButton {{ background: transparent; border: none; font-size: 14px; color: {TEXT_DIM}; }}"
            f"QPushButton:hover {{ background-color: {PANEL}; color: {TEXT}; }}"
            f"QPushButton#closeBtn:hover {{ background-color: #e81123; color: white; }}"
        )
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QtWidgets.QLabel("is-robotic-arm — IK Solver Debugger & Teleoperation"))
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
        if event.button() == QtCore.Qt.LeftButton:
            self.offset = event.pos()

    def mouseMoveEvent(self, event):
        if self.offset is not None:
            self.parent.move(event.globalPos() - self.offset)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.offset = None


# ----------------------------------------------------------------------
# Application Principale
# ----------------------------------------------------------------------

class IKApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.resize(1300, 800)
        self.setStyleSheet(STYLESHEET + f" #Main {{ border: 1px solid {BORDER}; }}")

        # Géométrie physique réelle du bras
        # d5 = 50mm = distance entre J4 (centre poignet) et J5 (pointe outil)
        self.geom = {'d1': 0.20, 'a2': 0.20, 'a3': 0.20, 'd5': 0.05}

        # Position initiale : bras tendu DROIT VERS LE HAUT, outil pointant vers le ciel (pitch=90°)
        # Centre poignet à z = d1+a2+a3 = 0.60m → Pointe outil (J5) à z = 0.60 + d5 = 0.65m
        # → donne J2=0°, J3=0°, J4=0° (position home parfaite)
        self.target = {'x': 0.0, 'y': 0.0, 'z': 0.65, 'pitch': 90.0, 'roll': 0.0}
        self.elbow_down = True

        # Angles théoriques pour le homing vertical (J2=90°, reste=0°)
        self.last_valid_thetas = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Limites angulaires physiques mesurées sur le bras réel
        self.joint_limits = [
            (-1700.0, 1750.0),   # J1 base         — rotation libre (câbles)
            ( -85.0,  85.0),   # J2 épaule        — mesuré irl
            ( -100.0,  100.0),   # J3 coude         — mesuré irl
            ( -90.0,  90.0),   # J4 poignet pitch — mesuré irl
            (-180.0, 180.0),   # J5 roll          — rotation libre
        ]

        main_w = QtWidgets.QWidget()
        main_w.setObjectName("Main")
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

        # ------------------------------------------------------------------
        # Communication Série
        # ------------------------------------------------------------------
        try:
            self.ser = serial.Serial('COM8', 115200, timeout=0)
            self.robot_ready = True
            print("Connecté à l'Arduino Mega. Téleopération IK active.")
        except Exception as e:
            self.ser = None
            self.robot_ready = False
            print(f"Mode simulation uniquement (Arduino non détecté : {e})")

        self.last_sent_thetas = [0.0, 0.0, 0.0, 0.0, 0.0]

        # steps_per_deg corrigés selon les vrais rapports de réduction :
        #   J1 base      1:16  → 200*16/360 = 142.222 pas/°
        #   J2 épaule    1:64  → 200*64/360 = 568.888 pas/°
        #   J3 coude     1:64  → 200*64/360 = 568.888 pas/°
        #   J4 poignet   1:16  → 200*16/360 = 142.222 pas/°
        #   J5 roll      1:16  → 200*16/360 = 142.222 pas/°  (ajuster si différent)
        # Résolutions : 1/16 microstepping confirmé empiriquement
        # J1/J4/J5 (1:16) : 200×16×16/360 = 142.222 pas/°
        # J2/J3    (1:64) : 200×16×64/360 = 568.888 pas/°
        self.steps_per_deg = [142.222, 568.888, 568.888, 142.222, 142.222]

        # Correction de sens : mettre -1 pour inverser un joint
        self.dir_signs = [1, 1, -1, -1, 1]

        # Décalage Homing : zéro physique Arduino vs. zéro mathématique
        self.home_angles_deg = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.com_timer = QtCore.QTimer()
        self.com_timer.timeout.connect(self._serial_loop)
        self.com_timer.start(20)   # 50 Hz

    # ------------------------------------------------------------------
    # Communication Série
    # ------------------------------------------------------------------
    def _serial_loop(self):
        if not self.ser:
            return
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('ascii').strip()
                if "ACK:IDLE" in line:
                    self.robot_ready = True
            except Exception:
                pass

        if self.robot_ready and self.last_valid_thetas != self.last_sent_thetas:
            steps = [
                int((self.last_valid_thetas[i] - self.home_angles_deg[i]) * self.steps_per_deg[i] * self.dir_signs[i])
                for i in range(5)
            ]
            frame = f"<T,{steps[0]},{steps[1]},{steps[2]},{steps[3]},{steps[4]}>\n"
            self.ser.write(frame.encode('ascii'))
            self.last_sent_thetas = list(self.last_valid_thetas)
            self.robot_ready = False

    # ------------------------------------------------------------------
    # Builders UI & Rendu
    # ------------------------------------------------------------------
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

        # Marqueur de la cible (pointe outil J5)
        self.target_marker = gl.GLScatterPlotItem(size=20)
        self.gl_view.addItem(self.target_marker)

    def _build_panel(self):
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(450)
        v = QtWidgets.QVBoxLayout(panel)
        v.setSpacing(10)

        # --- Consigne de l'effecteur ---
        gb_target = QtWidgets.QGroupBox("Consigne de l'Effecteur — Pointe Outil J5 (IK Target)")
        vt = QtWidgets.QVBoxLayout(gb_target)

        self.s_x = TargetSlider("X (Avant)",   -0.6,   0.6,  self.target['x'])
        self.s_y = TargetSlider("Y (Gauche)",  -0.6,   0.6,  self.target['y'])
        self.s_z = TargetSlider("Z (Haut)",     0.0,   0.70, self.target['z'])
        self.s_p = TargetSlider("Pitch Outil", -180.0, 180.0, self.target['pitch'], True)
        self.s_r = TargetSlider("Roll Outil",  -180.0, 180.0, self.target['roll'],  True)

        for s, k in [(self.s_x, 'x'), (self.s_y, 'y'), (self.s_z, 'z'),
                     (self.s_p, 'pitch'), (self.s_r, 'roll')]:
            s.valueChanged.connect(self._make_tgt_cb(k))
            vt.addWidget(s)

        self.cb_posture = QtWidgets.QCheckBox("Posture : Coude vers le bas (Elbow Down)")
        self.cb_posture.setChecked(self.elbow_down)
        self.cb_posture.stateChanged.connect(self._on_posture_changed)
        vt.addWidget(self.cb_posture)
        v.addWidget(gb_target)

        # --- Géométrie ---
        gb_geom = QtWidgets.QGroupBox("Géométrie du Bras (m)")
        vg = QtWidgets.QHBoxLayout(gb_geom)
        params = [("Base (d1)", "d1"), ("Bras (a2)", "a2"),
                  ("Av.Bras (a3)", "a3"), ("Outil (d5)", "d5")]
        for lbl, key in params:
            lv = QtWidgets.QVBoxLayout()
            lv.addWidget(QtWidgets.QLabel(lbl))
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(0.001, 1.0)
            box.setSingleStep(0.01)
            box.setDecimals(3)
            box.setValue(self.geom[key])
            box.valueChanged.connect(self._make_geom_cb(key))
            lv.addWidget(box)
            vg.addLayout(lv)
        v.addWidget(gb_geom)

        # --- Résultat IK ---
        gb_out = QtWidgets.QGroupBox("Résultat Cinématique Inverse")
        vo = QtWidgets.QVBoxLayout(gb_out)
        self.readout = QtWidgets.QTextEdit()
        self.readout.setReadOnly(True)
        vo.addWidget(self.readout)
        v.addWidget(gb_out)

        btn_reset = QtWidgets.QPushButton("Reset Target (Homing Vertical)")
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
        # Bras tendu DROIT VERS LE HAUT : J2=0°, J3=0°, J4=0°
        self.s_x.set_value(0.0)
        self.s_y.set_value(0.0)
        self.s_z.set_value(0.65)
        self.s_p.set_value(90.0)
        self.s_r.set_value(0.0)

    def update_ik(self):
        """Calcule l'IK (cible = J5), met a jour le texte et le modele 3D."""
        tx, ty, tz = self.target['x'], self.target['y'], self.target['z']
        self.target_marker.setData(pos=np.array([[tx, ty, tz]]))

        success, result = compute_ik_5dof(
            tx, ty, tz,
            self.target['pitch'], self.target['roll'],
            self.geom['d1'], self.geom['a2'], self.geom['a3'], self.geom['d5'],
            self.elbow_down
        )

        if success:
            # Vérification des limites articulaires
            limit_err = None
            for i, (lo, hi) in enumerate(self.joint_limits):
                if not (lo <= result[i] <= hi):
                    limit_err = f"Limite J{i+1}: {result[i]:.1f}° hors de [{lo:.0f}°, {hi:.0f}°]"
                    break

            if limit_err:
                success = False
                result = limit_err
            else:
                self.last_valid_thetas = result
                self.target_marker.setData(color=TARGET_OK_COLOR)
                txt = "<span style='color:#4caf6f; font-weight:bold;'>[SUCCÈS] Cible atteinte.</span><br><br>"
                txt += "<b>Angles générés pour les moteurs :</b><br>"
                for i, th in enumerate(result):
                    lo, hi = self.joint_limits[i]
                    txt += f"&nbsp;&nbsp;Joint {i+1} : <span style='color:#4fa3ff'>{th:>+7.2f}°</span> <span style='color:#555'>[{lo:.0f}°, {hi:.0f}°]</span><br>"
                self.readout.setHtml(txt)
        if not success:
            self.target_marker.setData(color=TARGET_ERR_COLOR)
            txt = f"<span style='color:#e81123; font-weight:bold;'>[ERREUR] {result}</span><br><br>"
            txt += "<span style='color:#8a8f99'>Le bras reste sur sa dernière position valide.</span>"
            self.readout.setHtml(txt)

        # Rendu 3D — FK avec les thetas valides (d5 pour le dernier segment J4→J5)
        dh = [
            {'alpha_deg': 90.0, 'a': 0.0,              'd': self.geom['d1'], 'offset_deg': 0.0},
            {'alpha_deg':  0.0, 'a': self.geom['a2'],  'd': 0.0,             'offset_deg': 90.0},
            {'alpha_deg':  0.0, 'a': self.geom['a3'],  'd': 0.0,             'offset_deg': 0.0},
            {'alpha_deg': 90.0, 'a': 0.0,              'd': 0.0,             'offset_deg': 90.0},
            {'alpha_deg':  0.0, 'a': 0.0,              'd': self.geom['d5'], 'offset_deg': 0.0},
        ]

        frames = forward_kinematics(dh, self.last_valid_thetas)
        arm_origins = np.array([T[:3, 3] for T in frames])
        self.link_item.setData(pos=arm_origins)
        self.joint_markers.setData(pos=arm_origins)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = IKApp()
    win.show()
    sys.exit(app.exec_())