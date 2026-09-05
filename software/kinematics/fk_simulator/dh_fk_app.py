"""
is-robotic-arm - DH / FK interactive debugger
------------------------------------------------
Application PyQt5 + pyqtgraph (rendu OpenGL) pour visualiser et debugger
en temps reel le modele DH d'un bras robotique 5DOF.

Convention utilisee : DH standard, A_i = Rz(theta_i+offset_i)·Tz(d_i)·Tx(a_i)·Rx(alpha_i)
Repere monde : X=avant, Y=gauche, Z=haut (convention ROS REP-103)
"""

import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph.opengl as gl


# ----------------------------------------------------------------------
# Cinematique
# ----------------------------------------------------------------------

def dh_matrix(alpha, a, d, theta):
    """Construit la matrice de transformation homogene 4x4 d'un seul joint,
    a partir de ses 4 parametres DH (convention standard)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])


def forward_kinematics(dh_rows, thetas_deg):
    """Calcule le FK complet : construit chaque matrice A_i puis les enchaine
    (T0->T1->...->T5) par multiplication. theta_i utilise = angle physique
    du slider + l'offset constant de la ligne DH correspondante.
    Retourne la liste des 6 reperes cumules [T0, T1, T2, T3, T4, T5]
    (T0 = origine du robot, T5 = effecteur)."""
    T = np.eye(4)
    frames = [T.copy()]
    for row, th_deg in zip(dh_rows, thetas_deg):
        A = dh_matrix(np.radians(row['alpha_deg']), row['a'], row['d'],
                      np.radians(th_deg + row['offset_deg']))
        T = T @ A
        frames.append(T.copy())
    return frames


def joint_axis_world(frames, joint_index_1based):
    """Direction reelle, dans le repere monde, de l'axe de rotation du
    joint i (1..5). En DH standard, le joint i tourne autour de z_(i-1),
    qui est la 3e colonne de la matrice de rotation du repere i-1
    -> d'ou l'indexation frames[i-1]."""
    return frames[joint_index_1based - 1][:3, 2]


def rotation_to_rpy_deg(R):
    """Extrait roll/pitch/yaw (convention ZYX) en degres depuis une matrice
    de rotation 3x3, pour affichage lisible dans le panneau de lecture."""
    pitch = np.degrees(np.arcsin(-np.clip(R[2, 0], -1.0, 1.0)))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


JOINT_NAMES = ["J1 base (yaw)", "J2 epaule (pitch)", "J3 coude (pitch)",
               "J4 poignet1 (pitch)", "J5 poignet2 (roll)"]

# Table DH de depart (editable en live dans l'UI). L'offset de +90 sur J4
# est necessaire pour que l'axe du roll (J5) soit coaxial au bras a la
# config home (verifie et documente plus tot dans le projet).
DEFAULT_DH = [
    {'alpha_deg': 90.0, 'a': 0.0,  'd': 0.25, 'offset_deg': 0.0},
    {'alpha_deg': 0.0,  'a': 0.30, 'd': 0.0,  'offset_deg': 0.0},
    {'alpha_deg': 0.0,  'a': 0.25, 'd': 0.0,  'offset_deg': 0.0},
    {'alpha_deg': 90.0, 'a': 0.0,  'd': 0.0,  'offset_deg': 90.0},
    {'alpha_deg': 0.0,  'a': 0.0,  'd': 0.0,  'offset_deg': 0.0},
]

GEOMETRIC_CHECKS = [
    ("J1 vertical (yaw)", lambda ax: [abs(ax[1][0]), abs(ax[1][1]), abs(ax[1][2] - 1)]),
]


# ----------------------------------------------------------------------
# Palette theme sombre
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
QLabel[role="header"] {{ color: {ACCENT}; font-weight: 700; font-size: 13px; }}
QLabel[role="ok"] {{ color: {OK_COLOR}; font-weight: 600; }}
QLabel[role="warn"] {{ color: {WARN_COLOR}; font-weight: 600; }}
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
    """Un slider theta (degres, -180 a 180) avec label de nom et affichage
    de la valeur courante. Emet valueChanged(deg) a chaque mouvement."""
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, name, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        self.label = QtWidgets.QLabel(name)
        self.label.setFixedWidth(130)
        # Slider en dixiemes de degre (int) pour une precision de 0.1°
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(-1800)
        self.slider.setMaximum(1800)
        self.slider.setValue(0)
        self.value_label = QtWidgets.QLabel("0.0°")
        self.value_label.setProperty("role", "value")
        self.value_label.setFixedWidth(48)
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)
        self.slider.valueChanged.connect(self._on_change)

    def _on_change(self, v):
        deg = v / 10.0
        self.value_label.setText(f"{deg:.1f}°")
        self.valueChanged.emit(deg)

    def set_value(self, deg):
        """Deplace le slider par programme (ex: bouton Reset) ;
        declenche automatiquement _on_change() -> valueChanged."""
        self.slider.setValue(int(deg * 10))


class DHRow(QtWidgets.QWidget):
    """Une ligne editable de la table DH : 4 champs numeriques
    (alpha, a, d, offset) pour un joint. Modifie directement le
    dictionnaire `row` passe en reference et emet changed() a chaque edit."""
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
        """Fabrique un callback qui met a jour la bonne cle du dict `row`
        (une closure par champ, necessaire car chaque QDoubleSpinBox
        doit savoir quelle cle il represente)."""
        def cb(val):
            self.row[key] = val
            self.changed.emit()
        return cb


class AxisLegend(QtWidgets.QWidget):
    """Legende couleur (X/Y/Z/liens) affichee sous la vue 3D."""
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
    """Panneau de validation live de la table DH : verifie des proprietes
    geometriques structurelles (parallelisme/perpendicularite entre axes
    de joints consecutifs) et colore chaque ligne en vert/rouge.

    Important : ces 4 checks sont des INVARIANTS mathematiques de la table
    (ils ne dependent que des alpha_i, pas des valeurs actuelles de theta)
    -> ils restent valides quelle que soit la position des sliders, et se
    mettent a jour immediatement si on modifie un alpha dans la table DH."""
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
        """Recalcule les 4 verifications a partir des reperes FK actuels
        et met a jour la couleur (vert=OK, orange=incoherent) de chaque
        pastille."""
        def axis(i):
            return joint_axis_world(frames, i)

        def is_parallel(u, v, tol=1e-2):
            return abs(abs(np.dot(u, v)) - 1.0) < tol

        def is_perp(u, v, tol=1e-2):
            return abs(np.dot(u, v)) < tol

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
    """Barre de titre custom (remplace la barre native de l'OS, retiree
    via setWindowFlags(FramelessWindowHint) dans DHFKApp) : titre,
    boutons reduire/agrandir/fermer, et deplacement de la fenetre au
    clic-glisse puisque l'OS ne gere plus ca lui-meme."""
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.setFixedHeight(35)

        self.setStyleSheet(f"""
            QWidget {{
                background-color: #14161b;
                border-bottom: 1px solid {BORDER};
            }}
            QLabel {{
                color: {TEXT_DIM};
                font-weight: 600;
                font-size: 13px;
                padding-left: 14px;
                border: none;
            }}
            QPushButton {{
                background: transparent;
                border: none;
                border-radius: 0px;
                font-weight: normal;
                font-size: 14px;
                color: {TEXT_DIM};
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: {PANEL};
                color: {TEXT};
            }}
            QPushButton#closeBtn:hover {{
                background-color: #e81123;
                color: white;
            }}
        """)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.title_label = QtWidgets.QLabel("is-robotic-arm — DH / FK debugger")
        layout.addWidget(self.title_label)
        layout.addStretch()

        self.btn_min = QtWidgets.QPushButton("—")
        self.btn_min.setFixedSize(45, 35)
        self.btn_min.clicked.connect(self.parent.showMinimized)
        layout.addWidget(self.btn_min)

        self.btn_max = QtWidgets.QPushButton("□")
        self.btn_max.setFixedSize(45, 35)
        self.btn_max.clicked.connect(self.toggle_max_restore)
        layout.addWidget(self.btn_max)

        self.btn_close = QtWidgets.QPushButton("✕")
        self.btn_close.setObjectName("closeBtn")
        self.btn_close.setFixedSize(45, 35)
        self.btn_close.clicked.connect(self.parent.close)
        layout.addWidget(self.btn_close)

        self.offset = None  # position du clic dans la barre, pour le drag

    def toggle_max_restore(self):
        if self.parent.isMaximized():
            self.parent.showNormal()
        else:
            self.parent.showMaximized()

    # Deplacement de la fenetre par clic-glisse (necessaire car la fenetre
    # est frameless, donc l'OS ne fournit plus cette interaction par defaut)
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.offset = event.pos()

    def mouseMoveEvent(self, event):
        if self.offset is not None and not self.parent.isMaximized():
            self.parent.move(event.globalPos() - self.offset)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.offset = None

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.toggle_max_restore()


class DHFKApp(QtWidgets.QMainWindow):
    """Fenetre principale : vue 3D OpenGL a gauche, panneau de controle
    (onglets Controles / Debug) a droite. Orchestre le calcul FK et la
    mise a jour de tous les affichages a chaque changement (slider,
    table DH, options d'affichage)."""
    def __init__(self):
        super().__init__()

        # Fenetre sans cadre natif -> UI entierement custom (voir CustomTitleBar)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.resize(1480, 860)

        # Bordure ajoutee manuellement autour de toute l'appli pour qu'elle
        # ne se fonde pas dans le fond de l'ecran (consequence du frameless)
        modified_stylesheet = STYLESHEET + f"""
            #MainContainer {{
                border: 1px solid {BORDER};
                background-color: {BG};
            }}
        """
        self.setStyleSheet(modified_stylesheet)

        self.dh = [row.copy() for row in DEFAULT_DH]
        self.thetas_deg = [0.0] * 5
        self.axis_len = 0.15  # longueur d'affichage des triades d'axes (m)

        main_container = QtWidgets.QWidget()
        main_container.setObjectName("MainContainer")
        self.setCentralWidget(main_container)

        global_layout = QtWidgets.QVBoxLayout(main_container)
        global_layout.setContentsMargins(0, 0, 0, 0)
        global_layout.setSpacing(0)

        self.title_bar = CustomTitleBar(self)
        global_layout.addWidget(self.title_bar)

        # Contenu principal (vue 3D + panneau lateral), sous la barre de titre
        content_widget = QtWidgets.QWidget()
        content_layout = QtWidgets.QHBoxLayout(content_widget)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)
        global_layout.addWidget(content_widget, stretch=1)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(6)
        left.addWidget(self._build_3d_view(), stretch=1)
        self.legend = AxisLegend()
        left.addWidget(self.legend)
        content_layout.addLayout(left, stretch=3)

        content_layout.addWidget(self._build_side_panel(), stretch=2)

        self._init_gl_items()
        self._rescale_view()
        self.update_scene()

    # ------------------------------------------------------------------
    def _build_3d_view(self):
        """Cree la vue OpenGL (GLViewWidget) avec sa grille de sol."""
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
        """Cree une seule fois les objets graphiques 3D (lien du bras,
        marqueurs de joints, axes x/y/z) ; les updates suivants ne font
        que reecrire leurs donnees (setData), jamais les recreer
        -> essentiel pour la fluidite et pour ne pas reinitialiser la
        camera a chaque frame."""
        self.link_item = gl.GLLinePlotItem(color=LINK_COLOR, width=5, antialias=True)
        self.gl_view.addItem(self.link_item)

        self.joint_markers = gl.GLScatterPlotItem(color=JOINT_COLOR, size=14)
        self.gl_view.addItem(self.joint_markers)

        self.axis_x = gl.GLLinePlotItem(color=AXIS_X, width=2, mode='lines')
        self.axis_y = gl.GLLinePlotItem(color=AXIS_Y, width=2, mode='lines')
        self.axis_z = gl.GLLinePlotItem(color=AXIS_Z, width=2, mode='lines')
        for it in (self.axis_x, self.axis_y, self.axis_z):
            self.gl_view.addItem(it)

    def _rescale_view(self):
        """Ajuste la distance de la camera a l'envergure totale du bras
        (somme des a_i et d_i). Appele uniquement quand la table DH
        change (pas a chaque mouvement de theta)."""
        reach = sum(r['a'] for r in self.dh) + sum(r['d'] for r in self.dh) + 0.05
        self.gl_view.setCameraPosition(distance=max(reach, 0.3) * 2.6)

    def _build_side_panel(self):
        """Panneau lateral droit : onglets Controles/Debug + bouton reset."""
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(500)
        v = QtWidgets.QVBoxLayout(panel)
        v.setSpacing(8)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_controls_tab(), "Contrôles")
        tabs.addTab(self._build_debug_tab(), "Debug / Vérification")
        v.addWidget(tabs, stretch=1)

        reset_btn = QtWidgets.QPushButton("Reset angles (home)")
        reset_btn.clicked.connect(self._on_reset)
        v.addWidget(reset_btn)

        return panel

    def _build_controls_tab(self):
        """Onglet Controles : sliders theta, table DH editable, options
        d'affichage des reperes."""
        tab = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(tab)
        v.setSpacing(10)

        gb_theta = QtWidgets.QGroupBox("Angles articulaires (theta)")
        gb_theta_l = QtWidgets.QVBoxLayout(gb_theta)
        self.theta_sliders = []
        for i, name in enumerate(JOINT_NAMES):
            s = ThetaSlider(name)
            s.valueChanged.connect(self._make_theta_cb(i))
            gb_theta_l.addWidget(s)
            self.theta_sliders.append(s)
        v.addWidget(gb_theta)

        gb_dh = QtWidgets.QGroupBox("Table DH (alpha°, a[m], d[m], offset°)")
        gb_dh_l = QtWidgets.QVBoxLayout(gb_dh)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(self._dim_label("", 130))
        for h in ["alpha°", "a[m]", "d[m]", "offset°"]:
            header.addWidget(self._dim_label(h, 62))
        gb_dh_l.addLayout(header)

        self.dh_rows_widgets = []
        for i, name in enumerate(JOINT_NAMES):
            row_w = DHRow(name, self.dh[i])
            row_w.changed.connect(self._on_dh_changed)
            gb_dh_l.addWidget(row_w)
            self.dh_rows_widgets.append(row_w)
        v.addWidget(gb_dh)

        v.addWidget(self._build_frame_display_group())

        v.addStretch()
        return tab

    def _build_frame_display_group(self):
        """Groupe d'options de visualisation : taille des triades d'axes,
        quels axes (x/y/z) afficher, et quels reperes (parmi les 6
        disponibles) afficher individuellement."""
        gb = QtWidgets.QGroupBox("Repères affichés (Visualisation)")
        v = QtWidgets.QVBoxLayout(gb)

        len_row = QtWidgets.QHBoxLayout()
        len_row.addWidget(self._dim_label("Taille des axes", 110))
        self.axis_len_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.axis_len_slider.setMinimum(2)
        self.axis_len_slider.setMaximum(50)
        self.axis_len_slider.setValue(int(self.axis_len * 100))
        self.axis_len_value_lbl = QtWidgets.QLabel(f"{self.axis_len:.2f} m")
        self.axis_len_value_lbl.setProperty("role", "value")
        self.axis_len_value_lbl.setFixedWidth(48)
        self.axis_len_slider.valueChanged.connect(self._on_axis_len_changed)
        len_row.addWidget(self.axis_len_slider)
        len_row.addWidget(self.axis_len_value_lbl)
        v.addLayout(len_row)

        axis_row = QtWidgets.QHBoxLayout()
        axis_row.addWidget(self._dim_label("Axes :", 60))
        self.axis_checks = {}
        for key, label, color in [('x', 'X', AXIS_X), ('y', 'Y', AXIS_Y), ('z', 'Z', AXIS_Z)]:
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(True)
            rgb = tuple(int(c * 255) for c in color[:3])
            cb.setStyleSheet(f"QCheckBox {{ color: rgb{rgb}; font-weight: 600; }}")
            cb.stateChanged.connect(self._on_display_toggle)
            self.axis_checks[key] = cb
            axis_row.addWidget(cb)
        axis_row.addStretch()
        v.addLayout(axis_row)

        # 6 reperes affichables : le repere "monde" (place dans un coin pour
        # reference visuelle) + les 5 reperes physiques du bras
        frame_labels = [
            "Monde (Coin CAD)",
            "Repère 1 (Base)",
            "Repère 2 (Épaule)",
            "Repère 3 (Coude)",
            "Repère 4 (Poignet Pitch)",
            "Repère 5 (Poignet Roll)"
        ]

        self.frame_checks = []
        grid = QtWidgets.QGridLayout()
        for idx, lbl in enumerate(frame_labels):
            cb = QtWidgets.QCheckBox(lbl)
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_display_toggle)
            self.frame_checks.append(cb)
            grid.addWidget(cb, idx // 2, idx % 2)
        v.addLayout(grid)

        btn_row = QtWidgets.QHBoxLayout()
        all_btn = QtWidgets.QPushButton("Tout afficher")
        none_btn = QtWidgets.QPushButton("Tout cacher")
        all_btn.clicked.connect(lambda: self._set_all_frame_checks(True))
        none_btn.clicked.connect(lambda: self._set_all_frame_checks(False))
        btn_row.addWidget(all_btn)
        btn_row.addWidget(none_btn)
        v.addLayout(btn_row)

        return gb

    def _set_all_frame_checks(self, state):
        """Coche/decoche tous les reperes d'un coup (boutons Tout afficher/cacher)."""
        for cb in self.frame_checks:
            cb.blockSignals(True)  # evite de redessiner 6 fois de suite
            cb.setChecked(state)
            cb.blockSignals(False)
        self.update_scene()

    def _on_axis_len_changed(self, v):
        self.axis_len = v / 100.0
        self.axis_len_value_lbl.setText(f"{self.axis_len:.2f} m")
        self.update_scene()

    def _on_display_toggle(self):
        self.update_scene()

    def _build_debug_tab(self):
        """Onglet Debug : panneau de validation geometrique live +
        lecture texte complete du FK (position, orientation, axes)."""
        tab = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(tab)
        v.setSpacing(10)

        gb_check = QtWidgets.QGroupBox("Vérification géométrique (live)")
        gb_check_l = QtWidgets.QVBoxLayout(gb_check)
        self.geo_panel = GeometryCheckPanel()
        gb_check_l.addWidget(self.geo_panel)
        v.addWidget(gb_check)

        gb_read = QtWidgets.QGroupBox("Lecture FK")
        gb_read_l = QtWidgets.QVBoxLayout(gb_read)
        self.readout = QtWidgets.QTextEdit()
        self.readout.setReadOnly(True)
        self.readout.setMinimumHeight(260)
        gb_read_l.addWidget(self.readout)
        v.addWidget(gb_read)

        v.addStretch()
        return tab

    def _dim_label(self, text, width):
        lbl = QtWidgets.QLabel(text)
        lbl.setFixedWidth(width)
        return lbl

    def _make_theta_cb(self, idx):
        def cb(deg):
            self.thetas_deg[idx] = deg
            self.update_scene()
        return cb

    def _on_dh_changed(self):
        self._rescale_view()
        self.update_scene()

    def _on_reset(self):
        for s in self.theta_sliders:
            s.set_value(0.0)

    # ------------------------------------------------------------------
    def update_scene(self):
        """Point d'entree central : recalcule le FK et met a jour tous
        les affichages (3D, lecture texte, panneau de validation).
        Appele a chaque changement de slider, de table DH, ou d'option
        d'affichage."""
        # 1. FK pur : les 6 reperes cumules T0..T5
        dh_frames = forward_kinematics(self.dh, self.thetas_deg)

        # 2. Repere "monde" purement visuel, decale dans un coin pour
        # servir de reference fixe independamment de la pose du bras
        T_world = np.eye(4)
        T_world[0, 3] = -0.4
        T_world[1, 3] = -0.4

        # 3. Liste des reperes a afficher en 3D. On saute dh_frames[4] (T4,
        # apres le pitch du poignet mais avant le roll) car son origine est
        # identique a celle de T3 et T5 (a4=a5=d4=d5=0, axes concourants) ;
        # l'afficher en plus n'apporterait qu'une triade superposee.
        display_frames = [
            T_world,
            dh_frames[0],  # R1 (Base)
            dh_frames[1],  # R2 (Épaule)
            dh_frames[2],  # R3 (Coude)
            dh_frames[3],  # R4 (Poignet Pitch)
            dh_frames[5]   # R5 (Poignet Roll)
        ]

        self._update_3d(display_frames)

        # Les calculs (lecture + validation) utilisent la chaine DH complete,
        # pas la liste filtree ci-dessus qui ne sert qu'a l'affichage
        self._update_readout(dh_frames)
        self.geo_panel.update_checks(dh_frames)

    def _update_3d(self, display_frames):
        """Redessine le bras (liens + joints) et les triades d'axes
        selon les reperes/axes actuellement coches dans l'UI."""
        # display_frames[0] = repere monde (coin) -> exclu du trace des liens
        arm_origins = np.array([T[:3, 3] for T in display_frames[1:]])
        self.link_item.setData(pos=arm_origins)
        self.joint_markers.setData(pos=arm_origins)

        show_x = self.axis_checks['x'].isChecked()
        show_y = self.axis_checks['y'].isChecked()
        show_z = self.axis_checks['z'].isChecked()

        xs, ys, zs = [], [], []
        for idx, T in enumerate(display_frames):
            if not self.frame_checks[idx].isChecked():
                continue
            o = T[:3, 3]
            R = T[:3, :3]
            if show_x:
                xs += [o, o + R[:, 0] * self.axis_len]
            if show_y:
                ys += [o, o + R[:, 1] * self.axis_len]
            if show_z:
                zs += [o, o + R[:, 2] * self.axis_len]

        self.axis_x.setData(pos=np.array(xs) if xs else np.zeros((0, 3)))
        self.axis_y.setData(pos=np.array(ys) if ys else np.zeros((0, 3)))
        self.axis_z.setData(pos=np.array(zs) if zs else np.zeros((0, 3)))

    def _update_readout(self, frames):
        """Remplit le panneau texte : position/orientation de l'effecteur
        (T0_5) et direction reelle de l'axe de chaque joint."""
        T05 = frames[-1]
        pos = T05[:3, 3]
        roll, pitch, yaw = rotation_to_rpy_deg(T05[:3, :3])

        lines = ["EFFECTEUR (Par rapport au Repère 1 Base)",
                 f"  pos  x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}  [m]",
                 f"  rpy  r={roll:+7.2f}  p={pitch:+7.2f}  y={yaw:+7.2f}  [deg]",
                 "",
                 "AXES DE ROTATION REELS DES JOINTS (Dans le repère de la Base)"]
        for i in range(1, 6):
            z_i = joint_axis_world(frames, i)
            lines.append(f"  J{i}  [{z_i[0]:+.2f}, {z_i[1]:+.2f}, {z_i[2]:+.2f}]  ({JOINT_NAMES[i-1]})")

        self.readout.setPlainText("\n".join(lines))


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = DHFKApp()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()