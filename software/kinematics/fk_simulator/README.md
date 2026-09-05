# FK Simulator

Interactive DH/FK debugger built with PyQt5 + pyqtgraph (OpenGL). Lets you move each joint angle live, edit the DH table, and visually verify the geometry.

## Run

```
pip install PyQt5 pyqtgraph PyOpenGL
python3 dh_fk_app.py
```

## Features

- Live joint-angle sliders (theta) driving the 3D view
- Editable DH table (alpha, a, d, offset per joint)
- Toggleable reference frames: per-axis (X/Y/Z) and per-joint (world, J1–J5), with adjustable axis length — useful to visually confirm each joint's rotation axis direction
- Live geometric verification panel: checks that the base is vertical, that the three pitch joints (shoulder/elbow/wrist1) are parallel, and that the roll joint (wrist2) is perpendicular to the last pitch joint
- FK readout: end-effector position/orientation (T0_5) and each joint's real-world rotation axis
