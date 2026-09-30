# Python Kinematics Solvers & Teleoperation

This directory contains PyQt5/pyqtgraph-based interactive debugging tools for the 5-DOF robotic arm. These scripts allow for real-time visualization, mathematical validation, and direct serial teleoperation of the physical robot.

## Files Description

### 1. `inverse-kinematics-teleop.py`
An Inverse Kinematics (IK) interactive solver and teleoperation node.
* **Target Control:** Solves the 5-DOF analytical IK for a spatial target (X, Y, Z) and tool orientation (Pitch, Roll) targeting the end-effector (J5) with a defined tool offset (`d5 = 0.05m`).
* **Hardware Sync:** Translates theoretical angles into hardware stepper steps using precise physical reduction ratios (e.g., 1:16 and 1:64) and homing offsets.
* **Safety:** Implements joint limits directly in the GUI to prevent mechanical collisions.

### 2. `forward-kinematics-teleop.py`
A Denavit-Hartenberg (DH) and Forward Kinematics (FK) visual debugger.
* **Live DH Table:** Modify DH parameters (alpha, a, d, offset) on the fly and see the 3D frame transformations instantly.
* **Geometric Validation:** Live checks for required mechanical constraints (e.g., ensuring J2/J3 are parallel, and J5 is perpendicular to J4).
* **Teleoperation:** Asynchronous serial communication to drive the Arduino Mega controller in real-time without UI blocking.

## Dependencies
To run these tools, install the required Python packages:

```bash
pip install PyQt5 pyqtgraph pyserial numpy