# Software

Software side of the arm: kinematics, control, firmware, and the digital twin.

## Structure

- [`kinematics/`](kinematics/README.md) — DH parameters, forward kinematics, and (soon) inverse kinematics, with an interactive simulator to verify everything visually.

## Roadmap (not yet in the repo)

- `firmware/` — Arduino Mega firmware (custom, joint-angle to step/dir conversion; testing phase uses grbl-Mega-5X)
- `control/` — Raspberry Pi control node, RPi ↔ Mega serial communication
- `digital_twin/` — ROS2 nodes publishing joint states, RViz/Gazebo visualization
