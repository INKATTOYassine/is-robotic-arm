# Software

Software side of the arm: kinematics, control, firmware, and the digital twin.

## Structure

*   `kinematics/` — Mathematical models for Forward (FK) and Inverse (IK) kinematics, utilizing Denavit-Hartenberg parameters. Includes custom PyQt5/OpenGL interactive simulators to verify coordinate frames and joint accessibility visually.
    *   `/fk_simulator` : Real-time DH parameter and forward kinematics debugger.
    *   `/ik_simulator` : Target-based analytical inverse kinematics solver with posture control (Elbow Up/Down) and reachability checks.
    *   `/kinematics-teleop` : Python tools combining FK/IK solvers with asynchronous serial teleoperation to drive the Arduino Mega.
*   `firmware/` — Custom C++ firmware for the Arduino Mega + RAMPS stack. It features a non-blocking, interrupt-driven (Timer1) architecture. Includes a custom low-level motion planner using the Bresenham algorithm for multi-axis spatial synchronization and the David Austin algorithm for trapezoidal velocity profiles (acceleration/deceleration).
*   `control/` — Python control nodes acting as the bridge between the PC (or Raspberry Pi) and the Arduino Mega. Implements a strict, non-blocking asynchronous serial communication protocol (`<T,j1,j2,j3,j4,j5>`) to ensure EMI-resistant data transmission.

## Roadmap (not yet in the repo)

*   `digital_twin/` — ROS 2 packages containing the URDF model of the robotic arm, nodes for publishing joint states, and integration with RViz and Gazebo for advanced 3D simulation and trajectory planning (via MoveIt 2).