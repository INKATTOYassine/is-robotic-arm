# ROS 2 Digital Twin & Hardware Integration

This directory contains the ROS 2 (Jazzy) packages for the 5-DOF robotic arm. It serves as the bridge between the high-level trajectory planning and the low-level hardware control.

## Architecture Roadmap

### Phase 1: PC-Based Development (Current)
All computational nodes run on a centralized Ubuntu 24.04 PC.
* **Simulation:** Gazebo handles the physical simulation (gravity, inertia) of the URDF model.
* **Planning & Visualization:** MoveIt 2 computes collision-free trajectories, visualized in RViz 2.
* **Hardware Interface:** A custom `ros2_control` C++ plugin translates ROS joint commands into asynchronous serial packets (`<T,...>`) sent directly from the PC via USB to the Arduino Mega.

### Phase 2: Distributed Edge Computing (Future)
The system will be deployed using a distributed ROS 2 architecture over a local network (DDS).
* **Edge Controller (Raspberry Pi 4 - 4GB Model B):** Mounted on/near the robot. It will run the `ros2_control` node and the hardware interface, communicating locally via USB/UART with the Arduino Mega.
* **Master Node / UI (PC):** The heavy computational load (Gazebo simulation, MoveIt 2 IK/path planning, RViz 2 rendering) remains on the PC. It will stream trajectory execution commands to the Raspberry Pi over Wi-Fi/Ethernet.
* **Real-time Hardware (Arduino Mega):** Remains dedicated to non-blocking stepper pulse generation (Bresenham) and trapezoidal velocity profiling (David Austin).

## Implementation Steps
1. **Xacro Modularization:** Convert the static CAD-exported URDF into dynamic Xacro macros.
2. **`ros2_control` Integration:** Define the joint hardware interfaces (position control).
3. **Hardware Plugin:** Develop the C++ serial bridge for the Arduino Mega.
4. **MoveIt 2 Configuration:** Generate the semantic robot description (SRDF) and configure OMPL planners.
5. **Raspberry Pi Deployment:** Containerize or deploy the hardware nodes to the RPi 4.