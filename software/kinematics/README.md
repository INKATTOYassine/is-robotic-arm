# Kinematics

Mathematical model of the arm's motion.

## Structure

- [`fk_simulator/`](fk_simulator/README.md) — Interactive DH/FK debugger and visualizer (working)

## Status

- **Forward kinematics (FK):** done, derived from first principles (standard DH convention) and verified against the CAD design.
- **Inverse kinematics (IK):** analytical solution derived (kinematic decoupling, `atan2`-based) — not yet implemented in code.

## DH parameters

| Joint | Name | $\alpha$ | $a$ | $d$ |
|---|---|---|---|---|
| 1 | Base (yaw) | 0 | 0 | $d_1$ |
| 2 | Shoulder (pitch) | $\pi/2$ | 0 | 0 |
| 3 | Elbow (pitch) | 0 | $a_2$ | 0 |
| 4 | Wrist 1 (pitch) | 0 | $a_3$ | 0 |
| 5 | Wrist 2 (roll) | $-\pi/2$ | 0 | 0 |

Link lengths are kept symbolic until the CAD is finalized.
