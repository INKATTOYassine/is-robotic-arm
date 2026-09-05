# IK Simulator

Interactive analytical inverse-kinematics debugger built with PyQt5 + pyqtgraph (OpenGL). Given a target end-effector pose, it computes the joint angles and renders the resulting arm configuration live, alongside a marker for the requested target.

## Run

```
pip install PyQt5 pyqtgraph PyOpenGL
python3 ik_solver_app.py
```

## Method

Analytical solution via kinematic decoupling (position solved geometrically, orientation decoupled afterward) — no numerical iteration, so no risk of solver divergence or singularity-related instability:

1. **Base (J1):** `theta_1 = atan2(y, x)`
2. **Reduction to the 2D vertical plane** containing the shoulder
3. **Elbow (J3):** law of cosines on the arm triangle; two valid solutions (elbow up / elbow down), selectable via the posture checkbox
4. **Shoulder (J2):** absolute elevation angle combined with the arm's internal angle
5. **Wrist (J4, J5):** orientation decoupling — `theta_4 = pitch - theta_2 - theta_3`, `theta_5 = roll`

Reachability is checked before solving (`|C_3| <= 1`); out-of-range targets are reported instead of silently producing an invalid pose.

## Verification

Numerically verified: for multiple target poses and both postures (elbow up / elbow down), feeding the solver's output angles back through forward kinematics reproduces the requested (x, y, z) exactly (0.000000 m error). See [`../fk_simulator/`](../fk_simulator/README.md) for the FK model this is checked against.

## Geometry parameters

`d1`, `a2`, `a3` (base height, upper-arm length, forearm length) are adjustable in the UI — keep them in sync with the real CAD dimensions once finalized.
