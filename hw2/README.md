# NYCU Computer Animation 2024 Spring HW2 — Forward & Inverse Kinematics

StudentID: 110612117  \
Name: Chung-Yu Chang (張仲瑜)

## Introduction
This assignment builds a full FK/IK pipeline for articulated skeletons. Forward kinematics composes parent-to-child transforms and ZYX Euler rotations to place every joint in world space. Inverse kinematics then steers end-effectors toward user targets via Jacobian-based updates: we assemble per-joint angular Jacobian columns, solve a damped least-squares step with the Moore–Penrose pseudo-inverse, and clamp rotations to joint limits for stability. The solver iterates until the positional error falls below a threshold, updating chains while preserving base translations for multi-chain reach tasks.

## What I implemented
- Forward kinematics (FK) traversal that propagates start/end positions and composite rotations down the hierarchy in [HW2/src/simulation/kinematics.cpp](HW2/src/simulation/kinematics.cpp).
- Jacobian pseudo-inverse IK (SVD) with per-DOF column assembly, target-space step, and joint-limit clamping in [HW2/src/simulation/kinematics.cpp](HW2/src/simulation/kinematics.cpp).
- Root handling and multi-chain updates so end-effectors converge without breaking the base transform.

## How to build and run
Follow the platform-specific instructions in the nested [HW2/README.md](HW2/README.md). That document covers solution/CMake builds and dependencies.

## Notes and observations
- Using ZYX Euler composition matches the AMC/ASF conventions and keeps joint axes intuitive for editing.
- SVD-based pseudo-inverse is robust for near-singular poses; joint-limit clamping prevents oscillation when targets are out of reach.
- Multi-chain solving benefits from small step sizes and error thresholds to avoid overshoot on tightly coupled limbs.
- See [HW2/report_110612117.pdf](HW2/report_110612117.pdf) for detailed analysis and screenshots.
