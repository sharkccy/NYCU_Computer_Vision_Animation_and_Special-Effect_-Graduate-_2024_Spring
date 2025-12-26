# NYCU Computer Animation 2024 Spring HW1 — Cloth Simulation

StudentID: 110612117  
Name: Chung-Yu Chang (張仲瑜)

## Introduction
This homework builds a real-time mass–spring cloth simulator. The cloth mesh connects particles with structural, shear, and bending springs and applies spring–damper forces for stability. Multiple time integrators (Explicit Euler, Midpoint, Implicit Euler, Runge–Kutta 4) advance the system to compare visual quality and stability. Rendering supports particles, full shaded cloth, and per-spring visualizations. Collision handling with spheres uses the shared shape interface from the starter code.

## What I implemented
- Completed spring graph construction and per-type index buffers in [hw1/HW1/src/cloth.cpp](hw1/HW1/src/cloth.cpp).
- Added spring and damper force accumulation for all spring types in [hw1/HW1/src/cloth.cpp](hw1/HW1/src/cloth.cpp).
- Implemented four integrators (Explicit, Implicit, Midpoint, RK4) and the simulation callbacks in [hw1/HW1/src/integrator.cpp](hw1/HW1/src/integrator.cpp).
- Preserved pin constraints on the four cloth corners and provided multiple draw modes for debugging.

## How to build and run
Follow the project-specific instructions in the nested [hw1/HW1/README.md](hw1/HW1/README.md). That README covers the existing build system and dependencies.

## Notes and observations
- Spring–damper forces improve stability when combined with the higher-order integrators.
- Implicit Euler offers better robustness under larger time steps, while RK4 yields smoother motion at moderate steps.
- Visualizing structural, shear, and bending springs separately helps debug stretching and folding artifacts.
- See [hw1/report.pdf](hw1/report.pdf) for detailed analysis and screenshots.