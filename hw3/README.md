# NYCU Computer Animation 2024 Spring HW3 — Motion Graph Synthesis

StudentID: 110612117  
Name: Chung-Yu Chang (張仲瑜)

## Introduction
This assignment builds a motion graph to synthesize long, varied locomotion from a handful of AMC clips. Each clip is chopped into fixed-length segments; per-frame pose distances (with higher weights on hips, knees, and shoulders) score how blendable two segment endpoints are. Edges are created between consecutive segments and any pair whose overlap cost stays below a threshold. When traversing the graph, the player selects an outgoing edge stochastically, aligns the root of the destination segment to the current segment, and blends across a sine-weighted window to hide seams. The result is a stream of motions that can jump between clips while preserving spatial continuity and reducing foot-skate artifacts.

## What I implemented
- Segmenting the source motions, weighting key joints, and precomputing an overlap distance matrix in [HW3_110612117/src/acclaim/motion_graph.cpp](HW3_110612117/src/acclaim/motion_graph.cpp#L24-L139).
- Motion graph edge construction with consecutive links, thresholded cross-links, and normalized transition probabilities in [HW3_110612117/src/acclaim/motion_graph.cpp](HW3_110612117/src/acclaim/motion_graph.cpp#L141-L203).
- Probabilistic traversal that root-aligns jumped segments and blends them with a sine window to generate continuous output in [HW3_110612117/src/acclaim/motion_graph.cpp](HW3_110612117/src/acclaim/motion_graph.cpp#L205-L260).
- Root-space alignment of motion segments to target facing/position and quaternion-based windowed blending in [HW3_110612117/src/acclaim/motion.cpp](HW3_110612117/src/acclaim/motion.cpp).

## How to build and run
Use the provided project files under HW3_110612117 (Visual Studio solution/CMake as shipped in the starter). Build and run with the same toolchain described by the course starter kit.

## Notes and observations
- Joint weights emphasize lower-body joints to prioritize gait consistency; weights are normalized before scoring transitions.
- Edge costs average pose distances over the blend window; links are pruned when costs exceed the threshold to avoid visible pops.
- Sine-based blend weights give smooth ease-in/ease-out across the transition window, reducing discontinuities when jumping between clips.
- See [HW3_110612117/HW3_report_110612117.pdf](HW3_110612117/HW3_report_110612117.pdf) for detailed analysis, results, and comparisons.
