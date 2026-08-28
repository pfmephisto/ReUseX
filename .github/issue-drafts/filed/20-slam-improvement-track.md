title: Research: improve pose quality beyond stock RTABMap
labels: research, epic

## Problem
RTABMap poses limit reconstruction quality, and the input capture quality is
fixed — improvements must come from better processing. Poor poses cascade
into noisy merged clouds, fragmented plane segmentation, and failed solid
models.

## Direction (to be refined once benchmarking exists)
- Establish pose/reconstruction quality metrics first (plane flatness
  residuals, wall thickness spread, loop-closure consistency) so any change
  is measurable — depends on the benchmark harness
- Candidate approaches, roughly increasing effort:
  1. RTABMap parameter optimization against the metric suite
  2. Global refinement pass on stored frames: plane-aware pose-graph
     optimization / bundle adjustment (building on existing
     `joint_pairwise_registration` and COLMAP export)
  3. Depth-frame preprocessing (confidence-weighted filtering) before
     integration
  4. Alternative/hybrid SLAM back-end evaluation

## First step
Define and implement the metric suite on the existing fixture scans; get
baseline numbers for the current pipeline.

category=Geometry estimate=2w
