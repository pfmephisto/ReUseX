<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Models

This directory holds pre-trained model weights used by `rux create annotate`
and related vision commands. **Model files are never committed to this repo** —
`.gitignore` ignores everything under `models/` except this README (`models/*`
+ `!models/README.md`). Weights are downloaded or exported locally by each
developer/CI runner as needed.

## Expected layout

Place files directly in this directory (or point the CLI at another path
with `-n`/`--net`):

| File | Purpose |
|---|---|
| `yolo11n.pt`, `yolo11l.pt`, `yolo11x.pt` | YOLO11 object detection (varying size/accuracy tradeoffs) |
| `yolo11n-seg.pt`, `yolo11l-seg.pt` | YOLO11 instance segmentation |
| `sam2.1_s.pt`, `sam2.1_b.pt`, `sam2_hiera_large.pt` | SAM2 segmentation checkpoints |
| `superpoint.pt` | SuperPoint feature detector |
| `*.engine` | TensorRT engines exported from the above for optimized inference |

## Getting model files

- YOLO11 / SAM2 checkpoints: download from their respective upstream
  releases (Ultralytics / Meta) and drop the `.pt` file here.
- TensorRT `.engine` files: export locally from a `.pt`/`.onnx` checkpoint
  for your target GPU (engines are not portable across GPU architectures /
  TensorRT versions, so they must be built per machine, not shared).

See `CLAUDE.md` ("Pre-trained Models" section) and `rux create annotate --help` for
how the CLI locates and selects model backends
(`vision/BackendFactory.hpp` picks a backend from the file extension).
