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

## Managed SAM3 model (automatic provisioning for `rux gui`)

The `rux gui` segment endpoints provision SAM 3.1 automatically on first
use — no manual export or engine placement is required for GUI operation.

### Managed directory layout

```
<models-dir>/
  sam3.1/
    onnx/                  # portable ONNX bundle (downloaded once)
      vision-encoder.onnx
      text-encoder.onnx
      geometry-encoder.onnx
      decoder.onnx
      tracker-*.onnx        (4 files)
      tokenizer.json
      tracker-meta.json
      engine-build.json     # shape/precision recipe consumed by the C++ EngineBuilder
    engines/
      <gpu>-sm<cc>-trt<ver>/  # cache key: GPU name + compute capability + TRT version
        vision-encoder.engine
        text-encoder.engine
        ...
```

Engines in a cache-keyed subdirectory are never loaded on a different GPU /
TensorRT version combination; each machine builds its own set.

### Models-dir resolution precedence

1. `--models-dir <dir>` flag passed to `rux gui` (highest priority)
2. `$REUSEX_MODELS_DIR` environment variable
3. `$XDG_CACHE_HOME/reusex/models` (falls back to `$HOME/.cache/reusex/models`)

### Escape hatches

| Variable / flag | Effect |
|---|---|
| `--sam3-model <dir>` | Point `rux gui` at a pre-built model dir (skips managed provisioning entirely; back-compatible with the old explicit path) |
| `$REUSEX_SAM3_ONNX_DIR` | Point directly at a pre-exported ONNX bundle (e.g. `make -C python export` output) to skip download; engines are still built from it on-device |
| `--sam3-manifest-url <url>` | Override the default ONNX bundle download URL |

### On-device engine build

The C++ `EngineBuilder` (`libs/reusex/src/vision/tensor_rt/common/EngineBuilder.cpp`)
builds TensorRT engines from the portable ONNX using `nvonnxparser` + `IBuilder`.
The shape/precision recipe is read from `engine-build.json` — the same file
emitted by `python/reusex_sam3/build_engines.py` (`--emit-profiles`). This is
the single source of truth: the fp32-vision-encoder rule and all dynamic shape
profiles are encoded there, not duplicated between Python and C++.

To regenerate `engine-build.json` after changing `SHAPE_PROFILES` or
`FP32_ENGINES` in `build_engines.py`:

```bash
python -m reusex_sam3.build_engines --emit-profiles
```

`make -C python engines` also writes the file into the ONNX output directory
so the exported bundle is self-describing.

### Lazy provisioning and status polling

On the first segment request with no `model_path`, the server kicks off a
background provisioning task and returns HTTP 503 with a message like
`"SAM3 model is being prepared (downloading): ... — poll GET /api/v1/models/sam3/status"`.

`GET /api/v1/models/sam3/status` returns:

```json
{ "state": "building", "progress": 0.42, "message": "...", "use_cuda": true }
```

`state` ∈ `{absent, downloading, building, ready, error}`. When `state` is
`ready`, the response also includes `"model_path"`.

### SAM License and ONNX redistribution

The portable ONNX bundle is a derivative of Meta's gated `facebook/sam3.1`
checkpoint. Under the SAM License (https://huggingface.co/facebook/sam3/blob/main/LICENSE §1.b.i),
derivative works may be redistributed **only under the SAM License**, with a copy
of `LICENSE_SAM.txt` bundled alongside, and **not** relicensed under this
repo's GPL-3.0.

The planned distribution channel is a GitHub Release asset (manifest.json +
sha256 per file + LICENSE_SAM.txt). The download URL is not yet pinned — a
`TODO` in `libs/reusex/src/vision/sam3/sam3_assets.cpp` (`kDefaultManifestUrl`)
marks where it will be filled in. Until then, set `$REUSEX_SAM3_ONNX_DIR` to
point at a local ONNX export.
