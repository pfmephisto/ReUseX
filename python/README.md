<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# reusex_sam3 — SAM 3.1 → ONNX → TensorRT export

Reproducible pipeline that converts Meta's `facebook/sam3.1`
`sam3.1_multiplex.pt` checkpoint into ONNX graphs and TensorRT engines whose I/O
names match the shared engine-I/O contract the ReUseX C++ side binds to.

> This is the run-order + quickstart README. **The full teaching write-up — how
> the model was taken apart, every ONNX-export blocker and its fix, and how the
> C++ tracker consumes the engines — lives in
> [`../docs/sam3.1-tensorrt.md`](../docs/sam3.1-tensorrt.md).** Read that if you
> want to *understand* the pipeline; read this if you just want to *run* it.

## What this produces

**Eight** TensorRT engines plus `tracker-meta.json`:

- Detector (stateless, re-exported 1:1 from the SAM 3 detector — SAM 3.1 changed
  only the video tracker): `vision-encoder`, `text-encoder`, `geometry-encoder`,
  `decoder`.
- Tracker (new for SAM 3.1's Object Multiplex video path):
  `tracker-memory-encoder`, `tracker-memory-attention`, `tracker-prompt-encoder`,
  `tracker-multiplex-decoder`.

Drop the `.engine` files + `tokenizer.json` + `tracker-meta.json` into one
directory and point `rux annotate` at it (see "Feeding the engines to rux").

## Setup (venv, prebuilt wheels — NOT nix source builds)

> Do **not** build torch from nix source — this machine OOMs compiling torch.
> Use the official prebuilt cu124 wheels (see `requirements.txt` header).

```bash
python -m venv .venv && . .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
# native model source (see third_party/README.md for the pinned SHA
# 20dba30a35a497606b06cf241f5b5605ea10e77e):
git clone --branch sam3.1 https://github.com/facebookresearch/sam3.git third_party/sam3
pip install -e third_party/sam3 --no-build-isolation
```

The `sam3.1` checkpoint is a **gated** HuggingFace repo: accept the licence at
<https://huggingface.co/facebook/sam3.1> and authenticate (`huggingface-cli
login` or `export HF_TOKEN=...`) before `make download`.

## Run order

The stages are strictly ordered — each consumes the previous stage's artifacts:

```
download → load(inspect) → export-detector → export-tracker → engines → verify
```

```bash
make download          # gated HF download of sam3.1_multiplex.pt (+ bpe) → checkpoints/
make load              # inspect checkpoint + write checkpoints/remap_report.json
make export-detector   # → onnx/{vision,text,geometry}-encoder.onnx, decoder.onnx
make export-tracker    # → onnx/tracker-*.onnx + onnx/tracker-meta.json
make engines           # trtexec → engines/*.engine (FP16, dynamic shapes)
make verify            # torch vs onnxruntime vs tensorrt parity (per engine)
make drift             # 30-frame recurrent tracker drift test (optional)
# or the whole chain:
make all               # download → load → export → engines → verify
```

Parameterise with `CHECKPOINT=`, `ONNX_DIR=`, `ENGINE_DIR=` (e.g.
`make engines ENGINE_DIR=/models/sam3.1`). `make engines --dry-run`-equivalent:
run `python -m reusex_sam3.build_engines --dry-run` to print the `trtexec`
commands without building.

### First run after weights land

`load_native.py` remaps the checkpoint keys against the freshly-built empty
model and **hard-asserts** on any non-allowlisted residual. If `make load`
fails that assert, run the standalone inspector to see exactly which keys drift,
then extend `_REGEX_TRANSFORMS` / `_ALLOWLIST_PREFIXES` in `load_native.py`:

```bash
python -m reusex_sam3.inspect_checkpoint --checkpoint checkpoints/sam3.1_multiplex.pt
# section (1) works even without sam3/torch installed (prefix dump only)
python -m reusex_sam3.inspect_checkpoint --checkpoint checkpoints/sam3.1_multiplex.pt --no-model
```

### Optional: INT8 vision encoder

The ViT-L vision encoder dominates latency. `ptq_vision.py` documents the
NVIDIA Model-Optimizer ONNX-PTQ recipe (it raises `NotImplementedError` until a
calibration set is wired in); once you have a `vision-encoder.int8.onnx`, build
with `python -m reusex_sam3.build_engines --int8-vision`.

## Feeding the engines to rux

The C++ side (`TensorRTSam3p1`) discovers engines by filename in one directory:

```
<model_dir>/
  vision-encoder.engine  text-encoder.engine  decoder.engine   # required detector
  geometry-encoder.engine                                      # optional (unused by video path)
  tracker-memory-encoder.engine  tracker-memory-attention.engine  # required tracker
  tokenizer.json  tracker-meta.json
```

Presence of the two `tracker-*` engines is what selects the SAM 3.1 stateful
video path over the plain SAM 3 detector. Then:

```bash
rux annotate <project.rux> --net <model_dir> --video
# (--video is implicit when a SAM 3.1 directory is detected)
```

## Engine I/O contract

The authoritative I/O contract (names, shapes, dtypes, dynamic axes) lives in
`reusex_sam3/__init__.py` as `ENGINE_IO_CONTRACT`, and the memory-bank integer
layout in `TRACKER_META`. Both are dependency-free (no torch import at package
load), so they can be introspected on a GPU-less box and asserted against in
`verify.py`, `build_engines.py`, and C++ codegen. The C++ side binds to these
exact tensor names — do not rename an engine output without updating both sides.
The full contract is reproduced and explained in
[`../docs/sam3.1-tensorrt.md`](../docs/sam3.1-tensorrt.md).

## Layout

| file | role |
|------|------|
| `download.py` | gated HF download / local checkpoint passthrough |
| `inspect_checkpoint.py` | standalone key-prefix + remap dump (run once weights land) |
| `load_native.py` | build empty demo model + diff-driven bijective remap |
| `fixes.py` | ONNX-export monkeypatches (pos-enc, RoPE, scatter, roi_align, fp16 mask) |
| `wrappers_detector.py` | 4 detector engine wrappers |
| `wrappers_tracker.py` | 4 tracker engine wrappers |
| `export_detector.py` / `export_tracker.py` | torch.onnx.export drivers |
| `build_engines.py` | trtexec driver (FP16, dynamic shapes, `--int8-vision` stub) |
| `ptq_vision.py` | INT8 vision PTQ recipe (documented stub) |
| `verify.py` | 3-way parity + 30-frame drift harness |

## Notes

- FlashAttention-3 is disabled (`use_fa3=False`) and real-valued RoPE is enabled
  (`use_rope_real=True`) at model-build time for trace/ONNX compatibility; the
  ONNX-hostile native code paths are additionally monkeypatched for the duration
  of each `torch.onnx.export` by `fixes.apply_all()` (restored on exit so the
  `verify.py` PyTorch reference runs against the *un-patched* model). See
  [`../docs/sam3.1-tensorrt.md`](../docs/sam3.1-tensorrt.md) for the full list of
  blockers and fixes.
- All exports are opset 17, TorchScript exporter (`dynamo=False`),
  `do_constant_folding=True`.
- `checkpoints/`, `onnx/`, `engines/` are git-ignored (large artifacts).
- The native repo is pinned, not vendored — see `third_party/README.md`.
