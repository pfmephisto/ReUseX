<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# SAM 3.1 export guide — how to actually re-export and modify the model

This is the **hands-on, "how do I do it and change it"** companion to
[`sam3.1-tensorrt.md`](sam3.1-tensorrt.md). That document explains *why* the
model is decomposed into per-component engines and *why* certain layers can't be
exported. This one tells you the **practical workflow**: how to set up the
environment, re-export an engine, wrap a new sub-module, fix a fresh
ONNX-export failure, build the TensorRT engine, and verify the change didn't
break parity.

If you want the theory, read the other doc. If you have a checkpoint and a task
("re-export the decoder", "add a new tracker engine", "torch.onnx.export just
threw an error I've never seen"), you're in the right place.

All paths below are relative to the `python/` directory of this worktree
(`.worktrees/sam3.1-tensorrt/python/`) unless stated otherwise. Every code
reference is `file:function` so you can open the source alongside.

## Table of contents

1. [Setup — venv, LD_LIBRARY_PATH, checkpoint, native repo](#1-setup)
2. [The mental model — one component = one wrapper = one ONNX = one engine](#2-the-mental-model)
3. [Load and inspect the checkpoint](#3-load-and-inspect-the-checkpoint)
4. [Modify the model / add a new component (worked example)](#4-modify-the-model--add-a-new-component)
5. [Make a layer ONNX-exportable (the monkeypatch recipe)](#5-make-a-layer-onnx-exportable)
6. [Export to ONNX, then build the engine](#6-export-to-onnx-then-build-the-engine)
7. [Verify a change](#7-verify-a-change)
8. [Troubleshooting table](#8-troubleshooting-table)

---

## 1. Setup

You **cannot** run this from the Nix dev shell: building torch from Nix source
OOMs this machine (see `requirements.txt` header, `python/README.md` §Setup).
The pipeline runs from a plain venv with **prebuilt cu124 wheels**.

### 1.1 The venv

The venv already exists at `python/.venv`. It was built like this
(`python/README.md`, `requirements.txt`):

```bash
python -m venv .venv && . .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
# torch/torchvision come from the pytorch cu124 index, not PyPI:
pip install torch==2.9.0      --index-url https://download.pytorch.org/whl/cu124
pip install torchvision==0.24.0 --index-url https://download.pytorch.org/whl/cu124
```

Pinned wheels (`requirements.txt`): `torch==2.9.0` (matches ReUseX LibTorch
2.9.0), `onnx==1.17.0`, `onnxruntime-gpu==1.20.1`, `numpy==1.26.4`,
`opencv-python`, `huggingface_hub==0.26.2`, `polygraphy==0.49.9` (the TensorRT
engine runner used by `verify.py`). INT8 PTQ (`nvidia-modelopt[onnx]`) is
optional and commented out.

### 1.2 The environment helper (`/tmp/sam3env.sh`)

torch's bundled CUDA libs and the system driver/libstdc++ must be on
`LD_LIBRARY_PATH` or imports fail with symbol/driver errors. Source this before
any run:

```bash
source /tmp/sam3env.sh
```

It sets (verbatim, adjust the store hashes if the Nix GC moved them):

```bash
export SAM3_PY=.../python/.venv/bin/python
GCC=/nix/store/xm08aqdd7pxcdhm0ak6aqb1v7hw5q6ri-gcc-14.3.0-lib/lib
NVLIBS=$(echo .../python/.venv/lib/python*/site-packages/nvidia/*/lib | tr ' ' ':')
export LD_LIBRARY_PATH="$GCC:/run/opengl-driver/lib:$NVLIBS"
export CKPT=/home/mephisto/repos/ReUseX/models/sam3.1_multiplex.pt
```

So `LD_LIBRARY_PATH` = **gcc-lib** (libstdc++) + **`/run/opengl-driver/lib`**
(the NVIDIA driver, NixOS location) + **the torch-bundled nvidia libs** (cuBLAS,
cuDNN, cuRT, …). `$SAM3_PY` is the venv python; `$CKPT` is the checkpoint.

> **Caveat for `trtexec`.** `trtexec` is a *system* (Nix) binary and must **not**
> see the venv's torch nvidia libs — they collide with the system TensorRT libs.
> Always invoke it with the torch libs stripped:
>
> ```bash
> env -u LD_LIBRARY_PATH trtexec --onnx=... --saveEngine=...
> ```
>
> `build_engines.py` shells out to `trtexec` via `subprocess`; run the whole
> `make engines` / `build_engines` step under `env -u LD_LIBRARY_PATH` so the
> child `trtexec` gets a clean library path.

### 1.3 The gated checkpoint

`facebook/sam3.1` is a **gated** HF repo. Accept the licence at
<https://huggingface.co/facebook/sam3.1>, authenticate
(`huggingface-cli login` or `export HF_TOKEN=...`), then:

```bash
$SAM3_PY -m reusex_sam3.download          # → checkpoints/sam3.1_multiplex.pt (+ config, bpe)
```

`download.py:download()` is idempotent (reuses the HF cache) and accepts
`--checkpoint /path/to.pt` to skip the network and just symlink an existing
file into `checkpoints/`. The token resolves from `--token` → `HF_TOKEN` /
`HUGGING_FACE_HUB_TOKEN` → the cached `huggingface-cli login` token
(`download.py:_resolve_token`). If the gated pull stalls (HF Xet transfer can
hang), fall back to a direct `curl`/`hf download` and pass the result with
`--checkpoint` (see §8).

### 1.4 The pinned native `sam3` repo

The pipeline imports the **native** `sam3` package to build the real model.
It is **pinned, not vendored** (`python/third_party/README.md`):

| field  | value |
|--------|-------|
| repo   | `https://github.com/facebookresearch/sam3` |
| branch | `sam3.1` |
| commit | `20dba30a35a497606b06cf241f5b5605ea10e77e` |

```bash
cd python/third_party
git clone --branch sam3.1 https://github.com/facebookresearch/sam3.git sam3
cd sam3 && git checkout 20dba30a35a497606b06cf241f5b5605ea10e77e
pip install -e . --no-build-isolation
```

A read-only checkout of this same tree is already at `/tmp/sam3-inspect` (branch
`sam3.1`) — use it to look up native module paths (`sam3/model/*.py`) while
writing wrappers.

> **Critical:** every wrapper, patch, and accessor in this package was written
> against the class names / attribute paths / `forward` signatures at **exactly**
> this SHA (`__init__.py:NATIVE_SHA`). If you bump the pin, re-verify all of
> `load_native.py`, `wrappers_detector.py`, `wrappers_tracker.py`, `fixes.py`.

### 1.5 Running anything

Everything runs through the venv python and, for engines, a stripped-env
`trtexec`. The `Makefile` wraps the stages; set `PY=$SAM3_PY`:

```bash
source /tmp/sam3env.sh
make -C python PY="$SAM3_PY" CHECKPOINT="$CKPT" export-detector
env -u LD_LIBRARY_PATH make -C python PY="$SAM3_PY" engines
```

Or call the modules directly (each is `python -m reusex_sam3.<module>`).

---

## 2. The mental model

**You never export the whole model.** The full
`Sam3MultiplexTrackingWithInteractivity` is stateful (a per-frame memory bank),
control-flow-heavy (Python `if`/loops + a `MultiplexState` object), and full of
CUDA-only / untraceable ops — none of which ONNX (a static dataflow DAG) can
represent. The theory is in `sam3.1-tensorrt.md` §2.

Instead, for **each component** you repeat this six-step loop:

```
(a) load the native model on CPU        load_native.load(ckpt) → NativeHandles
(b) grab one native sub-module          handles.det_vision_neck, handles.tracker.…
(c) wrap it: clean forward, fixed I/O    wrappers_*.py  (nn.Module)
(d) monkeypatch unexportable ops         fixes.apply_all()  (context manager)
(e) torch.onnx.export the wrapper        export_*.py  (opset 17, dynamo=False)
(f) trtexec the ONNX → .engine           build_engines.py
```

The result is **one component = one wrapper = one ONNX = one engine**. There are
currently 8 (4 detector + 4 tracker), each a *pure function of tensors* with a
fixed I/O contract (`__init__.py:ENGINE_IO_CONTRACT`). All the control flow and
the memory bank live in **C++** (`libs/reusex/src/vision/tensor_rt/Sam3p1.cpp`),
not in any engine.

The boundary between Python and C++ is the **engine-I/O contract**: exact tensor
names, shapes, dtypes, dynamic axes. If you change I/O names, you change both
`ENGINE_IO_CONTRACT` and the C++ binding maps in `Sam3p1.cpp` — they must match.

---

## 3. Load and inspect the checkpoint

### 3.1 Building the model on CPU

`load_native.build_demo_model()` reproduces the native builder
(`sam3.model_builder.build_sam3_multiplex_video_predictor`)
**component-for-component on CPU**, but stops before the native builder's
`.cuda()` and predictor-wrapping. It calls the native `mb._create_*` helpers
(`_create_multiplex_tri_backbone`, `_create_text_encoder`,
`_create_sam3_transformer`, `_create_segmentation_head`,
`_create_geometry_encoder`, `_create_dot_product_scoring`) and assembles a bare
`Sam3MultiplexTrackingWithInteractivity` with `.detector` + `.tracker`. It is
built with the **export-friendly combination**: `use_fa3=False` (no
FlashAttention-3, a CUDA-only kernel) and `use_rope_real=True` (real-valued RoPE,
no complex ops — see §5).

`load_native.load(ckpt)` does build + checkpoint-load + returns a
`NativeHandles`.

### 3.2 The remap, and why it's diff-driven

The checkpoint keys don't necessarily match the freshly-built model's keys.
`strict=False` alone is dangerous: a systematic prefix mismatch would silently
load *mostly random weights* and still "succeed". So `load_native.py` does a
**diff-driven, bijective remap** then **hard-asserts** the residual
(`remap_state_dict`, `load_checkpoint_into`):

1. Static prefix remaps (`_PREFIX_REMAPS`: `sam3_model.`→`detector.`,
   `sam2_predictor.`→`tracker.`) — applied only if the checkpoint actually uses
   those internal prefixes (`needs_prefix`).
2. Regex transforms (`_REGEX_TRANSFORMS`) kept **only if the transformed key
   exists in the model** — so the remap can never fabricate a parameter name.
3. `load_state_dict(strict=False)`, then assert every residual missing/unexpected
   key is covered by `_ALLOWLIST_PREFIXES` (buffers we re-register ourselves:
   `freqs_cis`, `position_encoding.cache`; plus unused `interactive_*` heads).
   Anything else → `AssertionError` (`hard_assert=True`).

Every run writes `checkpoints/remap_report.json` with counts + residual lists.

### 3.3 Re-validating a checkpoint

For the current gated `sam3.1_multiplex.pt`, the expected result is recorded in
`load_native.py` (the `# VERIFIED against the real facebook/sam3.1` comment):

> **1623 tensors**, top-level prefixes `detector.` (1166) and `tracker.model.`
> (457), already in native demo-model naming with flattened attention names
> (`...self_attn_q_proj.weight`). So the prefix/regex remaps are no-ops for this
> checkpoint and keys are expected to **match 1:1** — `matched 1623/1623`, with
> the ~64 RoPE / pos-enc buffers legitimately absent (allowlisted, re-registered
> at trace time).

To re-confirm on any checkpoint (new download, bumped pin), run the standalone
inspector (`inspect_checkpoint.py`):

```bash
# section (1) only — prefix dump, works WITHOUT sam3/torch installed:
$SAM3_PY -m reusex_sam3.inspect_checkpoint --checkpoint "$CKPT" --no-model
# full diff — builds the empty model and prints residuals (needs sam3 + torch):
$SAM3_PY -m reusex_sam3.inspect_checkpoint --checkpoint "$CKPT"
```

If the full run reports non-allowlisted residuals, extend `_REGEX_TRANSFORMS`
(for renamed params) or `_ALLOWLIST_PREFIXES` (for buffers you register yourself
or known-unused subtrees) in `load_native.py`, then re-run.

### 3.4 Finding what to wrap (submodule attribute paths)

The wrappers target **live submodules** by attribute path, exposed through
`load_native.NativeHandles` (e.g. `det_vision_neck`, `det_text_encoder`,
`trk_maskmem_backbone`, `trk_memory_attention`). To discover the path of a
sub-module you want to export, print the module tree:

```python
from reusex_sam3.load_native import load
h = load("checkpoints/sam3.1_multiplex.pt")

# detector / tracker roots:
det, trk = h.detector, h.tracker

# find the attribute path of every leaf module you might wrap:
for name, mod in det.named_modules():
    print(name, type(mod).__name__)

# or one level at a time to navigate:
for name, child in trk.named_children():
    print(name, type(child).__name__)
```

Match the printed dotted path (e.g. `transformer.encoder`) to an accessor on
`NativeHandles`, or add a new accessor property there. Cross-check against the
native source in `/tmp/sam3-inspect/sam3/model/*.py`.

---

## 4. Modify the model / add a new component

This is the core workflow. Say you want to **export a new native sub-module**
(or change an existing one) into its own engine. Follow these steps; every one
has a concrete anchor in the existing code.

### Step 1 — find the native module

Locate the `nn.Module` you want and its `forward` signature. Use §3.4 to get its
attribute path, and read the native `forward` in `/tmp/sam3-inspect`. Note:
- the exact **positional/keyword arguments** it wants,
- its internal **layout convention** (the native model is largely **seq-first**
  `[Seq, Batch, Channels]`; the engine contract is **batch-first** `[Batch, …]`),
- what it **returns** (often a dict — you'll pull named tensors out).

### Step 2 — add a `NativeHandles` accessor

If there isn't already a property for the module, add one in
`load_native.NativeHandles`, mirroring the existing ones:

```python
@property
def trk_my_new_module(self):
    return self.tracker.my_new_module  # SomeNativeClass
```

Keep the returned-type comment — it documents the contract for the wrapper.

### Step 3 — write the wrapper

Add an `nn.Module` subclass in `wrappers_detector.py` or `wrappers_tracker.py`.
The shape to follow (from `wrappers_detector.py:TextEncoderWrapper` /
`GeometryEncoderWrapper`):

```python
class MyComponentWrapper(nn.Module):
    """input_a[B,...], input_b[B,...] -> output_x[B,...], output_y[B,...].

    One-line-per-arg docstring of the exact contract shapes; note any
    seq-first <-> batch-first transposes and any deviations from native.
    """

    def __init__(self, handles):
        super().__init__()
        self.mod = handles.trk_my_new_module   # grab the live native submodule

    def forward(self, input_a, input_b):
        # 1. translate batch-first contract inputs -> native seq-first if needed
        a = input_a.transpose(0, 1).contiguous()      # [B,S,C] -> [S,B,C]
        # 2. call the REAL native forward
        out = self.mod(a, input_b)                    # native returns e.g. a dict
        # 3. pull named tensors, translate back to batch-first, and return
        #    IN THE EXACT ORDER of the contract's output_names
        x = out["x"].transpose(0, 1).contiguous()
        y = out["y"]
        return x, y
```

Rules the existing wrappers all obey:
- **`forward` returns tensors in the exact order of the contract
  `output_names`** — `torch.onnx.export` binds outputs positionally.
- Wrap the **real** native submodule; don't reimplement its math unless you must
  drop untraceable bookkeeping (see `MemoryEncoderWrapper`, which reproduces just
  the tensor math of `_encode_new_memory` and drops the `MultiplexState` object).
- Do batch-first↔seq-first transposes **inside** the wrapper so the engine
  boundary stays batch-first.
- If a contract input is *functionally unused* by the native module but must
  still appear in the graph, tie it into an output as an exact-zero so ONNX
  doesn't prune it — see `MemoryAttentionWrapper.forward`'s
  `keep = memory_mask.sum() * 0.0` trick.

### Step 4 — add the contract entry

In `__init__.py`, add a `ENGINE_IO_CONTRACT["my-component"]` block: `inputs`,
`outputs` (names + shapes + dtypes), and `dynamic_axes`. Copy the shape of an
existing entry. Conventions: float tensors are `float32` in ONNX; masks are
`bool`; token-id tensors are `int64` (opset-17 `Gather` needs int64 indices).
Only put an axis in `dynamic_axes` if it's genuinely dynamic — everything else is
baked (see §5's constant-folding note; `prompt_len`/`num_boxes` are *fixed* on
purpose).

### Step 5 — add example inputs + register in the export driver

In `export_detector.py` or `export_tracker.py`:
- Add a branch to `_example_inputs(engine)` returning a tuple of fixed-shape
  tracing tensors (batch=1, small N/L). These shapes are what the exporter
  traces and constant-folds against, so any dimension that must be *fixed* in the
  engine (see §5) is set here.
- Register the wrapper in the `builders` dict inside `export_all`:

```python
builders = {
    ...,
    "my-component": lambda h: MyComponentWrapper(h),
}
```

`--only my-component` then exports just yours.

### Step 6 — add the shape profile

In `build_engines.py`, add `SHAPE_PROFILES["my-component"]`: a
`{input_name: (min, opt, max)}` dict of shape triples. Static inputs get
`min==opt==max`; dynamic ones get a real range (batch `1→1→4`, etc.). Any
dimension you did **not** mark dynamic in the contract must be identical across
min/opt/max here (it's baked). `ALL_ENGINES` is derived from `SHAPE_PROFILES`,
so adding the entry wires it into `make engines`.

### Step 7 — match the C++ binding names (if C++ consumes it)

If the new engine is consumed by the C++ pipeline, the binding names in
`libs/reusex/src/vision/tensor_rt/Sam3p1.cpp` must match your contract's
`input_names`/`output_names` **exactly**. Grep for an existing engine name (e.g.
`fpn_feat_2`, `pix_feat_with_mem`) in `Sam3p1.cpp` to see how bindings are set,
and add yours the same way. The C++ side discovers engines by filename in the
model dir; drop `my-component.engine` there.

---

## 5. Make a layer ONNX-exportable

This is the most valuable section: when `torch.onnx.export` throws on a native
op, this is how you fix it. Every fix lives in `fixes.py` and is applied as a
context manager around the export call (`export_*.py:_export` wraps the export in
`with torch.no_grad(), apply_all():`).

### 5.1 The monkeypatch pattern

You **patch the symbol at its import site** and **restore it via an
`ExitStack`** so the un-patched model is intact afterwards (verify.py compares
against the un-patched reference). The template (`fixes.py:_patch_*`):

```python
def _patch_something(handles, stack: contextlib.ExitStack):
    import sam3.model.some_module as m
    orig = m.some_module.SomeClass.forward    # or a free function m.some_fn

    def patched(self, *a, **k):
        ...  # ONNX-friendly reimplementation
    m.SomeClass.forward = patched
    stack.callback(setattr, m.SomeClass, "forward", orig)   # auto-restore on exit
```

Then add it to `apply_all()`'s `try` block. `apply_all()` opens one `ExitStack`,
applies every patch, `yield`s, and `stack.close()` restores all of them in
`finally`.

**Watch the import site.** If a module does `from x import fn` at import time,
patching `x.fn` is too late — the name is already bound in the consumer's module.
`_patch_fused_addmm_act` handles exactly this: `vitdet` did
`from sam3.perflib.fused import addmm_act`, so the patch sets the symbol in
**both** `sam3.model.vitdet` and `sam3.perflib.fused`.

### 5.2 Diagnosing a new failure

When `torch.onnx.export` errors:
1. **Read the traceback to the offending op / module.** The exporter names the
   `aten::` op or the Python line that broke.
2. **Classify it** against the categories below.
3. **Write a patch** that replaces just that op with an ONNX-representable
   equivalent, valid for our *fixed-shape, right-padded, CPU-trace* export
   scenario (you don't need full generality — the engine only ever runs fixed
   shapes).
4. **Add it to `apply_all()`** and re-export.

### 5.3 The categories we hit, with the actual errors and fixes

Each is a real fix already in `fixes.py`; use them as templates for new failures
of the same *kind*.

**(a) Complex-valued ops (RoPE) → real-valued.** ONNX opset 17 has essentially no
complex-tensor support, and the native `SimpleRoPEAttention.forward` also
*mutates* `self.freqs_cis` when the seq length changes (a state mutation a static
graph can't honour). Fix is two-part: build with `use_rope_real=True`
(real sin/cos formulation, `build_demo_model`), and `_patch_simple_rope` freezes
`SimpleRoPEAttention.forward` to read the pre-registered `freqs_cis_real` /
`freqs_cis_imag` buffers and never mutate state.
Related device gotcha it also fixes: those freqs are built on `cuda` as *plain
attributes* (not buffers), so `.cpu()` doesn't move them → *"Expected all
tensors to be on the same device"* on CPU export; the patch does
`freqs.to(q.device)` at call time without mutating `self`.

**(b) Python-int shapes baked as constants.** The TorchScript exporter
(`dynamo=False`) **constant-folds python ints** into the graph — notably an
attention head-reshape bakes the *prompt length* and *number of boxes*. If the
engine is later run at a different length you get an enqueue-time reshape volume
mismatch, e.g. **"reshape would change the total number of elements from 2304 to
1024"**. Fix: **fix the value** at export (don't make it dynamic):
`export_detector.GEOM_NUM_BOXES = 8` and decoder `L = 32`; the matching
`SHAPE_PROFILES` entries set `min==opt==max` for those dims so TensorRT builds a
static engine at exactly that size. Only truly-runtime axes (batch, memory
length) stay dynamic. If you genuinely need a variable length, you must either
export multiple engines or re-architect the reshape to be shape-generic.

**(c) Data-dependent asserts / scatter → static reimplementation.**
`concat_padded_sequences` used `torch._assert_async` (a data-dependent
assertion) + `Tensor.scatter`. Fix (`_patch_geometry_concat` +
`concat_padded_right`): a plain `torch.cat` of the two sequences and masks —
valid because our export feeds fixed-length, already-right-padded prompts.
Same category: the geometry `roi_align` was fed a **python list** of per-batch
box tensors (via `.unbind(0)`) plus a CUDA-only `pin_memory()`.
`_patch_geometry_roi_align` rebuilds it as the traceable single `[K,5]` rois
tensor (column 0 = batch index) and drops `pin_memory`.

**(d) Fused / custom kernels → plain-eval replacement.** FlashAttention-3 is
disabled at build (`use_fa3=False`) — no ONNX equivalent. The fused
`addmm_act` (`sam3.perflib.fused`) is a perf kernel that also casts to
**bfloat16**, producing the eval-time error **"mat1 and mat2 must have the same
dtype"** (BFloat16 vs Float) when its bf16 output hits the fp32 `fc2` linear.
`_patch_fused_addmm_act` replaces it with a mathematically-equivalent **fp32**
`F.linear` + GELU/ReLU (and patches both import sites, see §5.1).

**(e) Unsupported dtype / op variant → cast or swap the op.** The bf16 issue in
(d) is the dtype-cast case. A pure op-variant case: `SimpleMaskDownSampler`
resized masks with `F.interpolate(..., antialias=True)`, which lowers to
`aten::_upsample_bilinear2d_aa` — **`UnsupportedOperatorError`** on opset 17.
`_patch_mask_downsampler_antialias` re-runs the interpolate with
`antialias=False` (a numerical no-op here because the resize is an *upsample*
1008→1152, and antialiasing only band-limits on downscale).

**(f) CUDA-resident caches on a CPU trace.** `PositionEmbeddingSine` caches its
grid in a `{shape: cuda_tensor}` dict; a cached CUDA tensor on a CPU trace bakes
a wrong-device constant and hides the real computation.
`_patch_position_encoding` empties `self.cache` for the trace, forcing the
already-`arange`-based recompute on the input's own device. Same pattern for the
DETR decoder's cuda-pinned coord cache (`_patch_decoder_coord_cache` nulls
`compilable_cord_cache` so it recomputes on `reference_boxes.device`).

**(g) fp16 additive masks: `-inf` → `-1e4`.** Not an export error but a
*numerical* one at fp16 engine runtime: `-inf` in a fused attention softmax
produces NaNs. `additive_mask_from_bool` clamps to the finite sentinel
`NEG_INF_FP16 = -1e4` (`exp(-1e4) ≈ 0` in softmax, no fp16 overflow).
`build_engines.FP16_MASK_CLAMP_ENGINES = {"tracker-memory-attention", "decoder"}`
flags where this matters.

> **A native-path gotcha you *will* hit while writing patches:** an import that
> "should" work fails because the symbol moved. Example we hit:
> **"cannot import name `functional_attention` from `sam3.sam.transformer`"** —
> it actually lives in `sam3.model.decoder` (that's exactly where
> `_patch_simple_rope` imports it: `from sam3.model.decoder import
> SimpleRoPEAttention, functional_attention`). When an import fails, grep the
> native tree (`/tmp/sam3-inspect`) for the symbol's real home rather than
> trusting the path in some docstring.

---

## 6. Export to ONNX, then build the engine

### 6.1 The `torch.onnx.export` settings and why

`export_detector.py:_export` / `export_tracker.py:_export` both call:

```python
torch.onnx.export(
    module, args, out_path,
    input_names=in_names, output_names=out_names,   # from ENGINE_IO_CONTRACT
    dynamic_axes=dyn,                                # from ENGINE_IO_CONTRACT
    opset_version=17,
    do_constant_folding=True,
    dynamo=False,                                    # legacy TorchScript exporter
)
```

- **opset 17** — the fixes target ops available and well-behaved at 17.
- **`dynamo=False`** — the classic TorchScript tracing exporter. It traces one
  execution and **constant-folds python ints**. That's *why* `prompt_len` and
  `num_boxes` must be fixed (§5b): the exporter bakes them into reshapes.
- **`do_constant_folding=True`** — folds constant subexpressions.
- Names + `dynamic_axes` come straight from `ENGINE_IO_CONTRACT`, so the ONNX I/O
  names are the contract names the C++ side binds.

Run it:

```bash
$SAM3_PY -m reusex_sam3.export_detector --checkpoint "$CKPT" --onnx-dir onnx --only decoder
$SAM3_PY -m reusex_sam3.export_tracker  --checkpoint "$CKPT" --onnx-dir onnx
```

`export_tracker` also writes `onnx/tracker-meta.json` (`write_meta`, from
`TRACKER_META`) — the memory-bank integer layout the C++ side reads.

### 6.2 `trtexec` → engine

`build_engines.py` shells out to `trtexec`, one invocation per engine, deriving
`--minShapes/--optShapes/--maxShapes` from `SHAPE_PROFILES` and adding `--fp16`
by default plus `--memPoolSize=workspace:8192`. Preview the commands without
building:

```bash
$SAM3_PY -m reusex_sam3.build_engines --dry-run
```

Build (remember the stripped env for the child `trtexec`, §1.2):

```bash
env -u LD_LIBRARY_PATH $SAM3_PY -m reusex_sam3.build_engines \
    --onnx-dir onnx --engine-dir engines --engines decoder
```

INT8 for the vision encoder (`--int8-vision`) is a **documented stub**: it
expects a Q/DQ ONNX (`vision-encoder.int8.onnx`) from `ptq_vision.py`, which
currently raises until a calibration set is wired in.

### 6.3 PROMINENT CAVEAT — the vision encoder MUST be built fp32

> **Do NOT build `vision-encoder` in fp16.** The ViT-L trunk is **bf16-native**,
> and TensorRT fp16 corrupts it: cosine collapses to ~**0.33** (garbage), which
> downstream shows up as **all-background** annotation output.
>
> - The ViT is **Myelin-fused** inside TensorRT, so per-layer fp16→fp32
>   precision constraints don't help — TensorRT fuses across the layers you tried
>   to pin.
> - **bf16 won't build** either.
> - So the vision encoder must be built **fp32**. Pass `--no-fp16` for it:
>   ```bash
>   env -u LD_LIBRARY_PATH $SAM3_PY -m reusex_sam3.build_engines \
>       --engines vision-encoder --no-fp16
>   ```
>
> The **text encoder, decoder, and all tracker engines are fp16-fine** — build
> those with the default `--fp16`. (Selective-precision fp16 vision is tracked as
> an open experiment, not yet working.)

---

## 7. Verify a change

Two independent checks: numerical parity, and the C++ contract.

### 7.1 Parity harness

`verify.py` is a **3-way parity harness**: for each engine it runs the PyTorch
reference (the **un-patched** model — that's why `apply_all` restores on exit),
onnxruntime on the `.onnx`, and TensorRT on the `.engine`, on the *same* fixed
random fixture. Feature engines are compared by **cosine ≥ 0.999**; mask engines
(`decoder` → `pred_masks`, `tracker-multiplex-decoder` → `masks`) by **IoU ≥
0.98** (binarised at logit > 0). Each backend is guarded (runs only if its
deps/artifacts exist), so it degrades cleanly.

```bash
$SAM3_PY -m reusex_sam3.verify --checkpoint "$CKPT" --onnx-dir onnx \
    --engine-dir engines --engines decoder
# recurrent tracker drift over 30 frames (IoU >= 0.9):
$SAM3_PY -m reusex_sam3.verify --checkpoint "$CKPT" --onnx-dir onnx \
    --engine-dir engines --drift --engines
```

### 7.2 Manual per-frame diagnostic (real image)

To sanity-check a real change on a real frame, run the ONNX graphs in dependency
order through onnxruntime and inspect tensor stats/cosine against the PyTorch
reference — the diagnostic we used chains:

```
vision.onnx → text.onnx → decoder.onnx
```

feeding the vision `fpn_feat_*`/`fpn_pos_2` and text `text_features` into the
decoder, then checking `pred_logits` (per-query scores) and `presence_logits` are
sane (not all-background, not NaN). If `pred_logits` are uniformly very negative
→ suspect the fp16-vision corruption (§6.3). Compare cosine of each intermediate
tensor against the un-patched PyTorch forward to localise *which* stage drifted.

### 7.3 C++-contract check

Independently of numbers, confirm the I/O **names** still match:
- the engine's `input_names`/`output_names` in `ENGINE_IO_CONTRACT`, and
- the binding maps in `libs/reusex/src/vision/tensor_rt/Sam3p1.cpp`.

A renamed output that passes parity will still fail at C++ bind time. Grep both
sides for the tensor name after any rename.

---

## 8. Troubleshooting table

| symptom | cause | fix |
|---|---|---|
| Annotation output is **all background** | `vision-encoder` built fp16 → ViT bf16 corruption (cosine ~0.33) | Rebuild vision-encoder **fp32** (`--no-fp16`, §6.3). Other engines stay fp16. |
| `torch.onnx.export`: **"mat1 and mat2 must have the same dtype"** (BFloat16 vs Float) | fused `addmm_act` casts to bf16, hits fp32 `fc2` | `_patch_fused_addmm_act` (fp32 linear+act); confirm it's in `apply_all()`. Patch **both** import sites. |
| enqueue: **"reshape would change the total number of elements ... 2304 to 1024"** | baked `prompt_len`/`num_boxes` constant ≠ runtime shape | Fix the value at export (`GEOM_NUM_BOXES=8`, decoder `L=32`) and set `SHAPE_PROFILES` `min==opt==max` for that dim (§5b). |
| `torch.onnx.export`: **"cannot import name `functional_attention` from `sam3.sam.transformer`"** | symbol moved | It's in `sam3.model.decoder` — grep `/tmp/sam3-inspect` for the real home (§5 gotcha). |
| `UnsupportedOperatorError` on `_upsample_bilinear2d_aa` | `F.interpolate(antialias=True)` | `_patch_mask_downsampler_antialias` (antialias=False; no-op on upsample). |
| **"Expected all tensors to be on the same device, cuda:0 and cpu"** during CPU export | RoPE freqs / coord cache built on cuda as plain attributes | `_patch_simple_rope` / `_patch_decoder_coord_cache` move to `q.device` / recompute on input device. |
| TensorRT: **"input shapes not specified"** at runtime | static engine needs explicit run dims | Per-engine `set_binding_dim` / `set_run_dims` on the C++ side within the built profile (see `Sam3p1.cpp`). |
| fp16 attention output is **NaN** | `-inf` additive mask overflows fused fp16 softmax | Clamp to `-1e4` (`additive_mask_from_bool` / `NEG_INF_FP16`); engine listed in `FP16_MASK_CLAMP_ENGINES`. |
| `make load` **AssertionError**: non-allowlisted residual keys | checkpoint key drift vs built model | Run `inspect_checkpoint.py`, read residuals, extend `_REGEX_TRANSFORMS` / `_ALLOWLIST_PREFIXES` (§3.3). |
| `trtexec` fails with library/symbol errors | venv torch nvidia libs collide with system TensorRT | Run `trtexec` (and `build_engines`) under `env -u LD_LIBRARY_PATH` (§1.2). |
| import errors on `import torch` / driver symbols | `LD_LIBRARY_PATH` missing gcc-lib / driver / torch nvidia libs | `source /tmp/sam3env.sh` before running (§1.2). |
| gated HF download **stalls** (Xet transfer hangs) | HF Xet backend | Use a direct `curl` / `hf download`, then pass the file with `download.py --checkpoint /path.pt` (§1.3). |

---

## See also

- **Theory / decomposition rationale + full contract:** [`sam3.1-tensorrt.md`](sam3.1-tensorrt.md)
- **Run order / quickstart:** `python/README.md`
- **Native repo pin:** `python/third_party/README.md`
- **Contract source of truth:** `python/reusex_sam3/__init__.py` (`ENGINE_IO_CONTRACT`, `TRACKER_META`)
- **The fixes:** `python/reusex_sam3/fixes.py`
- **C++ consumer:** `libs/reusex/src/vision/tensor_rt/Sam3p1.cpp`
