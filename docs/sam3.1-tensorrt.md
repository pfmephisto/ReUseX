<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# SAM 3.1 → ONNX → TensorRT, taken apart

A teaching write-up of how Meta's SAM 3.1 (`facebook/sam3.1`,
`sam3.1_multiplex.pt`) was decomposed, exported to ONNX, converted to TensorRT
engines, and wired into the ReUseX C++ pipeline — including the new stateful
video-tracker path.

**Audience.** A strong C++/graphics engineer who is comfortable with GPUs and
memory layouts but is *not* an ML-export specialist. Wherever a piece of ML
jargon appears, it is unpacked. Every claim is grounded in a specific file in
this repo; paths are given so you can read the source alongside.

**How the pieces fit (one-paragraph map).** A Python package,
`python/reusex_sam3/`, builds the real SAM 3.1 model on CPU, loads the weights,
and traces small *functional* sub-graphs of it to ONNX; `trtexec` turns each
ONNX file into a TensorRT `.engine`. A shared **engine-I/O contract**
(`python/reusex_sam3/__init__.py`, `ENGINE_IO_CONTRACT`) fixes the exact tensor
names and shapes at the boundary. On the C++ side,
`libs/reusex/src/vision/tensor_rt/Sam3p1.cpp` binds to those names and runs the
engines per frame, maintaining a GPU memory bank so detections are stabilised
across a video sequence.

---

## 1. Background: SAM 3, and what SAM 3.1 changed

### SAM 3 in this project

SAM 3 is an **open-vocabulary, promptable segmentation** model. "Open
vocabulary" means you are not restricted to a fixed class list: you hand it a
*concept prompt* — a short piece of text like `"chair"` or `"radiator"`, or a
box/point geometry prompt — and it returns instance masks for everything in the
image matching that concept. ReUseX uses this to annotate building-interior
scans with semantic labels without training a bespoke classifier.

In this codebase SAM 3 is already integrated as a **stateless** TensorRT model,
`TensorRTSam3` (`libs/reusex/src/vision/tensor_rt/Sam3.cpp`). Rather than
exporting the whole network as one monolithic graph, it is **split into four
engines**, each a pure function:

| engine | role |
|--------|------|
| `vision-encoder` | image → multi-scale feature pyramid (FPN) |
| `text-encoder` | tokenised concept text → per-token features |
| `geometry-encoder` | box/point prompts + top-level features → geometry tokens |
| `decoder` | features + prompt tokens → masks, boxes, scores |

This four-way split is not arbitrary. Each piece is called with a different
cadence: the vision encoder runs once per image; the text encoder can be cached
per concept string; the decoder runs per (image, prompt) pair. Splitting lets
the C++ side batch and cache each stage independently, and — crucially for
export — lets each stage be a **stateless, single-purpose function** that ONNX
can represent. The public reference this mirrors is
[`leon0514/trt-sam3`](https://github.com/leon0514/trt-sam3)
(`workspace/script/export.py`), which exports the same four engine names with
the same input/output tensor names (`images`; `fpn_feat_0/1/2`, `fpn_pos_2`;
`input_ids`, `attention_mask`; `pred_masks`, `pred_boxes`, …).

### What SAM 3.1 changed

SAM 3.1 is an incremental release over SAM 3. Its changes — an **Object
Multiplex** mechanism and memory optimisations — are confined to the **video
tracker**: the part of the model that carries information *between frames* so an
object segmented in frame *t* stays consistently segmented in frame *t+1*. The
checkpoint used here is the merged `sam3.1_multiplex.pt` from `facebook/sam3.1`.

The important architectural consequence for us:

> **The image detector path in SAM 3.1 is architecturally identical to SAM 3.**
> The four detector engines (`vision-encoder`, `text-encoder`,
> `geometry-encoder`, `decoder`) re-export **1:1** — same modules, same tensor
> names, same shapes. Only the *tracker* is new.

So the SAM 3.1 pipeline is: **the four existing detector engines, plus four new
tracker engines.** The tracker engines are:

| engine | role |
|--------|------|
| `tracker-prompt-encoder` | click/box/mask prompts → sparse + dense embeddings |
| `tracker-memory-encoder` | current features + predicted mask → a memory token |
| `tracker-memory-attention` | current features conditioned on the memory bank |
| `tracker-multiplex-decoder` | multiplex mask decode (K objects per bucket) |

Object Multiplex, concretely, is why several tracker tensors carry a leading
`K = 16` object dimension (`multiplex_count`, see `TRACKER_META`): the decoder
produces multiple candidate objects per bucket in one pass instead of looping.

---

## 2. Why you can't just `torch.onnx.export` the whole model

The instinct is to call `torch.onnx.export(model, example_input)` and be done.
That fails here for a fundamental reason: **ONNX is a static dataflow graph**, a
DAG of tensor ops with fixed structure. The full SAM 3.1 model is none of those
things:

- **Stateful / recurrent.** The tracker keeps a *memory bank* that mutates
  across frames. There is no single "input → output" function; the output at
  frame *t* depends on hidden state accumulated from frames `0..t-1`.
- **Multi-branch with control flow.** The native tracker
  (`VideoTrackingDynamicMultiplex`) drives the whole thing with Python `if`s,
  loops, and a runtime `MultiplexState` bookkeeping object (see the docstring of
  `python/reusex_sam3/wrappers_tracker.py`). Data-dependent control flow does
  not trace to a static graph — the tracer records *one* path and silently bakes
  in whatever branch the example input happened to take.
- **CUDA-only and untraceable ops.** FlashAttention-3 is a custom CUDA kernel
  with no ONNX equivalent; complex-number RoPE, CUDA-cached positional
  encodings, and `scatter`/`roi_align`-on-lists all either fail to trace or
  produce ops TensorRT can't build (details in §4).

The solution is **decomposition into pure functional sub-graphs**. Instead of
tracing the model, we trace *thin wrappers* around its individual neural
sub-modules, each of which *is* a pure function of its tensor inputs. All the
Python control flow, the memory bank, the mux/demux bookkeeping — that logic is
re-implemented in **C++** (per-frame, explicitly), where it belongs. The engines
only ever do fixed-shape tensor math.

### The component split, and where each lives natively

`python/reusex_sam3/load_native.py` builds the real model and exposes its live
sub-modules through `NativeHandles`. The wrappers target these exact attribute
paths (ground-truth at the pinned SHA `20dba30…`):

**Detector** (`wrappers_detector.py`):

| wrapper (→ engine) | native module (`NativeHandles.*`) |
|---|---|
| `VisionEncoderWrapper` → `vision-encoder` | `detector.backbone.vision_backbone` (`Sam3TriViTDetNeck`) |
| `TextEncoderWrapper` → `text-encoder` | `detector.backbone.language_backbone` (`VETextEncoder`) |
| `GeometryEncoderWrapper` → `geometry-encoder` | `detector.geometry_encoder` (`SequenceGeometryEncoder`) |
| `DecoderWrapper` → `decoder` | `detector.transformer.{encoder,decoder}` + `segmentation_head` + `dot_prod_scoring`, driven via the detector's own `_run_encoder`/`_run_decoder`/`_run_segmentation_heads` |

**Tracker** (`wrappers_tracker.py`):

| wrapper (→ engine) | native module (`NativeHandles.*`) |
|---|---|
| `PromptEncoderWrapper` → `tracker-prompt-encoder` | `tracker.sam_prompt_encoder` (`PromptEncoder`) |
| `MemoryEncoderWrapper` → `tracker-memory-encoder` | `tracker.maskmem_backbone` (`SimpleMaskEncoder`) + the sigmoid/scale/bias + `no_obj_embed` add from `_encode_new_memory` |
| `MemoryAttentionWrapper` → `tracker-memory-attention` | `tracker.transformer.encoder` (`TransformerEncoderDecoupledCrossAttention`) |
| `MultiplexDecoderWrapper` → `tracker-multiplex-decoder` | `tracker.sam_mask_decoder` (`MultiplexMaskDecoder`) |

Each wrapper's `forward()` deliberately returns tensors in the exact order of
the contract's `output_names`. The wrappers also translate between the model's
internal **seq-first** `[Seq, Batch, Channels]` convention and the **batch-first**
`[Batch, …]` convention we expose at the engine boundary (a `.transpose(0, 1)`
on the way in/out — see e.g. `TextEncoderWrapper`, `GeometryEncoderWrapper`).

---

## 3. The checkpoint-remap gotcha

Before anything can be traced, the weights must load into the freshly-built
model. This is the first non-obvious hurdle.

`sam3.1_multiplex.pt` **cannot be loaded by HuggingFace `transformers`** — it is
a raw native checkpoint, not an HF-format model. It only loads via the native
`sam3` builder. But we cannot use the convenient one-shot builder
`build_sam3_multiplex_video_predictor` either, because it (a) calls `.cuda()`
unconditionally (we need CPU tracing) and (b) wraps everything in a
request/response *predictor* that hides the bare `nn.Module`s we need to reach.

So `load_native.build_demo_model()` **reproduces the model assembly
component-for-component** on CPU (mirroring the native builder up to — but not
including — checkpoint load / `.cuda()` / predictor wrapping), and returns the
bare `Sam3MultiplexTrackingWithInteractivity` with its `.detector` and
`.tracker`. It is built with `use_fa3=False` (no FlashAttention-3) and
`use_rope_real=True` (real-valued RoPE) — the export-friendly combination (§4).

### Why `strict=False` alone is dangerous

The naive way to tolerate key-name drift between checkpoint and model is
`load_state_dict(strict=False)`, which simply **ignores** missing and unexpected
keys. That is dangerous: a systematic prefix mismatch (say every `tracker.` key
silently unmatched) would load a model that is *mostly random weights* and still
"succeed". You'd only notice when the masks came out garbage.

`load_native.py` instead does a **diff-driven, bijective remap** and then
**hard-asserts**:

1. **Static prefix remaps** — older checkpoints use internal prefixes
   (`sam3_model.` → `detector.`, `sam2_predictor.` → `tracker.`); apply these
   first (`_PREFIX_REMAPS`).
2. **Regex transforms, kept only if they land** — optional patterns
   (`_REGEX_TRANSFORMS`) for residual naming drift (e.g. fused attention proj
   names) are applied *only when the transformed key actually exists in the
   model*. This keeps the remap **bijective**: it can never invent a parameter
   name the model doesn't have (`remap_state_dict`).
3. **Load, then hard-assert the residual.** After
   `load_state_dict(strict=False)`, every remaining missing/unexpected key must
   be covered by an explicit allowlist (`_ALLOWLIST_PREFIXES` — buffers we
   re-register ourselves like `freqs_cis`, plus known-unused sub-trees like the
   click-interactivity heads). If anything else is left over,
   `load_checkpoint_into` **raises** (`hard_assert=True`).

Every run writes `checkpoints/remap_report.json` with counts and the full
residual lists, so when the real weights first land you can finalise the remap
by reading exactly what drifted. `inspect_checkpoint.py` is the standalone,
model-free companion: `--no-model` dumps just the checkpoint's key prefixes
(works without torch/`sam3` installed); the full run builds the empty model and
prints the same diff, so you can extend the regex/allowlist tables before
`export`.

> **Open risk (tracked).** The remap logic is written against the *expected*
> key structure and has not yet been validated against the real gated weights.
> The hard-assert is precisely the guard that will flag any surprise on first
> download — see §8.

---

## 4. The layers that block ONNX export, and the exact fix for each

This is the heart of the write-up. Every fix lives in
`python/reusex_sam3/fixes.py`, applied as a context manager (`apply_all()`)
around the `torch.onnx.export` call and **restored on exit** so the parity check
in `verify.py` compares against the *un-patched* model.

First, the export settings that frame these fixes (`export_detector.py`,
`export_tracker.py`):

- **opset 17** — the ONNX operator-set version. Fixes below target ops available
  and well-behaved at opset 17.
- **TorchScript exporter (`dynamo=False`)** — the classic tracing exporter, not
  the newer dynamo-based one. It traces a single execution and records the ops
  it sees; anything data-dependent or non-tensor is baked in or breaks.
- `do_constant_folding=True` — fold constant subexpressions at export time.

With that context, here are the five blockers.

### 4.1 Complex-valued RoPE → real-valued RoPE

**What the op is.** RoPE (Rotary Position Embedding) injects token positions
into attention by *rotating* query/key vectors. A slick way to implement a 2-D
rotation is with complex multiplication: pack pairs of channels as complex
numbers and multiply by `e^{iθ}`.

**Why it chokes.** ONNX opset 17 has essentially no support for complex tensors
(`torch.complex`, `torch.view_as_complex`, complex `mul`). The tracer either
errors or emits ops TensorRT cannot build. Additionally, the native
`SimpleRoPEAttention.forward` *recomputes* the rotation table (`freqs_cis`) and
**mutates `self.freqs_cis`** when the sequence length changes — an in-place state
mutation the tracer records but a static graph can't honour.

**The fix.** Two parts, working together:

- At build time, `use_rope_real=True` selects a **real-valued** RoPE formulation
  (plain `sin`/`cos` real arithmetic instead of complex multiply), with the
  real/imag rotation buffers pre-registered for the fixed 72×72 grid.
- At export time, `_patch_simple_rope` (fix 2) **freezes**
  `SimpleRoPEAttention.forward`: it reads the pre-registered `freqs_cis_real` /
  `freqs_cis_imag` buffers directly and **never mutates state**, so the trace is
  a clean static graph. For a fixed 5184-token grid the dynamic recompute branch
  is never taken at runtime anyway; freezing just prevents the tracer from
  recording the mutation.

*Before:* rotate via complex `e^{iθ}`, recompute + reassign `self.freqs_cis`.
*After:* rotate via real sin/cos on frozen, pre-registered buffers.

### 4.2 CUDA-cached / cumsum sine positional encodings → arange recompute

**What the op is.** `PositionEmbeddingSine` produces the classic
Transformer sinusoidal position grid added to features.

**Why it chokes.** The native implementation (a) **caches** the computed grid in
a `{shape: tensor}` dict where the cached tensor lives on **`cuda`**, and (b) in
some code paths builds positions with `cumsum`. A cached CUDA tensor traced on
CPU is a device/`untraceable` landmine — the tracer would embed a constant tied
to the wrong device, and repeating a cached tensor hides the real computation
from the graph. (The public `trt-sam3` reference notes the same "arange instead
of cumsum" fix.)

**The fix.** `_patch_position_encoding` (fix 1) **empties the cache** for the
duration of the trace, forcing the already-`arange`-based recompute to run on the
*input's own device*. The result is a deterministic, device-agnostic static
subgraph; the original cache is restored on exit.

*Before:* return a repeated, CUDA-resident cached grid.
*After:* recompute the grid from `arange` on `x.device` every call, cache-free.

### 4.3 Geometry-encoder `scatter` + data-dependent assert → cat-based right-padded reimpl

**What the op is.** `concat_padded_sequences` in the geometry encoder joins two
padded token sequences (text tokens + geometry tokens) into one padded sequence,
computing the merged padding mask.

**Why it chokes.** It uses `torch._assert_async` — a **data-dependent assertion**
(its truth depends on runtime tensor values, which a static graph can't
evaluate) — and `Tensor.scatter` to place tokens, which the tracer represents
awkwardly and TensorRT dislikes.

**The fix.** `_patch_geometry_concat` (fix 3) swaps in `concat_padded_right`, a
**scatter-free, assert-free** reimplementation that is a plain `torch.cat` of the
two sequences and a `cat` of the two masks. This is valid **because our export
scenario feeds fixed-length, already-right-padded prompts** — the general
scatter machinery only exists to handle arbitrary interleaved padding we never
produce.

*Before:* `scatter` tokens into place, guarded by a runtime `_assert_async`.
*After:* `cat([seq1, seq2])`, `cat([mask1, mask2])` — a right-padded concat.

### 4.4 `roi_align` list / `pin_memory` → single `[K,5]` rois tensor

**What the op is.** `roi_align` crops and resamples a fixed-size feature patch
for each prompt box out of the 72×72 feature map (region-of-interest pooling).

**Why it chokes.** The native `_encode_boxes` (a) feeds `roi_align` a **Python
list** of per-batch box tensors via `.unbind(0)` — a list input the ONNX
exporter can't express as a single op — and (b) calls `scale.pin_memory()`,
which is a **CUDA-only** host-memory operation with no meaning in a graph.

**The fix.** `_patch_geometry_roi_align` (fix 4) rebuilds the call the way
torchvision's traceable `roi_align` wants it: a **single `[K, 5]` rois tensor**
whose first column is the batch index and the remaining four are `x1,y1,x2,y2`
(built by concatenating an `arange`-derived batch-index column onto the flattened
absolute-pixel boxes). `pin_memory()` is dropped.

*Before:* `roi_align(feats, [boxes_b0, boxes_b1, …])`, with `pin_memory`.
*After:* `roi_align(feats, rois[K,5])` where column 0 = batch index; no
`pin_memory`.

### 4.5 fp16 additive attention masks: clamp `-inf` → `-1e4`

**What the op is.** Padding in attention is applied as an **additive mask**: add
`0` to valid positions and a very negative number to padded positions before the
softmax, so padded keys get ~zero weight.

**Why it chokes.** The usual "very negative number" is `-inf`. In **fp16**,
`-inf` arithmetic inside TensorRT's fused attention kernels can produce **NaNs**
(e.g. `-inf + inf`, or `-inf` propagating through a fused reduction), which then
poison the whole output.

**The fix.** `additive_mask_from_bool` builds the additive mask by
`masked_fill(bool_mask, -1e4)` — a **finite fp16-safe sentinel** (`NEG_INF_FP16
= -1.0e4`) instead of `-inf`. `-1e4` is small enough that `exp(-1e4) ≈ 0` in the
softmax (the padded key still gets ~zero weight) but large enough not to
overflow fp16. `build_engines.FP16_MASK_CLAMP_ENGINES = {"tracker-memory-attention",
"decoder"}` flags the engines where this matters most.

> Nuance for the tracker: the exported `tracker-memory-attention` graph carries
> **no** mask-add at all — the decoupled RoPE encoder relies on RoPE + fixed
> lengths and does not consume a key-padding mask (see §6). The clamp is a
> safety net that only bites if a future variant reintroduces a float mask.

*Before:* `masked_fill(pad, float('-inf'))` → fp16 NaNs.
*After:* `masked_fill(pad, -1e4)` → finite, softmax-negligible.

---

## 5. ONNX → TensorRT

`python/reusex_sam3/build_engines.py` shells out to **`trtexec`** (NVIDIA's
command-line engine builder), one invocation per engine. TensorRT reads the ONNX
graph, picks fast kernels for the target GPU, and serialises a `.engine`.

### FP16 by default

Every engine builds with `--fp16` by default (half precision). This roughly
halves memory bandwidth and lights up tensor cores; the fp16-mask clamp (§4.5)
is what keeps fp16 numerically safe.

### Dynamic shapes (min / opt / max)

Several inputs vary at runtime — batch size, number of prompt boxes `N`, prompt
length `L`, memory-bank length `M`. For each such input TensorRT needs an
**optimisation profile**: a `(min, opt, max)` triple. `build_engines.py` derives
these from the contract (`SHAPE_PROFILES`), e.g.:

- `vision-encoder`: `images` batch `1 → 1 → 4`.
- `decoder`: `prompt_features` length `1 → 40 → 300` (200 queries + text/geo).
- `geometry-encoder`: `input_boxes` `N` from `1 → 8 → 64`.
- `tracker-memory-attention`: `memory` length from one frame's worth
  (`5184`) up to the full bank (`5184 × 7`).

`opt` is the shape TensorRT tunes kernels for; `min`/`max` bound what's legal.
`_fmt` formats these into `--minShapes/--optShapes/--maxShapes` strings.
`--memPoolSize=workspace:8192` gives the builder scratch space.

### Optional INT8 PTQ for the vision encoder

The ViT-L vision encoder at 1008×1008 dominates latency, so it's the top INT8
target. `build_engines.py --int8-vision` expects a Q/DQ-annotated ONNX
(`vision-encoder.int8.onnx`) and adds `--int8` (keeping `--fp16` fallback for
un-quantised layers). Producing that Q/DQ ONNX is
`ptq_vision.py`'s job — currently a **documented stub** that raises until a
calibration set (~200–500 representative frames) is wired into NVIDIA
Model-Optimizer's ONNX PTQ. The module docstring carries the exact recipe
(`modelopt.onnx.quantization.quantize`, entropy calibration, exclude
LayerNorm/Softmax/RoPE) and an accuracy guardrail (target cosine ≥ 0.99, mask
IoU ≥ 0.95; fall back to fp16 if the first/last blocks regress).

### The fp16 mask-clamp caveat for memory-attention

As noted in §4.5, `tracker-memory-attention` and `decoder` are in
`FP16_MASK_CLAMP_ENGINES`. Keep the `-1e4` clamp in place for those if a float
mask is ever reintroduced; the current memory-attention graph is mask-free.

### Verifying the engines

`verify.py` is a **3-way parity harness**: for each engine it runs the PyTorch
reference (un-patched model), onnxruntime (the `.onnx`), and TensorRT (the
`.engine`) on the *same* fixed random fixture and compares. Feature engines are
compared by **cosine similarity** (`≥ 0.999`); mask engines by **IoU**
(`≥ 0.98`, binarised at logit > 0). Each backend is guarded — it runs only if its
deps/artifacts are present — so the harness degrades cleanly on a box with only
some of torch/onnxruntime/TensorRT. `verify_tracker_drift` additionally runs the
memory-encoder → memory-attention loop for 30 frames to exercise the recurrent
coupling and catch accumulated numerical drift.

---

## 6. The C++ side

### The detector path is unchanged; SAM 3 coexists

`TensorRTSam3p1` (`libs/reusex/src/vision/tensor_rt/Sam3p1.cpp`) replicates the
batch-1 detector path from `Sam3.cpp` almost line-for-line
(`preprocess` → `encode_image` → `encode_text` → `decode` → `postprocess`), with
identical engine binding names and the same `sam3_postprocess_plane` CUDA kernel.
`TensorRTSam3` (stateless SAM 3) and `TensorRTSam3p1` (stateful SAM 3.1)
**coexist**: the factory `TensorRTSam3p1::create` selects the tracker path only
when the model directory *additionally* contains `tracker-memory-encoder.engine`
and `tracker-memory-attention.engine` (both required); `geometry-encoder` is
optional and deliberately unused on the video path (text prompts only).

### The stateful interface: `IVideoModel`

`libs/reusex/include/vision/IVideoModel.hpp` defines the contract that makes
statefulness explicit and hard to misuse. It is intentionally distinct from the
stateless `IModel`:

- `reset()` — clear the memory bank at a **sequence boundary** (before the first
  frame of a new scan).
- `step(pair)` — process one frame, conditioned on all memory accumulated since
  the last `reset()`. Input `.first` is a `TensorRTData` (image + prompts +
  threshold); output `.first` is the `CV_32S` per-pixel label image (background
  `-1`, class ids `0+`) exactly like `TensorRTSam3::forward`; `.second` echoes
  the sample id.

The header states the caller's obligations bluntly: feed frames **in strict
temporal order**, and **never** drive an `IVideoModel` from the shuffled,
multi-worker `Dataloader` — out-of-order frames produce meaningless results
because each frame conditions on the previous ones.

### The per-frame control flow (`step()`)

The Python control flow that ONNX couldn't represent (§2) lives here, explicit
and per-frame. For each frame `step()` does:

1. **Vision-encode** (`encode_image`) → raw `fpn_feat_0/1/2` + `fpn_pos_2`
   (batch fixed to 1).
2. **Memory-attention** (`apply_memory_attention`) conditions **`fpn_feat_2`**
   (the 72×72 top level) on the memory bank, overwriting it in place. This is a
   **no-op on the first frame** (empty bank). Conditioning happens at the 72×72
   level, *not* at `fpn_feat_0`; `fpn_feat_0/1` and `fpn_pos_2` pass through to
   the decoder unchanged.
3. **Detect every frame** — text-encode + decode + postprocess run on the
   conditioned features for every concept prompt. The detector runs *every*
   frame for open-vocabulary all-classes annotation; the memory bank only
   *stabilises* the features across time, it does not replace detection.
4. **Memory-encode** (`build_aggregate_mask` + `append_memory`): build this
   frame's aggregate foreground mask from the per-object detection masks, run
   `tracker-memory-encoder` to produce a maskmem token, and push it into the
   ring (evicting the oldest).

### The GPU memory-bank ring buffer

The bank is a fixed ring of `mem_bank_max_ = 7` pre-allocated slots
(`mem_slots_`), each holding one frame's memory feature + positional encoding.
`mem_valid_` is a `std::deque<int>` of valid slot indices in chronological order;
`append_memory` rotates the newest slot in and `pop_front`s the oldest when full.
Everything is pre-allocated once in `allocate_memory_once()` — no per-frame
`cudaMalloc`.

`tracker-memory-encoder` ingests `vision_feat = fpn_feat_2` (the
*memory-conditioned* feature — mirroring SAM 2's `_encode_new_memory`, which
encodes memory from the conditioned `pix_feat`), the aggregate foreground mask,
and `object_score_logits` (the frame's max detection score). It emits a spatial
`[C,H,W]` maskmem token.

### The `[C,H,W] ↔ [HW,C]` rearrange, and exact-length packing

Two different tensor **layouts** collide at the memory boundary:

- The vision/memory encoders emit **spatial** `[C, H, W]` (channel-planar).
- `tracker-memory-attention` consumes **seq-major tokens** `[H*W, C]` (each
  spatial location is one token).

`rearrange_chw_to_hwc()` converts between them. Notably it does a **host
round-trip** (D2H → CPU transpose → H2D) because no transpose kernel exists in
this module and the task forbade adding one; the element count (`C·H·W ≈ 1.3M`
floats) makes this cheap enough for the per-frame path.

Memory tokens are stored **seq-major in the ring**, so `pack_memory()` is a
plain contiguous D2D copy of the valid slots into the `[M, 1, C]` `memory`
tensor. And critically:

> The memory is packed **exact-length** — exactly the valid slots, contiguously,
> with an all-true `memory_mask`. This is safe because **the decoupled RoPE
> encoder ignores the padding mask** (`MemoryAttentionWrapper` docstring: it
> relies on RoPE + fixed lengths, not a key-padding mask). So there is no padding
> to mask out; `memory_mask` is bound only for API completeness. `set_run_dims`
> tells TensorRT the actual `M = num_valid_slots · 5184` at each call, within the
> dynamic profile from §5.

The memory-attention output `pix_feat_with_mem` (spatial `[1,C,72,72]`)
overwrites `fpn_feat_2_` in place, so the decoder transparently consumes the
conditioned feature.

### How `annotate --video` feeds frames

`annotate_video` (`libs/reusex/src/vision/annotate.cpp`) is the ordered driver.
It **bypasses the shuffled batch `Dataloader`** and instead loops the dataset
indices in order, single-threaded, calling `step()` one frame at a time and
saving each result. It calls `reset()` once at the first frame (the dataset is a
single ordered scan; multi-sequence boundary detection is a tracked TODO). The
frames arrive in **node-id order** (the dataset's natural index order = SLAM node
order), which is the temporal order the tracker requires.

The CLI (`apps/rux/src/create/annotate.cpp`) routes to this path via `--video`,
or **implicitly** when a SAM 3.1 model directory is detected. In video mode
`--shuffle`, `--batch-size`, and `--workers` are force-disabled with warnings —
the tracker must run ordered and single-threaded.

---

## 7. The authoritative engine I/O contract

This table is the source of truth reproduced from
`python/reusex_sam3/__init__.py` (`ENGINE_IO_CONTRACT`). The C++ side binds to
these exact tensor names. Shapes use symbolic dims for dynamic axes: `B` batch,
`N` boxes, `L` prompt length, `M` memory length, `K` objects, `P` points,
`S` sparse. All float tensors are fp32 in ONNX (TensorRT builds fp16); mask
tensors are `bool`; token-id tensors are `int64` (opset-17 `Gather` wants int64
indices).

### Detector engines

**`vision-encoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `images` | `[B, 3, 1008, 1008]` | float32 |
| out | `fpn_feat_0` | `[B, 256, 288, 288]` | float32 |
| out | `fpn_feat_1` | `[B, 256, 144, 144]` | float32 |
| out | `fpn_feat_2` | `[B, 256, 72, 72]` | float32 |
| out | `fpn_pos_2` | `[B, 256, 72, 72]` | float32 |

Dynamic axis: `batch` on all (axis 0).

**`text-encoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `input_ids` | `[B, 32]` | int64 |
| in  | `attention_mask` | `[B, 32]` | int64 |
| out | `text_features` | `[B, 32, 256]` | float32 |
| out | `text_mask` | `[B, 32]` | bool |

Dynamic axis: `batch` (axis 0). (Seq-first internally; exported batch-first.)

**`geometry-encoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `input_boxes` | `[B, N, 4]` (cxcywh, normalised) | float32 |
| in  | `input_boxes_labels` | `[B, N]` | int64 |
| in  | `fpn_feat_2` | `[B, 256, 72, 72]` | float32 |
| in  | `fpn_pos_2` | `[B, 256, 72, 72]` | float32 |
| out | `geometry_features` | `[B, N+1, 256]` (N boxes + 1 CLS) | float32 |
| out | `geometry_mask` | `[B, N+1]` | bool |

Dynamic axes: `batch` (axis 0); `num_boxes`/`num_prompts` (axis 1).

**`decoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `fpn_feat_0` | `[B, 256, 288, 288]` | float32 |
| in  | `fpn_feat_1` | `[B, 256, 144, 144]` | float32 |
| in  | `fpn_feat_2` | `[B, 256, 72, 72]` | float32 |
| in  | `fpn_pos_2` | `[B, 256, 72, 72]` | float32 |
| in  | `prompt_features` | `[B, L, 256]` (text ⊕ geometry, right-padded) | float32 |
| in  | `prompt_mask` | `[B, L]` (True == pad) | bool |
| out | `pred_masks` | `[B, 200, 288, 288]` | float32 |
| out | `pred_boxes` | `[B, 200, 4]` (cxcywh) | float32 |
| out | `pred_logits` | `[B, 200, 1]` | float32 |
| out | `presence_logits` | `[B, 1]` | float32 |

Dynamic axes: `batch` (axis 0); `prompt_len` (axis 1) on `prompt_features`/`prompt_mask`.

### Tracker engines

**`tracker-memory-encoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `vision_feat` | `[B, 256, 72, 72]` | float32 |
| in  | `pred_mask` | `[K, 1, 1008, 1008]` (per-object logits) | float32 |
| in  | `object_score_logits` | `[K, 1]` | float32 |
| out | `maskmem_features` | `[B, 256, 72, 72]` | float32 |
| out | `maskmem_pos_enc` | `[B, 256, 72, 72]` | float32 |

Dynamic axes: `batch` (axis 0) on B-tensors; `num_objects` (axis 0) on `pred_mask`/`object_score_logits`.

**`tracker-memory-attention`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `current_feat` | `[5184, B, 256]` (seq-first `[HW,B,C]`) | float32 |
| in  | `current_pos` | `[5184, B, 256]` | float32 |
| in  | `memory` | `[M, B, 256]` | float32 |
| in  | `memory_pos` | `[M, B, 256]` | float32 |
| in  | `memory_mask` | `[B, M]` (bool; ignored by the RoPE encoder) | bool |
| out | `pix_feat_with_mem` | `[B, 256, 72, 72]` | float32 |

Dynamic axes: `batch` (axis 1 on seq-first tensors, axis 0 on `memory_mask`/output); `mem_len` (axis 0 on memory, axis 1 on `memory_mask`).

**`tracker-prompt-encoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `point_coords` | `[B, P, 2]` (absolute px x,y) | float32 |
| in  | `point_labels` | `[B, P]` | int64 |
| in  | `boxes` | `[B, 4]` (xyxy absolute px) | float32 |
| in  | `mask_input` | `[B, 1, 288, 288]` | float32 |
| out | `sparse_embeddings` | `[B, S, 256]` | float32 |
| out | `dense_embeddings` | `[B, 256, 72, 72]` | float32 |

Dynamic axes: `batch` (axis 0); `num_points` (axis 1); `num_sparse` (axis 1 on output).

**`tracker-multiplex-decoder`**

| dir | name | shape | dtype |
|---|---|---|---|
| in  | `image_embeddings` | `[B, 256, 72, 72]` | float32 |
| in  | `image_pe` | `[1, 256, 72, 72]` | float32 |
| in  | `high_res_feat_0` | `[B, 32, 288, 288]` | float32 |
| in  | `high_res_feat_1` | `[B, 64, 144, 144]` | float32 |
| in  | `extra_per_object_embeddings` | `[B, 16, 256]` | float32 |
| out | `masks` | `[B, 16, 3, 288, 288]` (multimask=3 per object) | float32 |
| out | `iou_pred` | `[B, 16, 3]` | float32 |
| out | `object_score_logits` | `[B, 16, 1]` | float32 |
| out | `sam_tokens_out` | `[B, 16, 3, 256]` | float32 |

Dynamic axis: `batch` (axis 0).

### Memory-bank metadata (`TRACKER_META`)

Emitted verbatim as `tracker-meta.json`; the C++ side reads it to size the bank
(engine-probed shapes take precedence, this is fallback/cross-check):

| key | value | meaning |
|---|---|---|
| `mem_bank_max` | 7 | 1 conditioning frame + 6 recent frames |
| `mem_tokens_per_frame` | 5184 | 72 × 72 spatial memory tokens per frame |
| `mem_dim` | 256 | memory token channel dim |
| `feat_c` / `feat_h` / `feat_w` | 256 / 72 / 72 | top-level vision feature grid |
| `multiplex_count` | 16 | objects per multiplex bucket |

---

## 8. Running it end to end, and known open risks

### End to end

Python side (see `python/README.md` and `python/Makefile`):

```bash
make -C python download          # gated HF pull of sam3.1_multiplex.pt (+ bpe)
make -C python load              # inspect + write remap_report.json
make -C python export-detector   # 4 detector ONNX graphs
make -C python export-tracker    # 4 tracker ONNX graphs + tracker-meta.json
make -C python engines           # trtexec → 8 .engine files (FP16, dynamic shapes)
make -C python verify            # torch vs onnxruntime vs tensorrt parity
# or: make -C python all
```

C++ side — drop the eight `.engine` files, `tokenizer.json`, and
`tracker-meta.json` into one directory and run:

```bash
rux annotate <project.rux> --net <model_dir> --video
# --video is implicit when a SAM 3.1 directory is detected; it forces ordered,
# single-threaded processing (shuffle/batch/workers are disabled).
```

### Known open risks / TODOs

These are real, tracked uncertainties — the pipeline is structurally complete but
several points can only be finalised against the real gated weights:

1. **The checkpoint remap is unvalidated on real weights.** `load_native.py`'s
   diff-driven remap + hard-assert is a *guard*, not a *proof*. On first
   download, `make load` may assert; use `inspect_checkpoint.py` to read the
   residual and extend `_REGEX_TRANSFORMS` / `_ALLOWLIST_PREFIXES`.

2. **Memory-encoder logit scale + per-object-vs-aggregate.** The tracked
   `TODO` in `Sam3p1.cpp::step` (category=Vision, estimate=4h) has two unknowns:
   (a) the exact logit scale the memory encoder's `sigmoid*scale+bias` expects —
   `build_aggregate_mask` approximates the binary foreground union as ±10 logits
   (`kLogit`); and (b) whether open-vocab annotation should run a **per-object**
   ring (`K > 1` `pred_mask` channels) instead of the **K=1 aggregate**
   foreground mask used here. The current design packs all detected objects into
   one union mask per frame.

3. **Memory-attention single-bank deviation.** The native encoder decouples
   memory into an *image* stream (spatial maskmem) and an *object* stream (object
   pointers), fused by two separate cross-attentions. The shared contract exposes
   a **single flat `memory` bank**: `MemoryAttentionWrapper` feeds that same bank
   to both streams and passes zero object-pointer tokens
   (`num_obj_ptr_tokens=0`). This matches the common spatial-memory-only case but
   is an explicit, documented deviation from the full native behaviour.

4. **Parity thresholds.** `verify.py` asserts cosine ≥ 0.999 (features) / IoU ≥
   0.98 (masks) / IoU ≥ 0.9 (30-frame drift). INT8 will not hit 0.999 — the
   `ptq_vision.py` guardrail targets cosine ≥ 0.99 / IoU ≥ 0.95 and prescribes
   excluding sensitive blocks or falling back to fp16 if accuracy regresses.

---

## File map (where to read next)

| topic | file |
|---|---|
| Engine I/O contract + memory metadata | `python/reusex_sam3/__init__.py` |
| Build model on CPU + checkpoint remap | `python/reusex_sam3/load_native.py` |
| Standalone checkpoint inspector | `python/reusex_sam3/inspect_checkpoint.py` |
| ONNX-export monkeypatches (the 5 fixes) | `python/reusex_sam3/fixes.py` |
| Detector engine wrappers | `python/reusex_sam3/wrappers_detector.py` |
| Tracker engine wrappers | `python/reusex_sam3/wrappers_tracker.py` |
| ONNX export drivers | `python/reusex_sam3/export_{detector,tracker}.py` |
| trtexec driver (fp16, dynamic shapes, int8 stub) | `python/reusex_sam3/build_engines.py` |
| INT8 vision PTQ recipe (stub) | `python/reusex_sam3/ptq_vision.py` |
| 3-way parity + drift harness | `python/reusex_sam3/verify.py` |
| Run order + quickstart | `python/README.md` |
| Native repo pin | `python/third_party/README.md` |
| Stateful video interface | `libs/reusex/include/vision/IVideoModel.hpp` |
| C++ tracker (header) | `libs/reusex/include/vision/tensor_rt/Sam3p1.hpp` |
| C++ tracker (impl) | `libs/reusex/src/vision/tensor_rt/Sam3p1.cpp` |
| SAM 3 detector reference (C++) | `libs/reusex/src/vision/tensor_rt/Sam3.cpp` |
| Ordered video annotation driver | `libs/reusex/src/vision/annotate.cpp` |
