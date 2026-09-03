# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
ONNX-export monkeypatches / utilities for the native SAM 3.1 modules.

Every fix here is grounded in a specific native code path (file:line references
in each docstring) that is either untraceable, CUDA-only, or produces an
unsupported ONNX op. Apply them with ``apply_all(handles)`` (a context manager)
right before ``torch.onnx.export``; they restore the originals on exit so the
verify.py PyTorch-reference path can run against the *un-patched* model.

Summary of blockers → fixes
---------------------------
1. Sine pos-enc caches on ``cuda`` and repeats a cached tensor
   (``position_encoding.py:40-58, 97-132``). Fix: clear the cache and force the
   arange-based recompute on the input's own device (already arange-based, just
   cache-free) so it traces to a static graph on CPU.
2. RoPE precompute buffers created on ``cuda`` and re-derived in-forward when the
   sequence length changes (``decoder.py:1048-1074``, ``vitdet.py:549-552``).
   Fix: with ``use_rope_real=True`` the real/imag buffers already exist for the
   fixed 72x72 grid; we freeze ``SimpleRoPEAttention.forward`` to skip the
   dynamic ``compute_cis`` branch and never mutate ``self.freqs_cis``.
3. ``concat_padded_sequences`` uses ``torch._assert_async`` (data-dependent) and
   ``Tensor.scatter`` (``geometry_encoders.py:23-80``). Fix: a right-padded,
   scatter-free reimplementation (plain ``cat``) valid for our fixed-length,
   already-right-padded prompts.
4. ``roi_align`` is fed a *list* of per-batch box tensors via ``.unbind(0)``
   (``geometry_encoders.py:663-665``); ONNX/TensorRT want a single ``[K,5]``
   rois tensor with a leading batch-index column. Also ``scale.pin_memory()``
   (``:660``) is CUDA-only. Fix: patch ``_encode_boxes`` to build
   ``[K,5]`` rois and drop ``pin_memory``.
5. fp16 attention masks: additive float masks must be clamped to a finite
   sentinel (-1e4) instead of ``-inf`` so fp16 TensorRT kernels don't produce
   NaNs. Provided as a helper ``additive_mask_from_bool``.
"""

from __future__ import annotations

import contextlib
from typing import List

NEG_INF_FP16 = -1.0e4  # finite sentinel for fp16-safe additive masks


# ---------------------------------------------------------------------------
# helpers usable by the wrappers directly
# ---------------------------------------------------------------------------


def additive_mask_from_bool(bool_mask, dtype):
    """Convert a boolean padding mask (True == pad) to an additive float mask
    clamped to a finite fp16-safe sentinel."""
    import torch

    add = torch.zeros_like(bool_mask, dtype=dtype)
    return add.masked_fill(bool_mask, NEG_INF_FP16)


def concat_padded_right(seq1, mask1, seq2, mask2):
    """Scatter-free replacement for
    ``sam3.model.geometry_encoders.concat_padded_sequences`` valid when both
    inputs are already right-padded and we only need a plain concatenation for a
    *fixed* number of prompts (the export scenario).

    seq* : [S*, B, C]   mask* : [B, S*]  (True == pad, pytorch convention)
    returns (cat_seq [S1+S2, B, C], cat_mask [B, S1+S2])
    """
    import torch

    cat_seq = torch.cat([seq1, seq2], dim=0)
    cat_mask = torch.cat([mask1, mask2], dim=1)
    return cat_seq, cat_mask


# ---------------------------------------------------------------------------
# individual patches
# ---------------------------------------------------------------------------


def _patch_fused_addmm_act(handles, stack: contextlib.ExitStack):
    """Fix (0): replace the fused ``addmm_act`` with a plain fp32 linear+act.

    Native ``sam3.perflib.fused.addmm_act`` (``perflib/fused.py:10``) calls
    ``torch.ops.aten._addmm_activation`` after unconditionally casting the bias,
    weight and activation to ``bfloat16``. Two problems for ONNX export:

      1. It returns a *bf16* tensor which then hits the fp32 ``fc2`` linear →
         ``RuntimeError: mat1 and mat2 must have the same dtype`` (BFloat16 vs
         Float) — even in plain eval on CPU.
      2. ``aten._addmm_activation`` is a fused perf kernel that does not lower to
         a clean ONNX subgraph.

    We replace it with a mathematically-equivalent fp32 ``Linear`` + GELU/ReLU
    (the fused op's ``use_gelu`` maps to ``F.gelu``). ``vitdet.py`` binds the
    symbol at import time (``from sam3.perflib.fused import addmm_act``), so we
    must patch the name in *both* ``sam3.model.vitdet`` and ``sam3.perflib.fused``.
    """
    import torch
    import torch.nn.functional as F

    import sam3.model.vitdet as vitdet
    import sam3.perflib.fused as fused

    def addmm_act(activation, linear, mat1):
        y = F.linear(mat1, linear.weight, linear.bias)
        if activation in (torch.nn.functional.relu, torch.nn.ReLU):
            return F.relu(y)
        if activation in (torch.nn.functional.gelu, torch.nn.GELU):
            return F.gelu(y)
        raise ValueError(f"Unexpected activation {activation}")

    orig_fused = fused.addmm_act
    orig_vitdet = vitdet.addmm_act
    fused.addmm_act = addmm_act
    vitdet.addmm_act = addmm_act
    stack.callback(setattr, fused, "addmm_act", orig_fused)
    stack.callback(setattr, vitdet, "addmm_act", orig_vitdet)


def _patch_position_encoding(handles, stack: contextlib.ExitStack):
    """Fix (1): make PositionEmbeddingSine cache-free & device-agnostic."""
    from sam3.model.position_encoding import PositionEmbeddingSine

    orig_forward = PositionEmbeddingSine.forward

    def forward(self, x):
        # Bypass the {shape: cached_cuda_tensor} cache entirely; recompute on
        # x.device. The body is already arange-based (ONNX-friendly).
        saved = self.cache
        self.cache = {}  # empty → forces recompute, never repeats a cuda cache
        try:
            return orig_forward(self, x)
        finally:
            self.cache = saved

    PositionEmbeddingSine.forward = forward
    stack.callback(setattr, PositionEmbeddingSine, "forward", orig_forward)


def _patch_simple_rope(handles, stack: contextlib.ExitStack):
    """Fix (2): freeze SimpleRoPEAttention at the fixed 72x72 grid.

    The native forward reassigns ``self.freqs_cis = self.freqs_cis.to(device)``
    and recomputes it when ``freqs_cis.shape[0] != q.shape[-2]``. For a fixed
    5184-token grid this branch is never taken at runtime, but the trace still
    records the mutation. We replace forward with a variant that reads the
    pre-registered real/imag freqs directly and never mutates state.

    Device note: ``SimpleRoPEAttention.__init__`` (``decoder.py:1048-1057``)
    builds ``freqs_cis``/``freqs_cis_real``/``freqs_cis_imag`` on
    ``cuda if is_available() else None`` and stores them as *plain attributes*
    (not buffers), so ``model.cpu()`` never moves them. On our CPU export the
    cuda freqs collide with cpu q/k → ``Expected all tensors on the same
    device``. We therefore move the freqs to ``q.device`` at call time (matching
    the native ``self.freqs_cis.to(q.device)`` at ``decoder.py:1069``) without
    mutating ``self``.
    """
    from sam3.model.decoder import SimpleRoPEAttention, functional_attention

    orig_forward = SimpleRoPEAttention.forward

    def forward(self, q, k, v, num_k_exclude_rope: int = 0):
        dropout_p = 0.0  # eval
        freqs_cis = self.freqs_cis
        freqs_cis_real = getattr(self, "freqs_cis_real", None)
        freqs_cis_imag = getattr(self, "freqs_cis_imag", None)
        if freqs_cis is not None:
            freqs_cis = freqs_cis.to(q.device)
        if freqs_cis_real is not None:
            freqs_cis_real = freqs_cis_real.to(q.device)
        if freqs_cis_imag is not None:
            freqs_cis_imag = freqs_cis_imag.to(q.device)
        return functional_attention(
            q,
            k,
            v,
            dropout=dropout_p,
            num_heads=self.num_heads,
            num_k_exclude_rope=num_k_exclude_rope,
            freqs_cis=freqs_cis,
            freqs_cis_real=freqs_cis_real,
            freqs_cis_imag=freqs_cis_imag,
            use_fa3=False,
            use_rope_real=self.use_rope_real,
            rope_k_repeat=self.rope_k_repeat,
        )

    SimpleRoPEAttention.forward = forward
    stack.callback(setattr, SimpleRoPEAttention, "forward", orig_forward)


def _patch_decoder_coord_cache(handles, stack: contextlib.ExitStack):
    """Fix (1b): drop the cuda-pinned boxRPB coord cache in the DETR decoder.

    ``TransformerDecoder.__init__`` (``decoder.py:280-286``) pre-builds
    ``compilable_cord_cache`` via ``_get_coords(..., device="cuda")`` when
    ``resolution``/``stride`` are set (they are: 1008/14 → feat_size 72). Because
    ``compilable_stored_size == (H, W)`` matches at runtime, ``_get_rpb_matrix``
    (``decoder.py:341-346``) uses that *cuda* cache, so on our CPU export the
    coords collide with cpu ``reference_boxes`` →
    ``Expected all tensors to be on the same device, cuda:0 and cpu``.

    We wrap ``_get_rpb_matrix`` to null the compilable cache on entry, which
    routes to the device-correct recompute branch (``decoder.py:337-338`` /
    ``350-354``) that builds coords on ``reference_boxes.device``. Pure
    device-consistency fix; the math is identical.
    """
    from sam3.model.decoder import TransformerDecoder

    orig = TransformerDecoder._get_rpb_matrix

    def _get_rpb_matrix(self, reference_boxes, feat_size):
        self.compilable_cord_cache = None
        self.compilable_stored_size = None
        self.coord_cache = {}
        return orig(self, reference_boxes, feat_size)

    TransformerDecoder._get_rpb_matrix = _get_rpb_matrix
    stack.callback(setattr, TransformerDecoder, "_get_rpb_matrix", orig)


def _patch_mask_downsampler_antialias(handles, stack: contextlib.ExitStack):
    """Fix (6): drop antialias in the maskmem mask downsampler's resize.

    ``SimpleMaskDownSampler.forward`` (``memory.py:77-86``) resizes the input
    masks with ``F.interpolate(..., mode="bilinear", antialias=True)``. The
    antialiased path lowers to ``aten::_upsample_bilinear2d_aa``, which ONNX
    opset 17 does not support (``UnsupportedOperatorError``).

    For this checkpoint the resize is 1008→1152 (``interpol_size=[1152,1152]``),
    i.e. an *upsample* — and antialiasing is a no-op when upsampling (it only
    band-limits when downscaling). So we re-run the same bilinear interpolate
    with ``antialias=False``: numerically equivalent here, and a plain
    ``Resize`` op that ONNX/TensorRT support.
    """
    import torch.nn.functional as F

    from sam3.model.memory import SimpleMaskDownSampler

    orig_forward = SimpleMaskDownSampler.forward

    def forward(self, x):
        if self.interpol_size is not None and self.interpol_size != list(x.shape[-2:]):
            x = F.interpolate(
                x.float(),
                size=self.interpol_size,
                align_corners=False,
                mode="bilinear",
                antialias=False,  # no-op vs True for upsampling; ONNX-exportable
            )
        return self.encoder(x)

    SimpleMaskDownSampler.forward = forward
    stack.callback(setattr, SimpleMaskDownSampler, "forward", orig_forward)


def _patch_geometry_concat(handles, stack: contextlib.ExitStack):
    """Fix (3): scatter/assert-free concat in geometry_encoders."""
    import sam3.model.geometry_encoders as geo

    orig = geo.concat_padded_sequences

    def concat_padded_sequences(seq1, mask1, seq2, mask2, return_index: bool = False):
        cat_seq, cat_mask = concat_padded_right(seq1, mask1, seq2, mask2)
        if return_index:
            import torch

            s2 = seq2.shape[0]
            idx = (
                torch.arange(s2, device=seq2.device)[:, None]
                + seq1.shape[0]
            ).repeat(1, seq2.shape[1])
            return cat_seq, cat_mask, idx
        return cat_seq, cat_mask

    geo.concat_padded_sequences = concat_padded_sequences
    stack.callback(setattr, geo, "concat_padded_sequences", orig)


def _patch_geometry_roi_align(handles, stack: contextlib.ExitStack):
    """Fix (4): roi_align with a [K,5] batch-index rois tensor, no pin_memory."""
    from sam3.model.box_ops import box_cxcywh_to_xyxy
    from sam3.model.geometry_encoders import SequenceGeometryEncoder

    orig = SequenceGeometryEncoder._encode_boxes

    def _encode_boxes(self, boxes, boxes_mask, boxes_labels, img_feats):
        import torch
        import torchvision

        boxes_embed = None
        n_boxes, bs = boxes.shape[:2]

        if self.boxes_direct_project is not None:
            boxes_embed = self.boxes_direct_project(boxes)

        if self.boxes_pool_project is not None:
            H, W = img_feats.shape[-2:]
            boxes_xyxy = box_cxcywh_to_xyxy(boxes)  # [N, B, 4], normalised
            scale = torch.tensor(
                [W, H, W, H], dtype=boxes_xyxy.dtype, device=boxes_xyxy.device
            ).view(1, 1, 4)
            boxes_xyxy = boxes_xyxy * scale  # abs px, [N, B, 4]

            # Build a single [B*N, 5] rois tensor: [batch_idx, x1, y1, x2, y2].
            boxes_bn = boxes_xyxy.permute(1, 0, 2).reshape(bs * n_boxes, 4)  # B-major
            batch_idx = (
                torch.arange(bs, device=boxes_xyxy.device)
                .view(bs, 1)
                .expand(bs, n_boxes)
                .reshape(bs * n_boxes, 1)
                .to(boxes_bn.dtype)
            )
            rois = torch.cat([batch_idx, boxes_bn], dim=1)  # [B*N, 5]
            sampled = torchvision.ops.roi_align(
                img_feats, rois, output_size=self.roi_size
            )
            proj = self.boxes_pool_project(sampled)  # [B*N, C, 1, 1]
            # back to seq-first [N, B, C]
            proj = proj.view(bs, n_boxes, self.d_model).transpose(0, 1)
            boxes_embed = proj if boxes_embed is None else boxes_embed + proj

        if self.boxes_pos_enc_project is not None:
            cx, cy, w, h = boxes.unbind(-1)
            enc = self.pos_enc.encode_boxes(
                cx.flatten(), cy.flatten(), w.flatten(), h.flatten()
            )
            enc = enc.view(boxes.shape[0], boxes.shape[1], enc.shape[-1])
            proj = self.boxes_pos_enc_project(enc)
            boxes_embed = proj if boxes_embed is None else boxes_embed + proj

        type_embed = self.label_embed(boxes_labels.long())
        return type_embed + boxes_embed, boxes_mask

    SequenceGeometryEncoder._encode_boxes = _encode_boxes
    stack.callback(setattr, SequenceGeometryEncoder, "_encode_boxes", orig)


@contextlib.contextmanager
def apply_all(handles=None):
    """Context manager applying every ONNX-export patch, restoring on exit.

    ``handles`` (a ``load_native.NativeHandles``) is accepted for symmetry /
    future per-instance buffer registration but the current patches are all at
    the class/function level.
    """
    stack = contextlib.ExitStack()
    try:
        _patch_fused_addmm_act(handles, stack)
        _patch_position_encoding(handles, stack)
        _patch_decoder_coord_cache(handles, stack)
        _patch_mask_downsampler_antialias(handles, stack)
        _patch_simple_rope(handles, stack)
        _patch_geometry_concat(handles, stack)
        _patch_geometry_roi_align(handles, stack)
        yield
    finally:
        stack.close()


__all__ = [
    "apply_all",
    "additive_mask_from_bool",
    "concat_padded_right",
    "NEG_INF_FP16",
]
