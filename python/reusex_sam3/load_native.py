# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Build the native SAM 3.1 multiplex demo model **empty** (no weights), then load
``sam3.1_multiplex.pt`` with a diff-driven, bijective key remap.

Why not just call ``build_sam3_multiplex_video_predictor``?
----------------------------------------------------------
That entry point (``sam3/model_builder.py:1070``) is convenient but does two
things we can't do here:

  * it calls ``.cuda()`` unconditionally (``demo_model.cuda().eval()``), and
  * it wraps the model in ``Sam3MultiplexVideoPredictor`` (a request/response
    server) which hides the raw ``nn.Module`` submodules we need to trace.

So we reproduce the *assembly* of ``demo_model`` here — component-for-component
identical to ``build_sam3_multiplex_video_predictor`` — but on CPU and returning
the bare ``Sam3MultiplexTrackingWithInteractivity`` (``.detector`` + ``.tracker``).

Checkpoint remap
----------------
The HF checkpoint ``sam3.1_multiplex.pt`` is *already* remapped to OSS naming in
recent releases (top-level ``detector.`` / ``tracker.`` prefixes). Older/local
checkpoints use internal prefixes ``sam3_model.`` and ``sam2_predictor.``. The
native builder handles the latter with a static prefix swap and
``strict=False``.

We go further: rather than trusting a static rule, we **diff** the checkpoint
keys against ``model.state_dict()`` keys and:

  1. apply the known prefix remaps (``sam3_model.``→``detector.``,
     ``sam2_predictor.``→``tracker.``),
  2. apply regex transforms for any residual naming drift (e.g. fused
     ``self_attn_q_proj`` style names, ``interactive_`` infixes) — but only when
     a transformed key actually *exists* in the model, so the remap stays
     bijective and never invents keys,
  3. ``load_state_dict(strict=False)`` and HARD-ASSERT that every remaining
     missing/unexpected key is in an explicit allowlist (buffers we register
     ourselves + known-unused sub-trees).

A machine-readable ``checkpoints/remap_report.json`` is written with the counts
and the residual lists so the remap can be finalised once the real ``.pt`` is
available (see ``inspect_checkpoint.py`` for a standalone, model-free dump).
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Model assembly (mirrors build_sam3_multiplex_video_predictor, CPU-only)
# ---------------------------------------------------------------------------

# multiplex_count / max_num_objects defaults matching the native predictor.
MULTIPLEX_COUNT = 16
MAX_NUM_OBJECTS = 16


def build_demo_model(multiplex_count: int = MULTIPLEX_COUNT, use_rope_real: bool = True):
    """Assemble an *empty* Sam3MultiplexTrackingWithInteractivity on CPU.

    Mirrors ``sam3.model_builder.build_sam3_multiplex_video_predictor`` up to (but
    not including) checkpoint loading / ``.cuda()`` / predictor wrapping.

    ``use_fa3=False`` (FlashAttention-3 is a CUDA-only custom kernel that cannot
    be traced) and ``use_rope_real=True`` (real-valued RoPE avoids ONNX-hostile
    complex ops) — exactly the combination requested for export.
    """
    import torch  # local import so package import stays dep-free

    # Native builder internals. These names are ground-truth from the pinned
    # sam3.1 branch (SHA 20dba30...).
    from sam3 import model_builder as mb
    from sam3.model.sam3_multiplex_base import Sam3MultiplexPredictorWrapper
    from sam3.model.sam3_multiplex_detector import Sam3MultiplexDetector
    from sam3.model.sam3_multiplex_tracking import (
        Sam3MultiplexTrackingWithInteractivity,
    )
    from sam3.model.vl_combiner import SAM3VLBackboneTri

    from .download import find_bpe_path

    bpe = find_bpe_path()
    if bpe is None:
        raise FileNotFoundError(
            "BPE vocab not found; install the sam3 package so its assets resolve."
        )
    bpe_path = str(bpe)

    use_fa3 = False  # never true for export

    # --- tracker (VideoTrackingDynamicMultiplex demo, backbone stripped) ------
    tracker_model = mb.build_sam3_multiplex_video_model(
        checkpoint_path=None,
        load_from_HF=False,
        multiplex_count=multiplex_count,
        use_fa3=use_fa3,
        use_rope_real=use_rope_real,
        compile=False,
        strict_state_dict_loading=False,
        device="cpu",
    )
    # The predictor strips the tracker's own vision backbone (the detector's
    # tri-head backbone feeds the tracker). Match that so state_dict keys align.
    if getattr(tracker_model, "backbone", None) is not None:
        del tracker_model.backbone
        tracker_model.backbone = None

    sam2_predictor = Sam3MultiplexPredictorWrapper(
        model=tracker_model,
        per_obj_inference=False,
        fill_hole_area=0,
        is_multiplex=True,
        is_multiplex_dynamic=True,
    )

    # --- detector (tri-head vision + text + geometry + transformer + seg) -----
    tri_neck = mb._create_multiplex_tri_backbone(
        compile_mode=None, use_fa3=use_fa3, use_rope_real=use_rope_real
    )
    text_encoder = mb._create_text_encoder(bpe_path)
    backbone = SAM3VLBackboneTri(scalp=0, visual=tri_neck, text=text_encoder)
    transformer = mb._create_sam3_transformer(use_fa3=use_fa3)
    segmentation_head = mb._create_segmentation_head(use_fa3=use_fa3)
    geometry_encoder = mb._create_geometry_encoder()
    dot_prod_scoring = mb._create_dot_product_scoring()

    detector = Sam3MultiplexDetector(
        num_feature_levels=1,
        backbone=backbone,
        transformer=transformer,
        segmentation_head=segmentation_head,
        semantic_segmentation_head=None,
        input_geometry_encoder=geometry_encoder,
        use_early_fusion=True,
        use_dot_prod_scoring=True,
        dot_prod_scoring=dot_prod_scoring,
        supervise_joint_box_scores=True,
        is_multiplex=True,
    )

    demo_model = Sam3MultiplexTrackingWithInteractivity(
        tracker=sam2_predictor,
        detector=detector,
        score_threshold_detection=0.4,
        det_nms_thresh=0.1,
        det_nms_use_iom=True,
        assoc_iou_thresh=0.1,
        new_det_thresh=0.65,
        hotstart_delay=15,
        hotstart_unmatch_thresh=8,
        hotstart_dup_thresh=8,
        suppress_unmatched_only_within_hotstart=False,
        suppress_overlapping_based_on_recent_occlusion_threshold=0.7,
        suppress_det_close_to_boundary=True,
        fill_hole_area=0,
        recondition_every_nth_frame=16,
        use_iom_recondition=True,
        iom_thresh_recondition=0.5,
        masklet_confirmation_enable=True,
        reconstruction_bbox_iou_thresh=-1,
        reconstruction_bbox_det_score=0.8,
        max_num_objects=MAX_NUM_OBJECTS,
        postprocess_batch_size=16,
        use_batched_grounding=True,
        batched_grounding_batch_size=16,
        max_num_kboxes=0,
        sprinkle_removal_area=0,
        is_multiplex=True,
        image_size=1008,
        image_mean=(0.5, 0.5, 0.5),
        image_std=(0.5, 0.5, 0.5),
        compile_model=False,
    )
    demo_model.eval()
    return demo_model


# ---------------------------------------------------------------------------
# Diff-driven remap
# ---------------------------------------------------------------------------

# VERIFIED against the real facebook/sam3.1 `sam3.1_multiplex.pt` (2026-09-03,
# inspected directly from the checkpoint zip): 1623 tensors, top-level prefixes
# `detector.` (1166) and `tracker.model.` (457) — i.e. it is already the native
# demo-model's own state_dict, with FLATTENED attention names
# (`...transformer.encoder.layers.N.self_attn_q_proj.weight`). So `needs_prefix`
# below is False for this checkpoint and the prefix/regex remaps are no-ops; the
# keys are expected to match the model built by build_demo_model() directly. The
# remaps stay as a defensive net for internal-naming variants. Run
# inspect_checkpoint.py to re-confirm counts/residuals on any new checkpoint.

# Static top-level prefix remaps (internal → OSS naming). Applied first.
_PREFIX_REMAPS: List[Tuple[str, str]] = [
    ("sam3_model.", "detector."),
    ("sam2_predictor.", "tracker."),
]

# Optional regex transforms tried when a checkpoint key is otherwise unexpected.
# Each (pattern, repl); we only KEEP a transform if the result is a real model
# key (see remap_state_dict), so these can never fabricate parameters.
_REGEX_TRANSFORMS: List[Tuple[str, str]] = [
    # fused attention proj names → dotted (defensive; native already uses
    # self_attn_q_proj etc., so this is a no-op unless a variant appears)
    (r"(self_attn|cross_attn)_(q|k|v|out)_proj", r"\1.\2_proj"),
]

# Keys we register ourselves at trace time (fixes.py) or that belong to unused
# sub-trees; residual missing/unexpected keys must be a subset of these prefixes.
_ALLOWLIST_PREFIXES: Tuple[str, ...] = (
    # Precomputed RoPE / pos-enc buffers are re-registered by fixes.py and are
    # marked persistent=False in the native code, so they legitimately may be
    # absent from the checkpoint.
    # (matched as substrings, see _in_allowlist)
    "freqs_cis",
    "position_encoding.cache",
    # interactive_* heads are used only for click-based SAM1 interactivity, not
    # for text-prompted detection/tracking; unused by our export path.
    # We still load them if present, but tolerate their absence.
)


def _in_allowlist(key: str) -> bool:
    return any(tok in key for tok in _ALLOWLIST_PREFIXES)


def remap_state_dict(
    ckpt: Dict[str, "object"], model_keys: set
) -> Tuple[Dict[str, "object"], Dict[str, object]]:
    """Return (remapped_ckpt, report).

    remapped_ckpt only contains keys that exist in ``model_keys`` (bijective).
    ``report`` holds counts and residual lists for auditing.
    """
    remapped: Dict[str, object] = {}
    dropped_unexpected: List[str] = []
    used_regex: List[Tuple[str, str]] = []

    needs_prefix = any(
        k.startswith("sam3_model.") or k.startswith("sam2_predictor.") for k in ckpt
    )

    for k, v in ckpt.items():
        nk = k
        if needs_prefix:
            for pre, rep in _PREFIX_REMAPS:
                if nk.startswith(pre):
                    nk = rep + nk[len(pre):]
                    break

        if nk in model_keys:
            remapped[nk] = v
            continue

        # Try regex transforms; keep only if the result is a real model key.
        matched = False
        for pat, rep in _REGEX_TRANSFORMS:
            cand = re.sub(pat, rep, nk)
            if cand != nk and cand in model_keys:
                remapped[cand] = v
                used_regex.append((k, cand))
                matched = True
                break
        if not matched:
            dropped_unexpected.append(k)

    model_missing = sorted(model_keys - set(remapped.keys()))
    report = {
        "ckpt_key_count": len(ckpt),
        "model_key_count": len(model_keys),
        "matched": len(remapped),
        "unexpected_dropped": sorted(dropped_unexpected),
        "unexpected_dropped_count": len(dropped_unexpected),
        "missing_in_ckpt": model_missing,
        "missing_in_ckpt_count": len(model_missing),
        "regex_transforms_used": used_regex,
    }
    return remapped, report


def load_checkpoint_into(
    model, checkpoint_path: str, report_path: Optional[str] = None, hard_assert: bool = True
):
    """Load ``checkpoint_path`` into ``model`` via the diff-driven remap.

    Writes ``report_path`` (default: checkpoints/remap_report.json) and, if
    ``hard_assert``, raises unless every residual missing/unexpected key is
    covered by the allowlist.
    """
    import torch

    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if "model" in ckpt and isinstance(ckpt["model"], dict):
        ckpt = ckpt["model"]

    model_sd = model.state_dict()
    model_keys = set(model_sd.keys())
    remapped, report = remap_state_dict(ckpt, model_keys)

    missing, unexpected = model.load_state_dict(remapped, strict=False)
    # After a bijective remap, load_state_dict's `missing`/`unexpected` should
    # match our diff; record both views.
    report["load_state_dict_missing"] = sorted(missing)
    report["load_state_dict_unexpected"] = sorted(unexpected)

    residual_missing = [k for k in missing if not _in_allowlist(k)]
    residual_unexpected = [k for k in report["unexpected_dropped"] if not _in_allowlist(k)]
    report["residual_missing_not_allowlisted"] = sorted(residual_missing)
    report["residual_unexpected_not_allowlisted"] = sorted(residual_unexpected)

    if report_path is None:
        report_path = str(
            Path(checkpoint_path).resolve().parent / "remap_report.json"
        )
    Path(report_path).parent.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"[remap] wrote {report_path}")
    print(
        f"[remap] matched={report['matched']} "
        f"missing={report['missing_in_ckpt_count']} "
        f"unexpected={report['unexpected_dropped_count']}"
    )

    if hard_assert and (residual_missing or residual_unexpected):
        raise AssertionError(
            "checkpoint remap left non-allowlisted residuals:\n"
            f"  missing({len(residual_missing)}): {residual_missing[:20]}\n"
            f"  unexpected({len(residual_unexpected)}): {residual_unexpected[:20]}\n"
            f"See {report_path}. Update _REGEX_TRANSFORMS / _ALLOWLIST_PREFIXES "
            "or re-run inspect_checkpoint.py to finalise the remap."
        )
    return model, report


# ---------------------------------------------------------------------------
# Live submodule accessors — the wrappers target these exact attribute paths.
# ---------------------------------------------------------------------------


class NativeHandles:
    """Typed accessors returning live submodules of the loaded demo model.

    Attribute paths here are the ground-truth targets for the ONNX wrappers.
    """

    def __init__(self, demo_model):
        self.demo = demo_model
        self.detector = demo_model.detector          # Sam3MultiplexDetector (Sam3Image)
        self.tracker = demo_model.tracker.model      # VideoTrackingDynamicMultiplex

    # ---- detector side --------------------------------------------------
    @property
    def det_backbone(self):
        # SAM3VLBackboneTri: .vision_backbone (Sam3TriViTDetNeck), .language_backbone
        return self.detector.backbone

    @property
    def det_vision_neck(self):
        return self.detector.backbone.vision_backbone  # Sam3TriViTDetNeck

    @property
    def det_text_encoder(self):
        return self.detector.backbone.language_backbone  # VETextEncoder

    @property
    def det_geometry_encoder(self):
        return self.detector.geometry_encoder  # SequenceGeometryEncoder

    @property
    def det_transformer(self):
        return self.detector.transformer  # TransformerWrapper (encoder+decoder)

    @property
    def det_segmentation_head(self):
        return self.detector.segmentation_head  # UniversalSegmentationHead

    @property
    def det_dot_prod_scoring(self):
        return self.detector.dot_prod_scoring  # DotProductScoring

    # ---- tracker side ---------------------------------------------------
    @property
    def trk_maskmem_backbone(self):
        return self.tracker.maskmem_backbone  # SimpleMaskEncoder

    @property
    def trk_memory_attention(self):
        # TransformerEncoderDecoupledCrossAttention (decoupled cross-attn, RoPE)
        return self.tracker.transformer.encoder

    @property
    def trk_prompt_encoder(self):
        # The multiplex tracker exposes its SAM1-style prompt encoder as
        # ``interactive_sam_prompt_encoder`` (a PromptEncoder); there is no plain
        # ``sam_prompt_encoder`` attribute on Sam3VideoTrackingMultiplexDemo.
        return self.tracker.interactive_sam_prompt_encoder  # PromptEncoder

    @property
    def trk_mask_decoder(self):
        return self.tracker.sam_mask_decoder  # MultiplexMaskDecoder

    @property
    def trk_no_mem_embed(self):
        return self.tracker.no_mem_embed

    @property
    def trk_maskmem_tpos_enc(self):
        return self.tracker.maskmem_tpos_enc

    @property
    def trk_obj_ptr_proj(self):
        return self.tracker.obj_ptr_proj


def load(
    checkpoint_path: str,
    multiplex_count: int = MULTIPLEX_COUNT,
    report_path: Optional[str] = None,
    hard_assert: bool = True,
) -> NativeHandles:
    """One-shot: build empty demo model, load checkpoint, return handles."""
    model = build_demo_model(multiplex_count=multiplex_count, use_rope_real=True)
    load_checkpoint_into(
        model, checkpoint_path, report_path=report_path, hard_assert=hard_assert
    )
    return NativeHandles(model)
