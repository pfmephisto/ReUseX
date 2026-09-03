# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Four ``nn.Module`` wrappers around ``demo.detector.*`` submodules, one per
detector engine. Each wrapper's ``forward`` returns tensors in the order of the
contract's ``output_names`` (see ``reusex_sam3.ENGINE_IO_CONTRACT``).

Native attribute paths targeted (via ``load_native.NativeHandles``):

  VisionEncoderWrapper   -> detector.backbone.vision_backbone  (Sam3TriViTDetNeck:
                            .trunk ViT, .convs, .position_encoding)
  TextEncoderWrapper     -> detector.backbone.language_backbone (VETextEncoder:
                            .encoder TextTransformer, .resizer)
  GeometryEncoderWrapper -> detector.geometry_encoder (SequenceGeometryEncoder)
  DecoderWrapper         -> detector.transformer.{encoder,decoder},
                            detector.segmentation_head, detector.dot_prod_scoring
                            (driven through the detector's own _run_encoder /
                            _run_decoder / _run_segmentation_heads methods)

All wrappers operate at the fixed 1008x1008 / 72x72 resolution and batch-first
external I/O, transposing to the native seq-first convention internally.
"""

from __future__ import annotations

import torch
import torch.nn as nn


# ---------------------------------------------------------------------------
# 1. Vision encoder
# ---------------------------------------------------------------------------
class VisionEncoderWrapper(nn.Module):
    """images[B,3,1008,1008] -> fpn_feat_0, fpn_feat_1, fpn_feat_2, fpn_pos_2.

    Runs only the SAM3 head of the tri-head neck (need_interactive_out=False,
    need_propagation_out=False). The neck returns 3 FPN levels (scale_factors
    4.0, 2.0, 1.0) as NestedTensors + their sine pos-encs; we unwrap to plain
    tensors and emit the top-level pos-enc as fpn_pos_2.
    """

    def __init__(self, handles):
        super().__init__()
        self.neck = handles.det_vision_neck  # Sam3TriViTDetNeck

    def forward(self, images: torch.Tensor):
        sam3_feats, sam3_pos, *_ = self.neck(
            images,
            need_sam3_out=True,
            need_interactive_out=False,
            need_propagation_out=False,
        )
        # sam3_feats: list[NestedTensor] length 3, highest→lowest resolution
        f0 = _unwrap(sam3_feats[0])   # [B,256,288,288]
        f1 = _unwrap(sam3_feats[1])   # [B,256,144,144]
        f2 = _unwrap(sam3_feats[2])   # [B,256,72,72]
        pos2 = sam3_pos[-1]           # [B,256,72,72]
        return f0, f1, f2, pos2


# ---------------------------------------------------------------------------
# 2. Text encoder
# ---------------------------------------------------------------------------
class TextEncoderWrapper(nn.Module):
    """input_ids[B,32] int64, attention_mask[B,32] int64 -> text_features, text_mask.

    Bypasses the tokenizer: takes pre-tokenised ids directly. Reproduces
    ``VETextEncoder.forward``'s already-tokenised branch: run the inner
    TextTransformer (output_tokens=True) to get per-token features, project
    1024->256 via ``resizer``. Exports batch-first [B,32,256].

    text_mask follows the model's convention (True == padding); we derive it
    from ``attention_mask`` (1 == real token) as ``attention_mask == 0``.
    """

    def __init__(self, handles):
        super().__init__()
        self.text = handles.det_text_encoder      # VETextEncoder
        self.encoder = self.text.encoder          # TextTransformer
        self.resizer = self.text.resizer          # Linear(1024, 256)

    def forward(self, input_ids: torch.Tensor, attention_mask: torch.Tensor):
        text = input_ids.long()
        # TextTransformer.forward with output_tokens=True returns (pooled, tokens)
        _, tokens = self.encoder(text)             # tokens: [B, 32, 1024]
        text_features = self.resizer(tokens)       # [B, 32, 256]
        text_mask = attention_mask == 0            # True == padding, [B, 32] bool
        return text_features, text_mask


# ---------------------------------------------------------------------------
# 3. Geometry encoder
# ---------------------------------------------------------------------------
class GeometryEncoderWrapper(nn.Module):
    """input_boxes[B,N,4] cxcywh, input_boxes_labels[B,N], fpn_feat_2, fpn_pos_2
    -> geometry_features[B,N+1,256], geometry_mask[B,N+1].

    Wraps ``SequenceGeometryEncoder.forward``. The native encoder expects
    seq-first box tensors and a Prompt object plus the img feats as a seq-first
    list; we build those from the batch-first contract inputs. The roi_align /
    scatter blockers inside are handled by ``fixes.apply_all``.
    """

    def __init__(self, handles):
        super().__init__()
        self.geo = handles.det_geometry_encoder   # SequenceGeometryEncoder
        self.feat_h = 72
        self.feat_w = 72

    def forward(
        self,
        input_boxes: torch.Tensor,       # [B, N, 4] cxcywh normalised
        input_boxes_labels: torch.Tensor,  # [B, N]
        fpn_feat_2: torch.Tensor,        # [B, 256, 72, 72]
        fpn_pos_2: torch.Tensor,         # [B, 256, 72, 72]
    ):
        from sam3.model.geometry_encoders import Prompt

        B, N, _ = input_boxes.shape
        # seq-first: boxes [N,B,4], labels [N,B], mask [B,N] (True == pad; none here)
        boxes = input_boxes.transpose(0, 1).contiguous()
        labels = input_boxes_labels.transpose(0, 1).contiguous().long()
        box_mask = torch.zeros(B, N, dtype=torch.bool, device=input_boxes.device)

        prompt = Prompt(box_embeddings=boxes, box_mask=box_mask, box_labels=labels)

        # img feats as a seq-first list [H*W, B, C] and matching pos-enc list
        img_feat_seq = fpn_feat_2.flatten(2).permute(2, 0, 1)   # [HW, B, C]
        img_pos_seq = fpn_pos_2.flatten(2).permute(2, 0, 1)     # [HW, B, C]

        feats, mask = self.geo(
            geo_prompt=prompt,
            img_feats=[img_feat_seq],
            img_sizes=[(self.feat_h, self.feat_w)],
            img_pos_embeds=[img_pos_seq],
        )
        # feats seq-first [N+1, B, C] (N boxes + 1 CLS); mask [B, N+1]
        geometry_features = feats.transpose(0, 1).contiguous()  # [B, N+1, C]
        geometry_mask = mask
        return geometry_features, geometry_mask


# ---------------------------------------------------------------------------
# 4. Decoder (encoder-fusion + DETR decoder + segmentation head)
# ---------------------------------------------------------------------------
class DecoderWrapper(nn.Module):
    """fpn_feat_0/1/2, fpn_pos_2, prompt_features[B,L,256], prompt_mask[B,L]
    -> pred_masks, pred_boxes, pred_logits, presence_logits.

    ``prompt_features`` is the concatenation of text tokens + geometry tokens
    (the C++ side concatenates the text-encoder and geometry-encoder outputs and
    right-pads to L). This wrapper reproduces the detector's own
    ``_run_encoder`` → ``_run_decoder`` → ``_run_segmentation_heads`` pipeline by
    building the minimal ``backbone_out`` / ``find_input`` those methods expect,
    so we exercise the *real* fusion encoder, DETR decoder (with box refine +
    presence token) and the universal segmentation head.
    """

    def __init__(self, handles):
        super().__init__()
        self.det = handles.detector
        self.feat_sizes = [(72, 72)]  # single feature level (num_feature_levels=1)

    def forward(
        self,
        fpn_feat_0: torch.Tensor,   # [B,256,288,288]
        fpn_feat_1: torch.Tensor,   # [B,256,144,144]
        fpn_feat_2: torch.Tensor,   # [B,256,72,72]
        fpn_pos_2: torch.Tensor,    # [B,256,72,72]
        prompt_features: torch.Tensor,  # [B,L,256]
        prompt_mask: torch.Tensor,      # [B,L] bool (True == pad)
    ):
        det = self.det
        B = fpn_feat_2.shape[0]
        device = fpn_feat_2.device

        # seq-first prompt [L,B,C]
        prompt = prompt_features.transpose(0, 1).contiguous()

        # Assemble a backbone_out dict as produced by the tri-head backbone's
        # SAM3 output, wrapped so _get_img_feats can index it.
        from sam3.model.data_misc import FindStage, NestedTensor

        backbone_fpn = [
            NestedTensor(fpn_feat_0, None),
            NestedTensor(fpn_feat_1, None),
            NestedTensor(fpn_feat_2, None),
        ]
        # position encodings per level; only the top-level (level 2) is used by
        # the single-feature-level encoder path (num_feature_levels=1).
        vision_pos = [torch.zeros_like(fpn_feat_0), torch.zeros_like(fpn_feat_1), fpn_pos_2]
        img_ids = torch.zeros(B, dtype=torch.long, device=device)
        backbone_out = {
            "backbone_fpn": backbone_fpn,
            "vision_pos_enc": vision_pos,
            "id_mapping": None,
        }
        find_input = FindStage(
            img_ids=img_ids,
            img_ids_np=None,
            text_ids=torch.arange(B, device=device),
            input_boxes=None,
            input_boxes_mask=None,
            input_boxes_label=None,
            input_points=None,
            input_points_mask=None,
            ptrs=None,
            ptrs_seg=None,
            object_ids=None,
        )

        # --- encoder fusion -------------------------------------------------
        backbone_out, encoder_out, _ = det._run_encoder(
            backbone_out, find_input, prompt, prompt_mask
        )
        out = {
            "encoder_hidden_states": encoder_out["encoder_hidden_states"],
            "prev_encoder_out": {"encoder_out": encoder_out, "backbone_out": backbone_out},
        }

        # --- DETR decoder ---------------------------------------------------
        out, hs = det._run_decoder(
            memory=out["encoder_hidden_states"],
            pos_embed=encoder_out["pos_embed"],
            src_mask=encoder_out["padding_mask"],
            out=out,
            prompt=prompt,
            prompt_mask=prompt_mask,
            encoder_out=encoder_out,
        )

        # --- segmentation head ---------------------------------------------
        det._run_segmentation_heads(
            out=out,
            backbone_out=backbone_out,
            img_ids=img_ids,
            vis_feat_sizes=encoder_out["vis_feat_sizes"],
            encoder_hidden_states=out["encoder_hidden_states"],
            prompt=prompt,
            prompt_mask=prompt_mask,
            hs=hs,
        )

        pred_masks = out["pred_masks"]          # [B, Q, H, W]
        pred_boxes = out["pred_boxes"]          # [B, Q, 4] cxcywh
        pred_logits = out["pred_logits"]        # [B, Q, 1]
        # presence: prefer the decoder presence logit, else the seg-head one
        presence = out.get("presence_logit_dec", out.get("presence_logit"))
        if presence is None:
            presence = pred_logits.new_zeros(pred_logits.shape[0], 1)
        else:
            presence = presence.reshape(pred_logits.shape[0], -1)[:, :1]
        return pred_masks, pred_boxes, pred_logits, presence


def _unwrap(x):
    """NestedTensor -> plain tensor (else identity)."""
    return getattr(x, "tensors", x)
