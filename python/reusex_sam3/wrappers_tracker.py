# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Tracker-side ``nn.Module`` wrappers. The native tracker (``demo.tracker.model``,
a ``VideoTrackingDynamicMultiplex``) drives the whole memory bank with Python
control flow and a runtime ``MultiplexState`` object that is neither traceable
nor meaningful to a fixed-shape engine. So instead of wrapping the big
``_prepare_memory_conditioned_features`` / ``_encode_new_memory`` methods
wholesale, each wrapper targets the underlying **neural sub-module** and
reproduces just the tensor math, with a *fixed* memory bank and boolean mask —
no dynamic control flow.

Native attribute paths targeted (via ``load_native.NativeHandles``):

  MemoryEncoderWrapper   -> tracker.maskmem_backbone (SimpleMaskEncoder) plus the
                            sigmoid/scale/bias pre-processing and no_obj_embed add
                            from ``_encode_new_memory``.
  MemoryAttentionWrapper -> tracker.transformer.encoder
                            (TransformerEncoderDecoupledCrossAttention) — "Step 2"
                            of ``_prepare_memory_conditioned_features``.
  PromptEncoderWrapper   -> tracker.sam_prompt_encoder (PromptEncoder)
  MultiplexDecoderWrapper-> tracker.sam_mask_decoder (MultiplexMaskDecoder)

See ``reusex_sam3.ENGINE_IO_CONTRACT`` for the exact I/O names and shapes and
``reusex_sam3.TRACKER_META`` for the memory-bank integer layout.
"""

from __future__ import annotations

import torch
import torch.nn as nn


# ---------------------------------------------------------------------------
# tracker-memory-encoder
# ---------------------------------------------------------------------------
class MemoryEncoderWrapper(nn.Module):
    """vision_feat[B,256,72,72], pred_mask[K,1,1008,1008], object_score_logits[K,1]
    -> maskmem_features[B,256,72,72], maskmem_pos_enc[B,256,72,72].

    Reproduces the SimpleMaskEncoder branch of the multiplex
    ``_encode_new_memory`` (video_tracking_multiplex.py:1616) for the fixed
    export case ``num_buckets == 1`` and ``multiplex_count == K == 16``, dropping
    the runtime ``MultiplexState`` bookkeeping (which only reshapes tensors when
    num_buckets==1). Verified against native config on the real checkpoint:

      * apply_sigmoid_to_mask_logits_for_mem_enc=True, sigmoid_scale=2.0,
        sigmoid_bias=-1.0 → ``mask_for_mem = sigmoid(pred_mask)*2 - 1``.
      * mux (num_buckets=1): ``[K,1,H,W] -> [1,K,H,W]``.
      * condition_as_mask_input=True (fg=1.0, bg=0.0): with no conditioning
        objects on this frame, all conditioning channels are ``bg`` (0.0), so we
        concat a zeros ``[1,K,H,W]`` block → ``[1,2K,H,W]`` (== the downsampler's
        32 in-channels: multiplex_count*input_channel_multiplier).
      * maskmem = maskmem_backbone(vision_feat, mux_mask, skip_mask_sigmoid=True).
      * no_obj_embed_spatial is ``[K,256]`` (multiplex override, NOT the base
        class ``[1,256]``); object_score_logits ``[K,1]`` mux → ``[1,K,1]``;
        ``no_obj = ((1-is_obj)*no_obj_embed_spatial).sum(dim=1)`` → ``[1,256]``,
        broadcast-added to features. Threshold = object_score_logit_threshold=0.

    K is ``multiplex_count`` (16); the extra 16 conditioning channels are derived
    internally, so the C++ side supplies exactly 16 per-object mask logits.
    """

    def __init__(self, handles):
        super().__init__()
        trk = handles.tracker
        self.maskmem_backbone = trk.maskmem_backbone  # SimpleMaskEncoder
        self.apply_sigmoid = bool(
            getattr(trk, "apply_sigmoid_to_mask_logits_for_mem_enc", True)
        )
        self.sigmoid_scale = float(getattr(trk, "sigmoid_scale_for_mem_enc", 2.0))
        self.sigmoid_bias = float(getattr(trk, "sigmoid_bias_for_mem_enc", -1.0))
        self.condition_as_mask_input = bool(
            getattr(trk, "condition_as_mask_input", True)
        )
        self.cond_bg = float(getattr(trk, "condition_as_mask_input_bg", 0.0))
        self.no_obj_embed_spatial = getattr(trk, "no_obj_embed_spatial", None)  # [K,256]
        self.obj_score_thresh = float(getattr(trk, "object_score_logit_threshold", 0.0))

    def forward(
        self,
        vision_feat: torch.Tensor,          # [B,256,72,72] (BCHW, top-level feats)
        pred_mask: torch.Tensor,            # [K,1,1008,1008] per-object logits
        object_score_logits: torch.Tensor,  # [K,1]
    ):
        # sigmoid + scale/bias (apply_sigmoid_to_mask_logits_for_mem_enc=True)
        if self.apply_sigmoid:
            mask_for_mem = torch.sigmoid(pred_mask)
            if self.sigmoid_scale != 1.0:
                mask_for_mem = mask_for_mem * self.sigmoid_scale
            if self.sigmoid_bias != 0.0:
                mask_for_mem = mask_for_mem + self.sigmoid_bias
        else:
            mask_for_mem = pred_mask

        # mux with num_buckets==1: [K,1,H,W] -> [1,K,H,W]
        mux_mask = mask_for_mem.squeeze(1).unsqueeze(0)  # [1, K, 1008, 1008]

        # condition_as_mask_input: append K conditioning channels. No conditioning
        # objects on an export frame → all bg (cond_bg, typically 0.0).
        if self.condition_as_mask_input:
            cond = torch.full_like(mux_mask, self.cond_bg)
            mux_mask = torch.cat([mux_mask, cond], dim=1)  # [1, 2K, H, W]

        out = self.maskmem_backbone(vision_feat, mux_mask, skip_mask_sigmoid=True)
        maskmem_features = out["vision_features"]      # [B,256,72,72]
        maskmem_pos_enc = out["vision_pos_enc"][0]     # [B,256,72,72]

        if self.no_obj_embed_spatial is not None:
            # no_obj_embed_spatial: [K,256]; object_score_logits [K,1] mux -> [1,K,1]
            no_obj_embed = self.no_obj_embed_spatial.unsqueeze(0)  # [1,K,256]
            osl = object_score_logits.unsqueeze(0)                 # [1,K,1]
            is_obj = (osl > self.obj_score_thresh).float()         # [1,K,1]
            no_obj = ((1.0 - is_obj) * no_obj_embed).sum(dim=1)    # [1,256]
            maskmem_features = maskmem_features + no_obj[..., None, None].expand_as(
                maskmem_features
            )

        return maskmem_features, maskmem_pos_enc


# ---------------------------------------------------------------------------
# tracker-memory-attention
# ---------------------------------------------------------------------------
class MemoryAttentionWrapper(nn.Module):
    """current_feat[5184,B,256], current_pos[5184,B,256],
    memory[M,B,256], memory_pos[M,B,256], memory_mask[B,M] (bool)
    -> pix_feat_with_mem[B,256,72,72].

    Wraps ``TransformerEncoderDecoupledCrossAttention.forward`` (decoder.py:1282),
    which the multiplex tracker calls with save_image_features=True as::

        encoder(image=current_feat, src=current_feat, memory_image=<img mem>,
                memory=<obj mem>, image_pos, src_pos, memory_image_pos,
                memory_pos, num_obj_ptr_tokens)

    DEVIATION (documented): the native encoder decouples memory into an
    *image* stream (spatial maskmem tokens) and an *object* stream (object
    pointers), fused by two separate cross-attentions. The shared C++ contract
    exposes a single flat ``memory`` bank. We therefore treat the whole contract
    ``memory`` as the spatial/image memory stream (``memory_image`` == ``memory``)
    and pass zero object-pointer tokens (``num_obj_ptr_tokens=0``); the object
    stream ``memory`` fed to the *object* cross-attention is the same bank. This
    matches the common case (spatial memory only). ``memory_mask`` is accepted
    for API completeness; the decoupled encoder does not consume a key-padding
    mask (it relies on RoPE + fixed lengths), so a fully-unpadded bank is
    assumed. The C++ side must therefore pass a right-padded, mask-free memory
    bank sized to ``mem_bank_max * mem_tokens_per_frame`` (see TRACKER_META).
    """

    def __init__(self, handles):
        super().__init__()
        self.encoder = handles.trk_memory_attention  # decoupled cross-attn encoder
        self.feat_h = 72
        self.feat_w = 72

    def forward(
        self,
        current_feat: torch.Tensor,   # [HW,B,C]
        current_pos: torch.Tensor,    # [HW,B,C]
        memory: torch.Tensor,         # [M,B,C]
        memory_pos: torch.Tensor,     # [M,B,C]
        memory_mask: torch.Tensor,    # [B,M] bool (documented: assumed all-valid)
    ):
        B = current_feat.shape[1]
        C = current_feat.shape[2]
        encoder_out = self.encoder(
            image=current_feat,
            src=current_feat,
            memory_image=memory,
            memory=memory,
            image_pos=current_pos,
            src_pos=current_pos,
            memory_image_pos=memory_pos,
            memory_pos=memory_pos,
            num_obj_ptr_tokens=0,
        )
        mem = encoder_out["memory"]  # [HW,B,C]
        pix_feat_with_mem = mem.permute(1, 2, 0).reshape(B, C, self.feat_h, self.feat_w)
        # The decoupled RoPE encoder does not consume a key-padding mask (it
        # relies on RoPE + fixed lengths — see class docstring), so ``memory_mask``
        # is functionally unused. But the shared C++/engine contract binds a
        # ``memory_mask`` input, and ONNX prunes trace-unused inputs. Tie it into
        # the output as an exact zero so the input survives to the graph without
        # altering any value: 0 * sum(mask) broadcasts to a scalar 0.
        keep = (memory_mask.to(pix_feat_with_mem.dtype).sum() * 0.0)
        pix_feat_with_mem = pix_feat_with_mem + keep
        return pix_feat_with_mem


# ---------------------------------------------------------------------------
# tracker-prompt-encoder
# ---------------------------------------------------------------------------
class PromptEncoderWrapper(nn.Module):
    """point_coords[B,P,2], point_labels[B,P], boxes[B,4], mask_input[B,1,288,288]
    -> sparse_embeddings[B,S,256], dense_embeddings[B,256,72,72].

    Thin wrapper over ``PromptEncoder.forward`` (sam3/sam/prompt_encoder.py:157).
    The native forward takes ``(points_tuple, boxes, masks)`` with Optionals; for
    a static graph we always pass all three (the C++ side supplies zero-length /
    zero tensors when a prompt kind is unused). box shape is [B,4] xyxy absolute
    px; points are absolute px (x,y).
    """

    def __init__(self, handles):
        super().__init__()
        self.pe = handles.trk_prompt_encoder  # PromptEncoder

    def forward(
        self,
        point_coords: torch.Tensor,  # [B,P,2]
        point_labels: torch.Tensor,  # [B,P]
        boxes: torch.Tensor,         # [B,4] xyxy
        mask_input: torch.Tensor,    # [B,1,288,288]
    ):
        sparse, dense = self.pe(
            points=(point_coords, point_labels.long()),
            boxes=boxes,
            masks=mask_input,
        )
        return sparse, dense


# ---------------------------------------------------------------------------
# tracker-multiplex-decoder
# ---------------------------------------------------------------------------
class MultiplexDecoderWrapper(nn.Module):
    """image_embeddings[B,256,72,72], image_pe[1,256,72,72],
    high_res_feat_0[B,32,288,288], high_res_feat_1[B,64,144,144],
    extra_per_object_embeddings[B,16,256]
    -> masks, iou_pred, object_score_logits, sam_tokens_out.

    Wraps ``MultiplexMaskDecoder.forward`` (multiplex_mask_decoder.py:149) with
    ``multimask_output=True`` baked in (num_multimask_outputs=3, K=16 objects).
    ``high_res_features`` is the length-2 list expected by the decoder's
    high-res path (use_high_res_features_in_sam=True).
    """

    def __init__(self, handles, multiplex_count: int = 16):
        super().__init__()
        self.dec = handles.trk_mask_decoder  # MultiplexMaskDecoder
        self.multiplex_count = multiplex_count

    def forward(
        self,
        image_embeddings: torch.Tensor,           # [B,256,72,72]
        image_pe: torch.Tensor,                   # [1,256,72,72]
        high_res_feat_0: torch.Tensor,            # [B,32,288,288]
        high_res_feat_1: torch.Tensor,            # [B,64,144,144]
        extra_per_object_embeddings: torch.Tensor,  # [B,16,256]
    ):
        out = self.dec(
            image_embeddings=image_embeddings,
            image_pe=image_pe,
            multimask_output=True,
            high_res_features=[high_res_feat_0, high_res_feat_1],
            extra_per_object_embeddings=extra_per_object_embeddings,
        )
        masks = out["masks"]
        iou_pred = out["iou_pred"]
        object_score_logits = out["object_score_logits"]
        sam_tokens_out = out.get("sam_tokens_out", out.get("mask_tokens_out"))
        return masks, iou_pred, object_score_logits, sam_tokens_out
