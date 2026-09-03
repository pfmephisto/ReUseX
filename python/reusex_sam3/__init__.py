# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
reusex_sam3 — reproducible SAM 3.1 (multiplex) → ONNX → TensorRT export pipeline.

This package converts Meta's ``facebook/sam3.1`` ``sam3.1_multiplex.pt`` checkpoint
into a set of ONNX graphs and TensorRT engines whose I/O names conform to the
shared engine-I/O contract that the ReUseX C++ side binds to (see ``README.md``
and the ``ENGINE_IO_CONTRACT`` dict below).

The pipeline is intentionally split into small, independently exportable
components ("engines"), each backed by a thin ``nn.Module`` wrapper around the
*real* native SAM 3.1 submodules (loaded from the ``sam3`` package on branch
``sam3.1``, pinned SHA ``20dba30a35a497606b06cf241f5b5605ea10e77e``).

Run order (see ``python/Makefile``)::

    download  → load  → export-detector → export-tracker → engines → verify

None of the modules import ``torch`` at package-import time so that
``ENGINE_IO_CONTRACT`` and metadata can be introspected on a machine without a
GPU / heavy deps installed.
"""

# The authoritative engine-I/O contract. The C++ side MUST bind to these exact
# tensor names. Kept here (dependency-free) so it can be imported and asserted
# against in verify.py, build_engines.py and, if desired, C++ codegen.
#
# dtype notes:
#   - all float tensors are fp32 in ONNX; TensorRT builds fp16 engines by
#     default (build_engines.py). The mask tensors passed into memory-attention
#     are bool.
#   - token id tensors are int64 (ONNX opset-17 Gather wants int64 indices).
ENGINE_IO_CONTRACT = {
    "vision-encoder": {
        "inputs": {
            "images": {"shape": ["B", 3, 1008, 1008], "dtype": "float32"},
        },
        "outputs": {
            "fpn_feat_0": {"shape": ["B", 256, 288, 288], "dtype": "float32"},
            "fpn_feat_1": {"shape": ["B", 256, 144, 144], "dtype": "float32"},
            "fpn_feat_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "fpn_pos_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
        },
        "dynamic_axes": {
            "images": {0: "batch"},
            "fpn_feat_0": {0: "batch"},
            "fpn_feat_1": {0: "batch"},
            "fpn_feat_2": {0: "batch"},
            "fpn_pos_2": {0: "batch"},
        },
    },
    "text-encoder": {
        "inputs": {
            "input_ids": {"shape": ["B", 32], "dtype": "int64"},
            "attention_mask": {"shape": ["B", 32], "dtype": "int64"},
        },
        "outputs": {
            # seq-first inside the model, but exported batch-first: [B, 32, 256]
            "text_features": {"shape": ["B", 32, 256], "dtype": "float32"},
            "text_mask": {"shape": ["B", 32], "dtype": "bool"},
        },
        "dynamic_axes": {
            "input_ids": {0: "batch"},
            "attention_mask": {0: "batch"},
            "text_features": {0: "batch"},
            "text_mask": {0: "batch"},
        },
    },
    "geometry-encoder": {
        "inputs": {
            "input_boxes": {"shape": ["B", "N", 4], "dtype": "float32"},  # cxcywh, normalised
            "input_boxes_labels": {"shape": ["B", "N"], "dtype": "int64"},
            "fpn_feat_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "fpn_pos_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
        },
        "outputs": {
            # geometry tokens = N boxes + 1 CLS token = N+1, exported batch-first
            "geometry_features": {"shape": ["B", "N+1", 256], "dtype": "float32"},
            "geometry_mask": {"shape": ["B", "N+1"], "dtype": "bool"},
        },
        "dynamic_axes": {
            "input_boxes": {0: "batch", 1: "num_boxes"},
            "input_boxes_labels": {0: "batch", 1: "num_boxes"},
            "fpn_feat_2": {0: "batch"},
            "fpn_pos_2": {0: "batch"},
            "geometry_features": {0: "batch", 1: "num_prompts"},
            "geometry_mask": {0: "batch", 1: "num_prompts"},
        },
    },
    "decoder": {
        "inputs": {
            "fpn_feat_0": {"shape": ["B", 256, 288, 288], "dtype": "float32"},
            "fpn_feat_1": {"shape": ["B", 256, 144, 144], "dtype": "float32"},
            "fpn_feat_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "fpn_pos_2": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "prompt_features": {"shape": ["B", "L", 256], "dtype": "float32"},
            "prompt_mask": {"shape": ["B", "L"], "dtype": "bool"},
        },
        "outputs": {
            "pred_masks": {"shape": ["B", 200, 288, 288], "dtype": "float32"},
            "pred_boxes": {"shape": ["B", 200, 4], "dtype": "float32"},  # cxcywh
            "pred_logits": {"shape": ["B", 200, 1], "dtype": "float32"},
            "presence_logits": {"shape": ["B", 1], "dtype": "float32"},
        },
        "dynamic_axes": {
            "fpn_feat_0": {0: "batch"},
            "fpn_feat_1": {0: "batch"},
            "fpn_feat_2": {0: "batch"},
            "fpn_pos_2": {0: "batch"},
            "prompt_features": {0: "batch", 1: "prompt_len"},
            "prompt_mask": {0: "batch", 1: "prompt_len"},
            "pred_masks": {0: "batch"},
            "pred_boxes": {0: "batch"},
            "pred_logits": {0: "batch"},
            "presence_logits": {0: "batch"},
        },
    },
    "tracker-memory-encoder": {
        "inputs": {
            # current top-level vision feature, BCHW (C=256, 72x72)
            "vision_feat": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            # per-object predicted mask logits at high res (K object channels, 1008x1008)
            "pred_mask": {"shape": ["K", 1, 1008, 1008], "dtype": "float32"},
            "object_score_logits": {"shape": ["K", 1], "dtype": "float32"},
        },
        "outputs": {
            "maskmem_features": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "maskmem_pos_enc": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
        },
        "dynamic_axes": {
            "vision_feat": {0: "batch"},
            "pred_mask": {0: "num_objects"},
            "object_score_logits": {0: "num_objects"},
            "maskmem_features": {0: "batch"},
            "maskmem_pos_enc": {0: "batch"},
        },
    },
    "tracker-memory-attention": {
        "inputs": {
            # current frame tokens, seq-first [HW, B, C] = [5184, B, 256]
            "current_feat": {"shape": [5184, "B", 256], "dtype": "float32"},
            "current_pos": {"shape": [5184, "B", 256], "dtype": "float32"},
            # fixed-length flattened memory bank, seq-first [M, B, C]
            "memory": {"shape": ["M", "B", 256], "dtype": "float32"},
            "memory_pos": {"shape": ["M", "B", 256], "dtype": "float32"},
            # boolean padding mask over the memory bank, batch-first [B, M]
            "memory_mask": {"shape": ["B", "M"], "dtype": "bool"},
        },
        "outputs": {
            "pix_feat_with_mem": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
        },
        "dynamic_axes": {
            "current_feat": {1: "batch"},
            "current_pos": {1: "batch"},
            "memory": {0: "mem_len", 1: "batch"},
            "memory_pos": {0: "mem_len", 1: "batch"},
            "memory_mask": {0: "batch", 1: "mem_len"},
            "pix_feat_with_mem": {0: "batch"},
        },
    },
    "tracker-prompt-encoder": {
        "inputs": {
            "point_coords": {"shape": ["B", "P", 2], "dtype": "float32"},  # absolute px (x,y)
            "point_labels": {"shape": ["B", "P"], "dtype": "int64"},
            "boxes": {"shape": ["B", 4], "dtype": "float32"},              # xyxy absolute px
            "mask_input": {"shape": ["B", 1, 288, 288], "dtype": "float32"},
        },
        "outputs": {
            "sparse_embeddings": {"shape": ["B", "S", 256], "dtype": "float32"},
            "dense_embeddings": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
        },
        "dynamic_axes": {
            "point_coords": {0: "batch", 1: "num_points"},
            "point_labels": {0: "batch", 1: "num_points"},
            "boxes": {0: "batch"},
            "mask_input": {0: "batch"},
            "sparse_embeddings": {0: "batch", 1: "num_sparse"},
            "dense_embeddings": {0: "batch"},
        },
    },
    "tracker-multiplex-decoder": {
        "inputs": {
            "image_embeddings": {"shape": ["B", 256, 72, 72], "dtype": "float32"},
            "image_pe": {"shape": [1, 256, 72, 72], "dtype": "float32"},
            "high_res_feat_0": {"shape": ["B", 32, 288, 288], "dtype": "float32"},
            "high_res_feat_1": {"shape": ["B", 64, 144, 144], "dtype": "float32"},
            "extra_per_object_embeddings": {"shape": ["B", 16, 256], "dtype": "float32"},
        },
        "outputs": {
            # multimask_output=True baked in → num_multimask_outputs (3) per object
            "masks": {"shape": ["B", 16, 3, 288, 288], "dtype": "float32"},
            "iou_pred": {"shape": ["B", 16, 3], "dtype": "float32"},
            "object_score_logits": {"shape": ["B", 16, 1], "dtype": "float32"},
            "sam_tokens_out": {"shape": ["B", 16, 3, 256], "dtype": "float32"},
        },
        "dynamic_axes": {
            "image_embeddings": {0: "batch"},
            "high_res_feat_0": {0: "batch"},
            "high_res_feat_1": {0: "batch"},
            "extra_per_object_embeddings": {0: "batch"},
            "masks": {0: "batch"},
            "iou_pred": {0: "batch"},
            "object_score_logits": {0: "batch"},
            "sam_tokens_out": {0: "batch"},
        },
    },
}

# Integer metadata describing the tracker memory bank; emitted verbatim as
# tracker-meta.json by export_tracker.py. The C++ side reads these to size its
# fixed memory bank and to lay out flattened memory tokens.
TRACKER_META = {
    "mem_bank_max": 7,          # num_maskmem: 1 cond frame + 6 recent frames
    "mem_tokens_per_frame": 5184,  # 72 * 72 spatial memory tokens per frame
    "mem_dim": 256,             # maskmem_backbone out_dim (== hidden_dim for multiplex)
    "feat_c": 256,              # top-level vision feature channels
    "feat_h": 72,               # top-level vision feature height (1008 / 14)
    "feat_w": 72,               # top-level vision feature width
    "multiplex_count": 16,      # objects per multiplex bucket
}

# Native repo pin (branch sam3.1). Recorded so third_party/ can be reconstructed.
NATIVE_REPO = "https://github.com/facebookresearch/sam3"
NATIVE_BRANCH = "sam3.1"
NATIVE_SHA = "20dba30a35a497606b06cf241f5b5605ea10e77e"

__all__ = [
    "ENGINE_IO_CONTRACT",
    "TRACKER_META",
    "NATIVE_REPO",
    "NATIVE_BRANCH",
    "NATIVE_SHA",
]
