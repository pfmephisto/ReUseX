# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Export the tracker engines to ONNX (opset 17, dynamo=False) and emit
``onnx/tracker-meta.json``.

Writes::

  onnx/tracker-memory-encoder.onnx
  onnx/tracker-memory-attention.onnx
  onnx/tracker-prompt-encoder.onnx
  onnx/tracker-multiplex-decoder.onnx
  onnx/tracker-meta.json

The memory-attention engine is exported with FIXED memory-bank shapes (no
dynamic per-frame control flow); its dynamic axes only cover batch and the
memory length (the C++ side always right-pads to
``mem_bank_max * mem_tokens_per_frame``). For fp16 builds the additive mask is
clamped to -1e4 (see fixes.NEG_INF_FP16); the exported graph itself carries no
mask add (the decoupled RoPE encoder does not consume a key-padding mask — see
MemoryAttentionWrapper docstring), so the clamp only matters if a future variant
reintroduces a float mask.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import ENGINE_IO_CONTRACT, TRACKER_META
from .fixes import apply_all
from .load_native import load
from .wrappers_tracker import (
    MemoryAttentionWrapper,
    MemoryEncoderWrapper,
    MultiplexDecoderWrapper,
    PromptEncoderWrapper,
)

OPSET = 17
DEFAULT_ONNX_DIR = Path(__file__).resolve().parent.parent / "onnx"

MEM_TOKENS = TRACKER_META["mem_tokens_per_frame"]
MEM_MAX = TRACKER_META["mem_bank_max"]
MEM_LEN = MEM_TOKENS * MEM_MAX  # fixed opt memory length for tracing
MULTIPLEX = TRACKER_META["multiplex_count"]


def _names_axes(engine: str):
    c = ENGINE_IO_CONTRACT[engine]
    return list(c["inputs"].keys()), list(c["outputs"].keys()), c["dynamic_axes"]


def _export(module, args, engine: str, onnx_dir: Path):
    import torch

    in_names, out_names, dyn = _names_axes(engine)
    out_path = onnx_dir / f"{engine}.onnx"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    module.eval()
    with torch.no_grad(), apply_all():
        torch.onnx.export(
            module,
            args,
            str(out_path),
            input_names=in_names,
            output_names=out_names,
            dynamic_axes=dyn,
            opset_version=OPSET,
            do_constant_folding=True,
            dynamo=False,
        )
    print(f"[onnx] wrote {out_path}")
    return out_path


def _example_inputs(engine: str):
    import torch

    if engine == "tracker-memory-encoder":
        vision_feat = torch.randn(1, 256, 72, 72)
        # K == multiplex_count (16) per-object mask logits. The extra 16
        # conditioning channels the SimpleMaskEncoder's 32-in-chan downsampler
        # wants are derived internally (condition_as_mask_input=True).
        K = MULTIPLEX
        pred_mask = torch.randn(K, 1, 1008, 1008)
        obj_scores = torch.randn(K, 1)
        return (vision_feat, pred_mask, obj_scores)
    if engine == "tracker-memory-attention":
        cur = torch.randn(MEM_TOKENS, 1, 256)     # current frame tokens [HW,B,C]
        cur_pos = torch.randn(MEM_TOKENS, 1, 256)
        mem = torch.randn(MEM_LEN, 1, 256)
        mem_pos = torch.randn(MEM_LEN, 1, 256)
        mem_mask = torch.zeros(1, MEM_LEN, dtype=torch.bool)
        return (cur, cur_pos, mem, mem_pos, mem_mask)
    if engine == "tracker-prompt-encoder":
        pts = torch.rand(1, 2, 2) * 1008.0
        labels = torch.ones(1, 2, dtype=torch.long)
        boxes = torch.tensor([[100.0, 100.0, 500.0, 500.0]])
        mask_in = torch.randn(1, 1, 288, 288)
        return (pts, labels, boxes, mask_in)
    if engine == "tracker-multiplex-decoder":
        img_emb = torch.randn(1, 256, 72, 72)
        img_pe = torch.randn(1, 256, 72, 72)
        hr0 = torch.randn(1, 32, 288, 288)
        hr1 = torch.randn(1, 64, 144, 144)
        extra = torch.randn(1, MULTIPLEX, 256)
        return (img_emb, img_pe, hr0, hr1, extra)
    raise ValueError(engine)


def write_meta(onnx_dir: Path):
    onnx_dir.mkdir(parents=True, exist_ok=True)
    meta_path = onnx_dir / "tracker-meta.json"
    with open(meta_path, "w") as f:
        json.dump(TRACKER_META, f, indent=2)
    print(f"[meta] wrote {meta_path}")


def export_all(checkpoint: str, onnx_dir: Path = DEFAULT_ONNX_DIR, only=None):
    handles = load(checkpoint)
    builders = {
        "tracker-memory-encoder": lambda h: MemoryEncoderWrapper(h),
        "tracker-memory-attention": lambda h: MemoryAttentionWrapper(h),
        "tracker-prompt-encoder": lambda h: PromptEncoderWrapper(h),
        "tracker-multiplex-decoder": lambda h: MultiplexDecoderWrapper(h, MULTIPLEX),
    }
    for engine, build in builders.items():
        if only and engine not in only:
            continue
        module = build(handles)
        _export(module, _example_inputs(engine), engine, onnx_dir)
    write_meta(onnx_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", required=True, help="Path to sam3.1_multiplex.pt")
    ap.add_argument("--onnx-dir", default=str(DEFAULT_ONNX_DIR))
    ap.add_argument("--only", nargs="*", default=None)
    args = ap.parse_args(argv)
    export_all(args.checkpoint, Path(args.onnx_dir), only=args.only)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
