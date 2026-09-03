# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Export the four detector engines to ONNX (opset 17, dynamo=False).

Writes ``onnx/{vision-encoder,text-encoder,geometry-encoder,decoder}.onnx``.

Each export uses the contract's exact input/output names + dynamic_axes (from
``reusex_sam3.ENGINE_IO_CONTRACT``) and runs inside ``fixes.apply_all`` so the
ONNX-hostile native code paths (cuda pos-enc cache, complex RoPE, scatter,
list-roi_align) are patched for the duration of the trace.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from . import ENGINE_IO_CONTRACT
from .fixes import apply_all
from .load_native import load
from .wrappers_detector import (
    DecoderWrapper,
    GeometryEncoderWrapper,
    TextEncoderWrapper,
    VisionEncoderWrapper,
)

OPSET = 17
DEFAULT_ONNX_DIR = Path(__file__).resolve().parent.parent / "onnx"

# Fixed number of geometry (box) prompts baked into the geometry-encoder graph.
# The attention head-reshape constant-folds this value, so the engine is built
# at exactly this count (see build_engines.SHAPE_PROFILES["geometry-encoder"]).
GEOM_NUM_BOXES = 8


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
    """Fixed-shape example tensors for tracing (batch=1, small N/L)."""
    import torch

    if engine == "vision-encoder":
        return (torch.randn(1, 3, 1008, 1008),)
    if engine == "text-encoder":
        ids = torch.randint(1, 49407, (1, 32), dtype=torch.long)
        mask = torch.ones(1, 32, dtype=torch.long)
        return (ids, mask)
    if engine == "geometry-encoder":
        # num_boxes is baked as a constant into the attention head-reshape by the
        # TorchScript exporter (same failure mode as the decoder prompt_len), so
        # it must be fixed. We use GEOM_NUM_BOXES; a geometry-prompted decoder
        # must then be built with prompt_len = 32 + GEOM_NUM_BOXES + 1 (the
        # text-only path here uses prompt_len=32 and does not invoke geometry).
        n = GEOM_NUM_BOXES
        boxes = torch.rand(1, n, 4)          # N boxes cxcywh in [0,1]
        labels = torch.ones(1, n, dtype=torch.long)
        fpn2 = torch.randn(1, 256, 72, 72)
        pos2 = torch.randn(1, 256, 72, 72)
        return (boxes, labels, fpn2, pos2)
    if engine == "decoder":
        f0 = torch.randn(1, 256, 288, 288)
        f1 = torch.randn(1, 256, 144, 144)
        f2 = torch.randn(1, 256, 72, 72)
        p2 = torch.randn(1, 256, 72, 72)
        # L is baked as a constant into the attention head-reshape by the
        # TorchScript ONNX exporter, so it must match what the C++ sends: the
        # text encoder emits exactly 32 tokens (make_ids pads to 32), and the
        # text-prompted path uses no geometry, so prompt_len is always 32.
        L = 32
        prompt = torch.randn(1, L, 256)
        pmask = torch.zeros(1, L, dtype=torch.bool)
        return (f0, f1, f2, p2, prompt, pmask)
    raise ValueError(engine)


def export_all(checkpoint: str, onnx_dir: Path = DEFAULT_ONNX_DIR, only=None):
    handles = load(checkpoint)
    builders = {
        "vision-encoder": VisionEncoderWrapper,
        "text-encoder": TextEncoderWrapper,
        "geometry-encoder": GeometryEncoderWrapper,
        "decoder": DecoderWrapper,
    }
    for engine, cls in builders.items():
        if only and engine not in only:
            continue
        module = cls(handles)
        _export(module, _example_inputs(engine), engine, onnx_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", required=True, help="Path to sam3.1_multiplex.pt")
    ap.add_argument("--onnx-dir", default=str(DEFAULT_ONNX_DIR))
    ap.add_argument(
        "--only",
        nargs="*",
        default=None,
        help="Subset of engines to export (default: all four).",
    )
    args = ap.parse_args(argv)
    export_all(args.checkpoint, Path(args.onnx_dir), only=args.only)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
