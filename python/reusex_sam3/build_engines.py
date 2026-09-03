# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Build TensorRT engines from the exported ONNX graphs by shelling out to
``trtexec`` (one invocation per engine).

Defaults to FP16. Dynamic-shape engines get concrete ``--minShapes`` /
``--optShapes`` / ``--maxShapes`` derived from the contract. Engines land in the
output model dir (default ``engines/``) as ``<engine>.engine``.

``--int8-vision`` is a documented stub: true INT8 for the vision encoder should
go through NVIDIA TensorRT Model-Optimizer ONNX PTQ (see ``ptq_vision.py``),
producing a Q/DQ-annotated ONNX that trtexec then builds with ``--int8``.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

DEFAULT_ONNX_DIR = Path(__file__).resolve().parent.parent / "onnx"
DEFAULT_ENGINE_DIR = Path(__file__).resolve().parent.parent / "engines"

# Per-engine min/opt/max shape profiles. Format: {input_name: (min, opt, max)}.
# Static inputs are given identical min==opt==max. Batch B=1..4, N boxes 1..64,
# prompt L 1..300 (200 queries + text/geo), memory length fixed to the bank size.
_MEM_TOKENS = 5184
_MEM_LEN = _MEM_TOKENS * 7  # mem_bank_max * mem_tokens_per_frame

SHAPE_PROFILES = {
    "vision-encoder": {
        "images": ((1, 3, 1008, 1008), (1, 3, 1008, 1008), (4, 3, 1008, 1008)),
    },
    "text-encoder": {
        "input_ids": ((1, 32), (1, 32), (4, 32)),
        "attention_mask": ((1, 32), (1, 32), (4, 32)),
    },
    "geometry-encoder": {
        # num_boxes is baked (constant-folded) into the attention head-reshape,
        # so it is fixed at GEOM_NUM_BOXES=8 (matches export_detector). Only the
        # image batch is dynamic.
        "input_boxes": ((1, 8, 4), (1, 8, 4), (4, 8, 4)),
        "input_boxes_labels": ((1, 8), (1, 8), (4, 8)),
        "fpn_feat_2": ((1, 256, 72, 72), (1, 256, 72, 72), (4, 256, 72, 72)),
        "fpn_pos_2": ((1, 256, 72, 72), (1, 256, 72, 72), (4, 256, 72, 72)),
    },
    "decoder": {
        "fpn_feat_0": ((1, 256, 288, 288), (1, 256, 288, 288), (4, 256, 288, 288)),
        "fpn_feat_1": ((1, 256, 144, 144), (1, 256, 144, 144), (4, 256, 144, 144)),
        "fpn_feat_2": ((1, 256, 72, 72), (1, 256, 72, 72), (4, 256, 72, 72)),
        "fpn_pos_2": ((1, 256, 72, 72), (1, 256, 72, 72), (4, 256, 72, 72)),
        # prompt_len is fixed at 32 (the text encoder emits 32 tokens and the
        # attention head-reshape bakes this constant); only batch is dynamic.
        "prompt_features": ((1, 32, 256), (1, 32, 256), (4, 32, 256)),
        "prompt_mask": ((1, 32), (1, 32), (4, 32)),
    },
    "tracker-memory-encoder": {
        # num_objects is baked to multiplex_count(16) by the SimpleMaskEncoder's
        # 32-in-channel downsampler, so the engine is fixed at 16 objects. The
        # C++ feeds 16 mask channels (aggregate mask in channel 0, rest zero).
        "vision_feat": ((1, 256, 72, 72), (1, 256, 72, 72), (1, 256, 72, 72)),
        "pred_mask": ((16, 1, 1008, 1008), (16, 1, 1008, 1008), (16, 1, 1008, 1008)),
        "object_score_logits": ((16, 1), (16, 1), (16, 1)),
    },
    "tracker-memory-attention": {
        "current_feat": ((_MEM_TOKENS, 1, 256),) * 3,
        "current_pos": ((_MEM_TOKENS, 1, 256),) * 3,
        # memory length varies from a single frame's worth up to the full bank
        "memory": ((_MEM_TOKENS, 1, 256), (_MEM_LEN, 1, 256), (_MEM_LEN, 1, 256)),
        "memory_pos": ((_MEM_TOKENS, 1, 256), (_MEM_LEN, 1, 256), (_MEM_LEN, 1, 256)),
        "memory_mask": ((1, _MEM_TOKENS), (1, _MEM_LEN), (1, _MEM_LEN)),
    },
    "tracker-prompt-encoder": {
        "point_coords": ((1, 1, 2), (1, 2, 2), (1, 8, 2)),
        "point_labels": ((1, 1), (1, 2), (1, 8)),
        "boxes": ((1, 4), (1, 4), (1, 4)),
        "mask_input": ((1, 1, 288, 288),) * 3,
    },
    "tracker-multiplex-decoder": {
        "image_embeddings": ((1, 256, 72, 72),) * 3,
        "image_pe": ((1, 256, 72, 72),) * 3,
        "high_res_feat_0": ((1, 32, 288, 288),) * 3,
        "high_res_feat_1": ((1, 64, 144, 144),) * 3,
        "extra_per_object_embeddings": ((1, 16, 256),) * 3,
    },
}

# Engines that benefit from stronger fp16 mask-overflow protection. For
# memory-attention keep additive masks clamped (-1e4) in the graph (fixes.py).
FP16_MASK_CLAMP_ENGINES = {"tracker-memory-attention", "decoder"}


def _fmt(shapes: dict, which: int) -> str:
    # which: 0=min, 1=opt, 2=max
    parts = []
    for name, triple in shapes.items():
        dims = "x".join(str(d) for d in triple[which])
        parts.append(f"{name}:{dims}")
    return ",".join(parts)


def build_engine(
    engine: str,
    onnx_dir: Path,
    engine_dir: Path,
    fp16: bool = True,
    int8_vision: bool = False,
    workspace_mb: int = 8192,
    extra_args=None,
    dry_run: bool = False,
) -> Path:
    trtexec = shutil.which("trtexec")
    if trtexec is None and not dry_run:
        raise FileNotFoundError(
            "trtexec not found on PATH. Enter the CUDA/TensorRT dev shell first."
        )
    onnx_path = onnx_dir / f"{engine}.onnx"
    engine_path = engine_dir / f"{engine}.engine"
    engine_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        trtexec or "trtexec",
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        f"--memPoolSize=workspace:{workspace_mb}",
    ]
    shapes = SHAPE_PROFILES.get(engine)
    if shapes:
        cmd += [
            f"--minShapes={_fmt(shapes, 0)}",
            f"--optShapes={_fmt(shapes, 1)}",
            f"--maxShapes={_fmt(shapes, 2)}",
        ]
    if int8_vision and engine == "vision-encoder":
        # Expect a Q/DQ ONNX from ptq_vision.py at <engine>.int8.onnx
        int8_onnx = onnx_dir / f"{engine}.int8.onnx"
        if int8_onnx.exists():
            cmd[1] = f"--onnx={int8_onnx}"
        cmd.append("--int8")
        cmd.append("--fp16")  # allow fp16 fallback for non-quantised layers
    elif fp16:
        cmd.append("--fp16")
    if extra_args:
        cmd += list(extra_args)

    print("[trtexec]", " ".join(cmd))
    if dry_run:
        return engine_path
    subprocess.run(cmd, check=True)
    print(f"[engine] wrote {engine_path}")
    return engine_path


ALL_ENGINES = list(SHAPE_PROFILES.keys())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--onnx-dir", default=str(DEFAULT_ONNX_DIR))
    ap.add_argument("--engine-dir", default=str(DEFAULT_ENGINE_DIR))
    ap.add_argument("--engines", nargs="*", default=ALL_ENGINES)
    ap.add_argument("--no-fp16", action="store_true")
    ap.add_argument(
        "--int8-vision",
        action="store_true",
        help="Build the vision encoder in INT8 (needs a Q/DQ ONNX from ptq_vision.py).",
    )
    ap.add_argument("--workspace-mb", type=int, default=8192)
    ap.add_argument("--dry-run", action="store_true", help="Print trtexec cmds only.")
    args = ap.parse_args(argv)

    for engine in args.engines:
        build_engine(
            engine,
            Path(args.onnx_dir),
            Path(args.engine_dir),
            fp16=not args.no_fp16,
            int8_vision=args.int8_vision,
            workspace_mb=args.workspace_mb,
            dry_run=args.dry_run,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
