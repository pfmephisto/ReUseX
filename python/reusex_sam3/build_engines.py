# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Build TensorRT engines from the exported ONNX graphs by shelling out to
``trtexec`` (one invocation per engine).

Defaults to FP16, **except** for the engines listed in ``FP32_ENGINES`` (the
bf16-native ``vision-encoder``), which are always forced to a pure fp32 build —
fp16 silently corrupts the ViT trunk into all-background output. No separate
manual pass is needed: ``make engines`` produces a correct set.

Dynamic-shape engines get concrete ``--minShapes`` / ``--optShapes`` /
``--maxShapes`` derived from the contract. Engines land in the output model dir
(default ``engines/``) as ``<engine>.engine``.

``--int8-vision`` is a documented stub: true INT8 for the vision encoder should
go through NVIDIA TensorRT Model-Optimizer ONNX PTQ (see ``ptq_vision.py``),
producing a Q/DQ-annotated ONNX that trtexec then builds with ``--int8``.
"""

from __future__ import annotations

import argparse
import json
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

# Engines that MUST be built in pure fp32 — never fp16 (nor bf16/int8).
#
# The SAM 3.1 ViT-L trunk is bf16-native (the native model runs it under
# autocast). bf16 keeps fp32's 8-bit exponent; TensorRT fp16 has only 5 exponent
# bits, so the ViT activations overflow and the features become garbage —
# cosine ~0.33 against the fp32 reference (which scores 1.000). Downstream that
# surfaces as *all-background* annotation output with no error anywhere.
#
# It is not fixable with trtexec flags: the trunk is Myelin-fused into
# ForeignNodes, so `--precisionConstraints=obey --layerPrecisions=...:fp32` does
# not penetrate the fusion, and the fused RoPE node has no bf16 or int8 tactic.
# See docs/sam3.1-export-guide.md section 6.3.
FP32_ENGINES = {"vision-encoder"}

# The fp32 ViT only builds with a modest workspace; the default 8 GiB pool makes
# the builder pick tactics that fail on the fused RoPE node.
FP32_WORKSPACE_MB = 4096

# ...and it only builds at a fixed batch of 1 — a max batch > 1 fails on that
# same fused RoPE node. Harmless for inference: the C++ probes the engine's max
# image batch and chunks its input to match (Sam3::forward), and the SAM 3.1
# video path is batch-1 by construction.
FP32_SHAPE_OVERRIDES = {
    "vision-encoder": {
        "images": ((1, 3, 1008, 1008), (1, 3, 1008, 1008), (1, 3, 1008, 1008)),
    },
}


def _fmt(shapes: dict, which: int) -> str:
    # which: 0=min, 1=opt, 2=max
    parts = []
    for name, triple in shapes.items():
        dims = "x".join(str(d) for d in triple[which])
        parts.append(f"{name}:{dims}")
    return ",".join(parts)


def _resolve_engine(
    engine: str, fp16: bool, int8_vision: bool, workspace_mb: int
) -> dict:
    """Resolve one engine's effective build recipe (precision, workspace,
    shapes) for the given CLI choices. Shared by build_engine() and the
    engine-build.json emitter, so the recipe written next to the ONNX is by
    construction the one trtexec was given."""
    force_fp32 = engine in FP32_ENGINES and not (
        int8_vision and engine == "vision-encoder"
    )
    shapes = dict(SHAPE_PROFILES.get(engine, {}))
    if force_fp32:
        fp16 = False
        workspace_mb = min(workspace_mb, FP32_WORKSPACE_MB)
        if engine in FP32_SHAPE_OVERRIDES:
            shapes = {**shapes, **FP32_SHAPE_OVERRIDES[engine]}
    return {
        "force_fp32": force_fp32,
        "precision": "fp16" if fp16 else "fp32",
        "workspace_mb": workspace_mb,
        "shapes": shapes,
    }


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

    # Engines in FP32_ENGINES override the caller's precision/workspace/shape
    # choices: fp16 does not merely lose accuracy there, it silently produces a
    # broken engine (see the FP32_ENGINES comment). --int8-vision still wins for
    # the vision encoder, since that path is an explicit opt-in experiment.
    recipe = _resolve_engine(engine, fp16, int8_vision, workspace_mb)
    force_fp32 = recipe["force_fp32"]
    if force_fp32 and fp16:
        print(
            f"[fp32] {engine} is bf16-native; forcing a pure fp32 build "
            f"(fp16 corrupts it to all-background output) with "
            f"workspace:{recipe['workspace_mb']}"
        )
    fp16 = recipe["precision"] == "fp16"
    workspace_mb = recipe["workspace_mb"]

    cmd = [
        trtexec or "trtexec",
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        f"--memPoolSize=workspace:{workspace_mb}",
    ]
    shapes = recipe["shapes"]
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

# The default workspace for fp16 engines (mirrors build_engine()'s default).
DEFAULT_WORKSPACE_MB = 8192

# The canonical checked-in copy, shipped in the model bundle and read by the
# native C++ EngineBuilder (see EngineBuildProfiles / engine-build.json).
DEFAULT_PROFILES_PATH = Path(__file__).resolve().parent / "engine-build.json"


def _resolved_engine_profile(
    engine: str,
    fp16: bool = True,
    workspace_mb: int = DEFAULT_WORKSPACE_MB,
) -> dict:
    """One engine's engine-build.json entry — with the fp32-vision-encoder
    rule and its workspace/shape overrides already applied. With the defaults
    this is the canonical recipe (the SINGLE SOURCE OF TRUTH the C++ side
    consumes); a --no-fp16 / --workspace-mb run resolves to what it really
    built. INT8 is not expressible in schema v1, so it has no entry here."""
    r = _resolve_engine(engine, fp16, False, workspace_mb)
    shapes_json = {
        name: {
            "min": list(triple[0]),
            "opt": list(triple[1]),
            "max": list(triple[2]),
        }
        for name, triple in r["shapes"].items()
    }
    return {
        "precision": r["precision"],
        "workspace_mb": r["workspace_mb"],
        "shapes": shapes_json,
    }


def build_profiles(
    fp16: bool = True, workspace_mb: int = DEFAULT_WORKSPACE_MB
) -> dict:
    """The full engine-build.json document (schema v1) resolved from the tables
    above. Always lists every engine: the C++ builder needs the recipe of each
    engine it may build, even one a restricted ``--engines`` run skipped (whose
    recipe is unaffected by that restriction)."""
    return {
        "_comment": (
            "SINGLE SOURCE OF TRUTH for SAM 3.1 TensorRT engine builds. Emitted "
            "by python/reusex_sam3/build_engines.py from its in-code tables "
            "(SHAPE_PROFILES / FP32_ENGINES / FP32_WORKSPACE_MB / "
            "FP32_SHAPE_OVERRIDES) with the fp32-vision-encoder rule already "
            "resolved, and shipped in the model bundle. Read verbatim by both "
            "the python trtexec driver and the native C++ EngineBuilder "
            "(libs/reusex/.../tensor_rt/common/EngineBuildProfiles.*). Do not "
            "hand-edit: run `python -m reusex_sam3.build_engines "
            "--emit-profiles`."
        ),
        "schema_version": 1,
        "engines": {
            e: _resolved_engine_profile(e, fp16, workspace_mb) for e in ALL_ENGINES
        },
    }


def write_profiles(
    path: Path, fp16: bool = True, workspace_mb: int = DEFAULT_WORKSPACE_MB
) -> Path:
    """Write engine-build.json to `path` (JSON has no comment syntax for an SPDX
    header, so a REUSE `.license` sidecar is written next to it)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(build_profiles(fp16, workspace_mb), f, indent=2)
        f.write("\n")
    license_path = path.with_suffix(path.suffix + ".license")
    # REUSE-IgnoreStart
    with open(license_path, "w") as f:
        f.write(
            "SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen\n\n"
            "SPDX-License-Identifier: GPL-3.0-or-later\n"
        )
    # REUSE-IgnoreEnd
    print(f"[profiles] wrote {path}")
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--onnx-dir", default=str(DEFAULT_ONNX_DIR))
    ap.add_argument("--engine-dir", default=str(DEFAULT_ENGINE_DIR))
    ap.add_argument("--engines", nargs="*", default=ALL_ENGINES)
    ap.add_argument(
        "--no-fp16",
        action="store_true",
        help="Build every engine in fp32. Engines in FP32_ENGINES are fp32 "
        "regardless of this flag.",
    )
    ap.add_argument(
        "--int8-vision",
        action="store_true",
        help="Build the vision encoder in INT8 (needs a Q/DQ ONNX from ptq_vision.py).",
    )
    ap.add_argument("--workspace-mb", type=int, default=8192)
    ap.add_argument("--dry-run", action="store_true", help="Print trtexec cmds only.")
    ap.add_argument(
        "--emit-profiles",
        nargs="?",
        const=str(DEFAULT_PROFILES_PATH),
        default=None,
        metavar="PATH",
        help="Write engine-build.json (the shared shape/precision recipe the "
        "C++ EngineBuilder reads) and exit. Defaults to the checked-in "
        f"{DEFAULT_PROFILES_PATH.name} when no path is given.",
    )
    args = ap.parse_args(argv)

    if args.emit_profiles is not None:
        write_profiles(Path(args.emit_profiles))
        return 0

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

    # Ship the recipe that was actually used alongside the ONNX so the model
    # bundle is self-describing (the C++ side reads engine-build.json from the
    # ONNX dir, falling back to its embedded canonical copy when absent).
    recipe_path = Path(args.onnx_dir) / "engine-build.json"
    if args.dry_run:
        print(f"[profiles] dry run: not writing {recipe_path}")
    elif args.int8_vision:
        # Schema v1 has no int8 precision, so the C++ builder could only
        # rebuild these engines differently from what trtexec just did.
        print(
            f"[profiles] WARNING: --int8-vision is not expressible in "
            f"engine-build.json; not writing {recipe_path} (the C++ builder "
            f"will use its canonical fp16/fp32 recipe)"
        )
    else:
        write_profiles(
            recipe_path, fp16=not args.no_fp16, workspace_mb=args.workspace_mb
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
