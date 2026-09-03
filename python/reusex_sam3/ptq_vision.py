# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
INT8 post-training quantisation (PTQ) for the vision encoder — DOCUMENTED STUB.

The vision encoder (ViT-L, 1008x1008) dominates the SAM 3.1 latency budget, so
INT8 is the highest-value quantisation target. The recommended path is NVIDIA
TensorRT Model-Optimizer's ONNX PTQ, which inserts Q/DQ nodes into the exported
FP32 ONNX using a small calibration set; trtexec then builds with ``--int8``.

Pipeline (once weights + a calibration set of ~200-500 representative frames are
available)::

    1. Export the FP32 vision-encoder ONNX (export_detector.py).
    2. pip install nvidia-modelopt[onnx]
    3. Calibrate + insert Q/DQ:

         import modelopt.onnx.quantization as moq
         moq.quantize(
             onnx_path="onnx/vision-encoder.onnx",
             calibration_data=calib_npz,          # {"images": [N,3,1008,1008]}
             calibration_method="entropy",        # or "minmax"
             output_path="onnx/vision-encoder.int8.onnx",
             op_types_to_quantize=["Conv", "MatMul", "Gemm"],
             # keep LayerNorm / softmax / RoPE in fp16 for accuracy:
             nodes_to_exclude=[".*LayerNorm.*", ".*Softmax.*", ".*rope.*"],
         )

    4. build_engines.py --int8-vision  (picks up vision-encoder.int8.onnx).

Accuracy guardrail: re-run verify.py's vision cosine-similarity check
(cos >= 0.999 is unlikely for INT8; target cos >= 0.99 and IoU >= 0.95 on the
downstream decoder masks). If accuracy regresses, exclude the first/last blocks
from quantisation or fall back to FP16 for the vision encoder.

This module intentionally raises when invoked so the pipeline fails loudly until
a real calibration set is wired in.
"""

from __future__ import annotations

import argparse

import numpy as np

INPUT_SIZE = 1008


def build_calibration_set(project_db: str, out_npz: str, n: int = 200) -> tuple:
    """Sample ``n`` color frames from a ReUseX project DB and preprocess them to
    the vision-encoder input ([N,3,1008,1008] fp32, RGB, x/127.5 - 1), matching
    the C++ preprocess (norm alpha=1/127.5 beta=-1, SwapRB). Saved as .npz with
    key ``images``."""
    import sqlite3

    import cv2

    con = sqlite3.connect(project_db)
    rows = con.execute(
        "SELECT color FROM sensor_frames WHERE color IS NOT NULL"
    ).fetchall()
    if not rows:
        raise RuntimeError(f"no color frames in {project_db}")
    idx = np.unique(np.linspace(0, len(rows) - 1, min(n, len(rows))).astype(int))
    imgs = []
    for i in idx:
        img = cv2.imdecode(np.frombuffer(rows[i][0], np.uint8), cv2.IMREAD_COLOR)
        r = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        r = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 127.5 - 1.0
        imgs.append(np.transpose(r, (2, 0, 1)))
    arr = np.ascontiguousarray(np.stack(imgs).astype(np.float32))
    np.savez(out_npz, images=arr)
    print(f"[calib] wrote {out_npz} with {arr.shape}")
    return arr.shape


def quantize_vision_encoder(
    onnx_path: str,
    calibration_data: str,
    output_path: str,
    method: str = "entropy",
) -> str:
    """Insert INT8 Q/DQ into the FP32 vision-encoder ONNX via NVIDIA
    Model-Optimizer, calibrated on ``calibration_data`` (.npz with key
    ``images``). trtexec then builds it with ``--int8 --fp16``."""
    from modelopt.onnx.quantization import quantize

    data = np.load(calibration_data)
    calib = {"images": data["images"]}
    print(
        f"[ptq] quantizing {onnx_path} (int8/{method}) with "
        f"{calib['images'].shape[0]} calibration frames"
    )
    quantize(
        onnx_path=onnx_path,
        quantize_mode="int8",
        calibration_data=calib,
        calibration_method=method,
        output_path=output_path,
        # Calibrate on the GPU — the default ['cpu', ...] runs the ViT forward on
        # CPU at 1008x1008 and is orders of magnitude slower.
        calibration_eps=["cuda:0", "cpu"],
        # Keep the non-INT8 remainder in FP32, NOT the default fp16 autocast:
        # this ViT is bf16-native and fp16 corrupts it (cosine 0.33), and the
        # fp16 autocast pass also crashes on this graph. INT8 handles Conv/MatMul;
        # everything else stays fp32.
        high_precision_dtype="fp32",
        mha_accumulation_dtype="fp32",
        # Keep the RoPE attention in fp32: its fused node has no int8 TRT tactic
        # (the same Myelin ForeignNode that blocks fp16/bf16). Only Conv + MLP
        # MatMul/Gemm get int8.
        nodes_to_exclude=[".*attn.*", ".*rope.*", ".*Rope.*"],
    )
    print(f"[ptq] wrote {output_path}")
    return output_path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--onnx", default="onnx/vision-encoder.onnx")
    ap.add_argument("--calib", help="Calibration data (.npz with key 'images').")
    ap.add_argument("--out", default="onnx/vision-encoder.int8.onnx")
    ap.add_argument("--method", default="entropy", choices=["entropy", "minmax"])
    ap.add_argument(
        "--gen-calib",
        metavar="PROJECT.rux",
        help="Build a calibration .npz from a project DB (writes --calib), then exit.",
    )
    ap.add_argument("--n", type=int, default=200, help="Calibration frame count.")
    args = ap.parse_args(argv)

    if args.gen_calib:
        out = args.calib or "checkpoints/calib.npz"
        build_calibration_set(args.gen_calib, out, n=args.n)
        return 0

    if not args.calib:
        ap.error("--calib is required (or use --gen-calib to build one)")
    quantize_vision_encoder(args.onnx, args.calib, args.out, method=args.method)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
