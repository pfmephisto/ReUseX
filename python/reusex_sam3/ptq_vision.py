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


def quantize_vision_encoder(
    onnx_path: str,
    calibration_data: str,
    output_path: str,
    method: str = "entropy",
) -> str:  # pragma: no cover - stub
    raise NotImplementedError(
        "ptq_vision.quantize_vision_encoder is a documented stub. See module "
        "docstring for the nvidia-modelopt ONNX PTQ recipe. Provide a calibration "
        "set and wire in modelopt.onnx.quantization.quantize before use."
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--onnx", default="onnx/vision-encoder.onnx")
    ap.add_argument("--calib", required=False, help="Calibration data (.npz).")
    ap.add_argument("--out", default="onnx/vision-encoder.int8.onnx")
    ap.add_argument("--method", default="entropy", choices=["entropy", "minmax"])
    args = ap.parse_args(argv)
    quantize_vision_encoder(args.onnx, args.calib, args.out, method=args.method)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
