# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# SAM 3.1 -> ONNX -> TensorRT export pipeline.
# Run inside a venv with prebuilt wheels (see requirements.txt); do NOT build
# torch from source (the machine OOMs compiling torch).

PY            ?= python
CHECKPOINT    ?= checkpoints/sam3.1_multiplex.pt
ONNX_DIR      ?= onnx
ENGINE_DIR    ?= engines
MODEL_DIR     ?= $(ENGINE_DIR)

.PHONY: all download load inspect export-detector export-tracker export engines verify drift clean help

help:
	@echo "targets: download load export-detector export-tracker engines verify all"
	@echo "vars:    CHECKPOINT=$(CHECKPOINT) ONNX_DIR=$(ONNX_DIR) ENGINE_DIR=$(ENGINE_DIR)"

## download the gated checkpoint + tokenizer assets into checkpoints/
download:
	$(PY) -m reusex_sam3.download

## load = inspect the checkpoint + write the diff-driven remap_report.json
load: inspect
inspect:
	$(PY) -m reusex_sam3.inspect_checkpoint --checkpoint $(CHECKPOINT)

## export the 4 detector engines to ONNX
export-detector:
	$(PY) -m reusex_sam3.export_detector --checkpoint $(CHECKPOINT) --onnx-dir $(ONNX_DIR)

## export the 4 tracker engines to ONNX + tracker-meta.json
export-tracker:
	$(PY) -m reusex_sam3.export_tracker --checkpoint $(CHECKPOINT) --onnx-dir $(ONNX_DIR)

export: export-detector export-tracker

## build all TensorRT engines (FP16) via trtexec
engines:
	$(PY) -m reusex_sam3.build_engines --onnx-dir $(ONNX_DIR) --engine-dir $(MODEL_DIR)

## 3-way parity harness (torch vs onnxruntime vs tensorrt)
verify:
	$(PY) -m reusex_sam3.verify --checkpoint $(CHECKPOINT) --onnx-dir $(ONNX_DIR) --engine-dir $(MODEL_DIR)

## 30-frame recurrent tracker drift test
drift:
	$(PY) -m reusex_sam3.verify --checkpoint $(CHECKPOINT) --onnx-dir $(ONNX_DIR) --engine-dir $(MODEL_DIR) --drift --engines

## full pipeline in order
all: download load export-detector export-tracker engines verify

clean:
	rm -rf $(ONNX_DIR) $(ENGINE_DIR) checkpoints/*.json __pycache__ reusex_sam3/__pycache__
