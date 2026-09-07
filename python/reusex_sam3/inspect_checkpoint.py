# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Standalone checkpoint inspector — run this **once** the ``sam3.1_multiplex.pt``
download finishes to finalise the remap in ``load_native.py``.

It prints, in three sections:

  1. Top-level key prefixes of the checkpoint (how many keys under
     ``detector.`` / ``tracker.`` / ``sam3_model.`` / ``sam2_predictor.`` / …).
  2. A proposed remap table (checkpoint prefix → model prefix) and how many keys
     it would match against the empty model built by ``load_native``.
  3. Residual missing/unexpected keys after the diff-driven remap, so you can
     extend ``_REGEX_TRANSFORMS`` / ``_ALLOWLIST_PREFIXES`` if needed.

Usage::

    python -m reusex_sam3.inspect_checkpoint --checkpoint checkpoints/sam3.1_multiplex.pt

Section (1) works even without the ``sam3`` package installed (it only reads the
checkpoint). Sections (2)/(3) require ``sam3`` + torch so the empty model can be
built for the diff.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path


def _top_prefix(key: str, depth: int = 1) -> str:
    return ".".join(key.split(".")[:depth])


def inspect(checkpoint_path: str, prefix_depth: int = 1, build_model: bool = True) -> int:
    import torch

    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if "model" in ckpt and isinstance(ckpt["model"], dict):
        ckpt = ckpt["model"]
    keys = list(ckpt.keys())

    print("=" * 72)
    print(f"[1] checkpoint: {checkpoint_path}")
    print(f"    total tensors: {len(keys)}")
    print(f"    top-level prefixes (depth={prefix_depth}):")
    counts = Counter(_top_prefix(k, prefix_depth) for k in keys)
    for pref, n in counts.most_common():
        print(f"      {n:6d}  {pref}")

    # deeper breakdown for the two expected roots
    for root in ("detector", "tracker", "sam3_model", "sam2_predictor"):
        sub = [k for k in keys if k.startswith(root + ".")]
        if sub:
            sub_counts = Counter(_top_prefix(k[len(root) + 1:], 1) for k in sub)
            print(f"    under '{root}.' ({len(sub)}):")
            for pref, n in sub_counts.most_common(20):
                print(f"        {n:6d}  {pref}")

    if not build_model:
        return 0

    print("=" * 72)
    print("[2] building empty demo model for diff (requires sam3 + torch)...")
    try:
        from .load_native import build_demo_model, remap_state_dict
    except Exception as exc:  # pragma: no cover
        print(f"    could not import load_native: {exc}")
        print("    (install the sam3 package: pip install -e third_party/sam3)")
        return 1

    model = build_demo_model()
    model_keys = set(model.state_dict().keys())
    print(f"    model tensors: {len(model_keys)}")
    print("    model top-level prefixes:")
    mcounts = Counter(_top_prefix(k, prefix_depth) for k in model_keys)
    for pref, n in mcounts.most_common():
        print(f"      {n:6d}  {pref}")

    remapped, report = remap_state_dict(ckpt, model_keys)
    print("=" * 72)
    print("[2] proposed remap result:")
    print(f"    matched            : {report['matched']} / {len(keys)}")
    print(f"    unexpected dropped : {report['unexpected_dropped_count']}")
    print(f"    missing in ckpt    : {report['missing_in_ckpt_count']}")
    if report["regex_transforms_used"]:
        print("    regex transforms used (sample):")
        for src, dst in report["regex_transforms_used"][:10]:
            print(f"        {src}  ->  {dst}")

    print("=" * 72)
    print("[3] residuals (first 40 of each):")
    print("    -- unexpected in ckpt (no model home) --")
    for k in report["unexpected_dropped"][:40]:
        print(f"        {k}")
    print("    -- missing in ckpt (model wants, ckpt lacks) --")
    for k in report["missing_in_ckpt"][:40]:
        print(f"        {k}")

    out = Path(checkpoint_path).resolve().parent / "inspect_report.json"
    import json

    with open(out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n[wrote] {out}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", required=True, help="Path to sam3.1_multiplex.pt")
    ap.add_argument("--prefix-depth", type=int, default=1)
    ap.add_argument(
        "--no-model",
        action="store_true",
        help="Only dump checkpoint prefixes (section 1); skip the model diff.",
    )
    args = ap.parse_args(argv)
    return inspect(
        args.checkpoint,
        prefix_depth=args.prefix_depth,
        build_model=not args.no_model,
    )


if __name__ == "__main__":
    raise SystemExit(main())
