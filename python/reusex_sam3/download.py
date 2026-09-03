# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Download the gated ``facebook/sam3.1`` checkpoint + tokenizer assets.

The ``sam3.1`` model is a **gated** HuggingFace repo: you must accept the
licence at https://huggingface.co/facebook/sam3.1 and be authenticated. The
token is taken (in order) from:

  1. the ``--token`` CLI argument,
  2. the ``HF_TOKEN`` / ``HUGGING_FACE_HUB_TOKEN`` env var,
  3. the token cached by ``huggingface-cli login`` (``~/.cache/huggingface``).

This script is idempotent: ``hf_hub_download`` reuses the local HF cache, so if
an interactive ``hf download facebook/sam3.1`` has already fetched the files,
this will resolve to the cached paths without re-downloading. You can also point
directly at an already-downloaded checkpoint with ``--checkpoint /path.pt`` and
this script will only verify / copy it into ``python/checkpoints/``.

Files fetched:
  * ``sam3.1_multiplex.pt``  (~3.5 GB) — the merged multiplex checkpoint
  * ``config.json``
  * BPE tokenizer vocab — the native ``sam3`` package ships it as
    ``sam3/assets/bpe_simple_vocab_16e6.txt.gz``; SAM 3.1 uses the same vocab.
    We resolve it from the installed ``sam3`` package (preferred) and fall back
    to the repo asset if present.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
from pathlib import Path
from typing import Optional

REPO_ID = "facebook/sam3.1"
CKPT_NAME = "sam3.1_multiplex.pt"
CONFIG_NAME = "config.json"
BPE_ASSET = "bpe_simple_vocab_16e6.txt.gz"

# Fill this in once known (see inspect_checkpoint.py / download output) to enable
# a hard integrity assert. Leave None to skip the check.
CKPT_SHA256: Optional[str] = None

DEFAULT_CKPT_DIR = Path(__file__).resolve().parent.parent / "checkpoints"


def _resolve_token(cli_token: Optional[str]) -> Optional[str]:
    if cli_token:
        return cli_token
    for env in ("HF_TOKEN", "HUGGING_FACE_HUB_TOKEN"):
        if os.environ.get(env):
            return os.environ[env]
    # None → huggingface_hub will fall back to the cached login token.
    return None


def _sha256(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def _copy_into(src: Path, dst_dir: Path) -> Path:
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / src.name
    if dst.resolve() == src.resolve():
        return dst
    # symlink to avoid duplicating a 3.5 GB blob; fall back to copy if unsupported
    try:
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        dst.symlink_to(src.resolve())
    except OSError:
        shutil.copy2(src, dst)
    return dst


def find_bpe_path() -> Optional[Path]:
    """Locate the BPE vocab shipped with the installed ``sam3`` package."""
    try:
        import pkg_resources  # type: ignore

        p = Path(pkg_resources.resource_filename("sam3", f"assets/{BPE_ASSET}"))
        if p.exists():
            return p
    except Exception:
        pass
    # Fall back: search a checked-out native repo next to third_party/.
    here = Path(__file__).resolve().parent.parent
    for cand in (here / "third_party" / "sam3" / "sam3" / "assets" / BPE_ASSET,):
        if cand.exists():
            return cand
    return None


def download(
    checkpoint: Optional[str] = None,
    token: Optional[str] = None,
    ckpt_dir: Path = DEFAULT_CKPT_DIR,
    assert_sha256: bool = True,
) -> dict:
    """Return a dict of resolved local paths: {checkpoint, config, bpe}."""
    ckpt_dir = Path(ckpt_dir)
    out: dict = {}

    if checkpoint is not None:
        ckpt_path = Path(checkpoint).expanduser().resolve()
        if not ckpt_path.exists():
            raise FileNotFoundError(f"--checkpoint not found: {ckpt_path}")
        out["checkpoint"] = _copy_into(ckpt_path, ckpt_dir)
    else:
        try:
            from huggingface_hub import hf_hub_download
            from huggingface_hub.utils import GatedRepoError, HfHubHTTPError
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "huggingface_hub is required. Install with:\n"
                "    pip install huggingface_hub"
            ) from exc

        tok = _resolve_token(token)
        try:
            cfg = hf_hub_download(repo_id=REPO_ID, filename=CONFIG_NAME, token=tok)
            ckpt = hf_hub_download(repo_id=REPO_ID, filename=CKPT_NAME, token=tok)
        except GatedRepoError as exc:  # pragma: no cover - network
            raise SystemExit(
                f"\n[gated] {REPO_ID} is a gated repo.\n"
                "  1. Accept the licence at https://huggingface.co/facebook/sam3.1\n"
                "  2. Authenticate: `huggingface-cli login` or set HF_TOKEN=...\n"
                f"  underlying error: {exc}"
            ) from exc
        except HfHubHTTPError as exc:  # pragma: no cover - network
            raise SystemExit(f"[hf] download failed: {exc}") from exc

        out["config"] = _copy_into(Path(cfg), ckpt_dir)
        out["checkpoint"] = _copy_into(Path(ckpt), ckpt_dir)

    # Integrity check (optional / best-effort).
    ckpt_final = Path(out["checkpoint"]).resolve()
    size_gb = ckpt_final.stat().st_size / 1e9
    if not (2.5 < size_gb < 5.0):
        print(
            f"[warn] {ckpt_final.name} is {size_gb:.2f} GB, expected ~3.5 GB. "
            "This may be a truncated download.",
            file=sys.stderr,
        )
    if assert_sha256 and CKPT_SHA256:
        got = _sha256(ckpt_final)
        if got != CKPT_SHA256:
            raise SystemExit(
                f"[sha256] mismatch for {ckpt_final.name}:\n"
                f"  expected {CKPT_SHA256}\n  got      {got}"
            )
        print(f"[ok] sha256 verified: {got}")

    bpe = find_bpe_path()
    if bpe is not None:
        out["bpe"] = bpe
    else:
        print(
            "[warn] BPE vocab not found. Install the sam3 package "
            "(`pip install -e third_party/sam3`) so its assets are importable.",
            file=sys.stderr,
        )

    print("[resolved]")
    for k, v in out.items():
        print(f"  {k:12s} {v}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--checkpoint",
        default=None,
        help="Path to an already-downloaded sam3.1_multiplex.pt (skips HF download).",
    )
    ap.add_argument("--token", default=None, help="HuggingFace access token.")
    ap.add_argument(
        "--ckpt-dir",
        default=str(DEFAULT_CKPT_DIR),
        help="Where to place resolved files (default: python/checkpoints/).",
    )
    ap.add_argument(
        "--no-sha256", action="store_true", help="Skip the sha256 integrity assert."
    )
    args = ap.parse_args(argv)
    download(
        checkpoint=args.checkpoint,
        token=args.token,
        ckpt_dir=Path(args.ckpt_dir),
        assert_sha256=not args.no_sha256,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
