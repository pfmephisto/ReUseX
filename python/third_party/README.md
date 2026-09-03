<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# third_party — native SAM 3.1 source

The export pipeline imports the **native** `sam3` Python package to build the
real model and load the checkpoint. We deliberately do **not** vendor the full
Meta repo into this git tree; instead pin and clone it here.

## Pin

| field  | value |
|--------|-------|
| repo   | `https://github.com/facebookresearch/sam3` |
| branch | `sam3.1` |
| commit | `20dba30a35a497606b06cf241f5b5605ea10e77e` |

All of `load_native.py`, `wrappers_detector.py`, `wrappers_tracker.py` and
`fixes.py` were written against the class names, attribute paths and `forward`
signatures at this exact SHA. If you bump the pin, re-verify those references.

## Clone + install

```bash
cd python/third_party
git clone --branch sam3.1 https://github.com/facebookresearch/sam3.git sam3
cd sam3
git checkout 20dba30a35a497606b06cf241f5b5605ea10e77e

# install editable, without pulling a source torch build:
pip install -e . --no-build-isolation
```

`load_native.py` imports it as a normal top-level package (`import sam3`), so an
editable install (`pip install -e`) is the recommended way to make it
resolvable. Alternatively, add the clone to `PYTHONPATH`:

```bash
export PYTHONPATH="$PWD/python/third_party/sam3:$PYTHONPATH"
```

## Assets

The BPE tokenizer vocab (`bpe_simple_vocab_16e6.txt.gz`) ships inside the
package at `sam3/assets/`. `download.find_bpe_path()` resolves it via
`pkg_resources` (preferred) or falls back to
`third_party/sam3/sam3/assets/`.

## Why not vendor?

The repo is large and Meta-licensed; vendoring would bloat this GPL tree and
conflate licences. Pinning the SHA gives full reproducibility without copying
their sources into ours.
