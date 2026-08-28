#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# File the issue drafts in .github/issue-drafts/ as GitHub issues.
# Draft format: line 1 "title: ...", line 2 "labels: a, b", rest = body.
#
# Usage:
#   scripts/file-issues.sh --dry-run     # show what would be created
#   scripts/file-issues.sh               # create labels + issues
#
# Successfully filed drafts are moved to .github/issue-drafts/filed/ so the
# script is safe to re-run after partial failures.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
draft_dir="$repo_root/.github/issue-drafts"
dry_run="${1:-}"

# Label set (idempotent creation).
declare -A label_colors=(
  [infrastructure]="0e8a16"
  [testing]="0e8a16"
  [data-model]="d93f0b"
  [robustness]="b60205"
  [refactor]="1d76db"
  [build-speed]="1d76db"
  [research]="5319e7"
  [epic]="5319e7"
  [phase-0]="fbca04"
  [phase-1]="fbca04"
  [phase-2]="fbca04"
  [phase-3]="fbca04"
)

if [[ "$dry_run" != "--dry-run" ]]; then
  for label in "${!label_colors[@]}"; do
    gh label create "$label" --color "${label_colors[$label]}" --force >/dev/null
  done
  echo "==> labels ensured"
  mkdir -p "$draft_dir/filed"
fi

shopt -s nullglob
for draft in "$draft_dir"/*.md; do
  title="$(sed -n '1s/^title: //p' "$draft")"
  labels="$(sed -n '2s/^labels: //p' "$draft" | tr -d ' ')"
  if [[ -z "$title" ]]; then
    echo "skip (no title): $draft" >&2
    continue
  fi
  if [[ "$dry_run" == "--dry-run" ]]; then
    echo "would create: [$labels] $title"
  else
    body="$(tail -n +3 "$draft")"
    url="$(gh issue create --title "$title" --label "$labels" --body "$body")"
    echo "created: $url  ($(basename "$draft"))"
    mv "$draft" "$draft_dir/filed/"
  fi
done
