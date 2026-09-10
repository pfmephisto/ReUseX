#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Parse GCC/Clang warnings from a cmake build log and sync them to GitHub
# issues. Creates one issue per unique warning, closes issues when the warning
# is gone, and skips creation when an issue already exists.
#
# Deduplication key: SHA-256(relative-path + flag + first-80-chars-of-message).
# Line numbers are excluded so that adding code above a warning does not
# create a duplicate issue.
#
# Usage:
#   GH_TOKEN=... python3 scripts/file_build_warnings.py build.log
#   GH_TOKEN=... python3 scripts/file_build_warnings.py build.log --dry-run

import hashlib
import json
import os
import re
import subprocess
import sys

LABEL = "build-warning"
FINGERPRINT_TAG = "<!-- warning-id:"

# Matches GCC / Clang one-line warning format:
#   /path/to/file.cpp:42:10: warning: some message [-Wsome-flag]
WARNING_RE = re.compile(r"^(.+?):(\d+):\d+:\s+warning:\s+(.+)$")

# Matches the [-Wflag] suffix at the end of a warning message
FLAG_SUFFIX_RE = re.compile(r"\s+(\[-W[^\]]+\])\s*$")


def make_relative(path: str) -> str:
    """Strip nix store / build-sandbox prefix, keeping the repo-relative path."""
    for anchor in ("/libs/", "/apps/", "/tests/", "/bindings/", "/python/", "/cmake/"):
        idx = path.find(anchor)
        if idx != -1:
            return path[idx + 1:]
    for segment in ("/source/", "/build/"):
        idx = path.find(segment)
        if idx != -1:
            return path[idx + len(segment):]
    return path


def warning_fingerprint(rel_path: str, flag: str, message: str) -> str:
    key = f"{rel_path}|{flag}|{message[:80]}"
    return hashlib.sha256(key.encode()).hexdigest()[:16]


# Only track warnings from code we own.  Vendored / generated / system paths
# are excluded so dependency headers and bundled third-party sources don't
# create noise in the issue tracker.
_OUR_PREFIXES = (
    "libs/reusex/src/",
    "libs/reusex/include/",
    "apps/",
    "tests/",
    "bindings/",
)


def is_our_source(rel_path: str) -> bool:
    return any(rel_path.startswith(p) for p in _OUR_PREFIXES)


def parse_warnings(log_path: str) -> list[dict]:
    seen: set[str] = set()
    warnings: list[dict] = []
    with open(log_path) as f:
        for line in f:
            line = line.rstrip()
            m = WARNING_RE.match(line)
            if not m:
                continue
            filepath, lineno, full_message = m.groups()
            rel = make_relative(filepath)
            if not is_our_source(rel):
                continue
            # Separate the [-Wflag] suffix from the message body
            fm = FLAG_SUFFIX_RE.search(full_message)
            if fm:
                flag = fm.group(1)
                message = full_message[: fm.start()].strip()
            else:
                flag = ""
                message = full_message.strip()
            fingerprint = warning_fingerprint(rel, flag, message)
            if fingerprint in seen:
                continue
            seen.add(fingerprint)
            title = f"[build-warning] {rel}:{lineno}: {message}"
            if flag:
                title += f" {flag}"
            warnings.append(
                {
                    "file": rel,
                    "line": int(lineno),
                    "message": message,
                    "flag": flag,
                    "fingerprint": fingerprint,
                    "title": title[:200],  # gh title limit
                    "raw": line,
                }
            )
    return warnings


def fetch_existing_issues(repo: str) -> dict[str, int]:
    """Return {fingerprint: issue_number} for all open build-warning issues."""
    result = subprocess.run(
        [
            "gh", "issue", "list",
            "--repo", repo,
            "--label", LABEL,
            "--state", "open",
            "--json", "number,body",
            "--limit", "500",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    issues: dict[str, int] = {}
    for issue in json.loads(result.stdout):
        body = issue.get("body") or ""
        for line in body.splitlines():
            if line.startswith(FINGERPRINT_TAG):
                fp = line.removeprefix(FINGERPRINT_TAG).rstrip(" -->").strip()
                issues[fp] = issue["number"]
                break
    return issues


def ensure_label(repo: str) -> None:
    subprocess.run(
        [
            "gh", "label", "create", LABEL,
            "--color", "e4e669",
            "--description", "Compiler warning tracked by the warnings workflow",
            "--repo", repo,
            "--force",
        ],
        capture_output=True,
    )


def create_issue(repo: str, w: dict, dry_run: bool) -> str:
    body = (
        f"Compiler warning detected during the CI build.\n\n"
        f"```\n{w['raw']}\n```\n\n"
        f"**File:** `{w['file']}`  \n"
        f"**Line:** {w['line']}  \n"
        f"**Flag:** `{w['flag'] or 'n/a'}`  \n\n"
        f"Fix the warning and the next run of the *Build Warnings* workflow "
        f"will close this issue automatically.\n\n"
        f"{FINGERPRINT_TAG} {w['fingerprint']} -->"
    )
    if dry_run:
        return "(dry-run)"
    result = subprocess.run(
        [
            "gh", "issue", "create",
            "--repo", repo,
            "--title", w["title"],
            "--label", LABEL,
            "--body", body,
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def close_issue(repo: str, number: int, dry_run: bool) -> None:
    if dry_run:
        return
    subprocess.run(
        [
            "gh", "issue", "close", str(number),
            "--repo", repo,
            "--comment",
            "Warning no longer detected in the build. Closing automatically.",
        ],
        check=True,
    )


def current_repo() -> str:
    result = subprocess.run(
        ["gh", "repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner"],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def main() -> None:
    args = sys.argv[1:]
    dry_run = "--dry-run" in args
    log_files = [a for a in args if not a.startswith("-")]

    if not log_files:
        print(
            "Usage: file_build_warnings.py <build.log> [--dry-run]",
            file=sys.stderr,
        )
        sys.exit(1)

    repo = current_repo()
    print(f"Repo: {repo}  dry-run={dry_run}")

    warnings = parse_warnings(log_files[0])
    print(f"Parsed {len(warnings)} unique warnings from {log_files[0]}")

    if not dry_run:
        ensure_label(repo)

    existing = fetch_existing_issues(repo)
    print(f"Open build-warning issues: {len(existing)}")

    current_fps = {w["fingerprint"] for w in warnings}

    created = 0
    for w in warnings:
        if w["fingerprint"] not in existing:
            url = create_issue(repo, w, dry_run)
            print(f"  + create: {w['title'][:80]}  →  {url}")
            created += 1
        else:
            print(f"  = exists: #{existing[w['fingerprint']]}  {w['title'][:70]}")

    closed = 0
    for fp, number in existing.items():
        if fp not in current_fps:
            close_issue(repo, number, dry_run)
            print(f"  x close:  #{number} (warning resolved)")
            closed += 1

    print(
        f"\nDone: {created} created, {closed} closed, "
        f"{len(warnings) - created} already tracked."
    )


if __name__ == "__main__":
    main()
