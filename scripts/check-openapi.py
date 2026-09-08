#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Parse-check the GUI API contract documents.

The OpenAPI spec in docs/gui/ is a *deliverable*: the frontend, and eventually
ruxd, are written against it, and generators read it directly. PR #274 shipped
it with an unquoted scalar containing ": ", which made the whole file
unreadable by every YAML tool while looking perfectly fine in review. This
script is the guard against that class of mistake.

It checks that:
  * docs/gui/openapi.yaml parses as YAML,
  * it has the handful of top-level keys an OpenAPI document must have,
  * every path/operation carries a summary and at least one response,
  * docs/gui/events.schema.json parses as JSON,
  * every $ref in the schema resolves to a $defs entry that exists.

It deliberately does not attempt full OpenAPI validation -- that needs a
dependency we do not carry. Catching "does not parse" and "is missing its
spine" is what actually regressed.

Usage:  scripts/check-openapi.py [files...]      (defaults to docs/gui/*)
Exit code 0 when everything is well-formed, 1 otherwise.
"""

from __future__ import annotations

import json
import pathlib
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - environment problem, not a spec problem
    print("check-openapi: PyYAML is not installed; skipping", file=sys.stderr)
    sys.exit(0)

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_SPEC = REPO_ROOT / "docs" / "gui" / "openapi.yaml"
DEFAULT_EVENTS = REPO_ROOT / "docs" / "gui" / "events.schema.json"


def fail(message: str) -> None:
    print(f"check-openapi: ERROR: {message}", file=sys.stderr)


def check_openapi(path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    try:
        spec = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        return [f"{path}: does not parse as YAML: {exc}"]

    if not isinstance(spec, dict):
        return [f"{path}: top level is not a mapping"]

    for key in ("openapi", "info", "paths"):
        if key not in spec:
            errors.append(f"{path}: missing top-level '{key}'")

    paths = spec.get("paths") or {}
    if not paths:
        errors.append(f"{path}: declares no paths")

    methods = {"get", "put", "post", "delete", "patch", "options", "head"}
    for route, operations in paths.items():
        if not isinstance(operations, dict):
            errors.append(f"{path}: path '{route}' is not a mapping")
            continue
        for method, operation in operations.items():
            if method not in methods:
                continue
            if not isinstance(operation, dict):
                errors.append(f"{path}: {method.upper()} {route} is not a mapping")
                continue
            if not operation.get("summary"):
                errors.append(f"{path}: {method.upper()} {route} has no summary")
            if not operation.get("responses"):
                errors.append(f"{path}: {method.upper()} {route} has no responses")

    # Every local $ref must resolve; a typo here silently produces an empty
    # schema in most generators rather than an error.
    schemas = ((spec.get("components") or {}).get("schemas") or {})
    for ref in collect_refs(spec):
        if not ref.startswith("#/components/schemas/"):
            continue
        name = ref.rsplit("/", 1)[-1]
        if name not in schemas:
            errors.append(f"{path}: $ref '{ref}' does not resolve")

    return errors


def check_events_schema(path: pathlib.Path) -> list[str]:
    try:
        schema = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        return [f"{path}: does not parse as JSON: {exc}"]

    errors: list[str] = []
    defs = schema.get("$defs") or {}
    for ref in collect_refs(schema):
        if not ref.startswith("#/$defs/"):
            continue
        name = ref.rsplit("/", 1)[-1]
        if name not in defs:
            errors.append(f"{path}: $ref '{ref}' does not resolve")
    return errors


def collect_refs(node: object) -> list[str]:
    found: list[str] = []
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "$ref" and isinstance(value, str):
                found.append(value)
            else:
                found.extend(collect_refs(value))
    elif isinstance(node, list):
        for item in node:
            found.extend(collect_refs(item))
    return found


def main(argv: list[str]) -> int:
    targets = [pathlib.Path(a) for a in argv[1:]] or [DEFAULT_SPEC, DEFAULT_EVENTS]

    errors: list[str] = []
    for target in targets:
        if not target.exists():
            errors.append(f"{target}: does not exist")
        elif target.suffix in (".yaml", ".yml"):
            errors.extend(check_openapi(target))
        elif target.suffix == ".json":
            errors.extend(check_events_schema(target))

    for error in errors:
        fail(error)
    if errors:
        return 1

    print(f"check-openapi: OK ({len(targets)} file(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
