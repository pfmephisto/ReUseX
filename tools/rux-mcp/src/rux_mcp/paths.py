# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Allowlist for the ``query_db`` passthrough over ``rux get``.

``rux get`` is path-addressable over the whole project database and happily
writes megabytes of binary PCD/PNG/PLY to stdout.  That is exactly what must
never reach an agent's context, so the gateway allows a path only when it is
known to yield JSON, and denies everything else with a message naming the
JSON-bearing alternative (progressive disclosure, per #267).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Iterable

#: Collections implemented by ``rux``'s router registry
#: (``apps/rux/src/database/resource_router.cpp``).  Note that the ``rux get
#: --help`` text still advertises ``components``/``passports``; neither is a
#: real collection — material passports live under ``materials``, and building
#: components have no ``rux get`` route at all (see ``list_components``).
COLLECTIONS: tuple[str, ...] = (
    "clouds",
    "frames",
    "labels",
    "log",
    "materials",
    "meshes",
    "panoramas",
    "projects",
)

#: A path component may only contain these characters.  Rejecting everything
#: else keeps a component from being read as a CLI flag or a path traversal,
#: on top of the argv list (never a shell) used by :class:`~rux_mcp.runner.RuxRunner`.
_COMPONENT = re.compile(r"[A-Za-z0-9_*?][A-Za-z0-9_*?.\-]*")


class PathNotAllowed(ValueError):
    """The requested ``rux get`` path is outside the read-only JSON allowlist."""


@dataclass(frozen=True)
class CollectionRule:
    """What is reachable under one collection.

    ``properties`` of ``None`` means "any property, any depth" — used for the
    collections whose routers only ever emit JSON.
    """

    item_allowed: bool
    properties: frozenset[str] | None
    binary_properties: frozenset[str] = frozenset()
    item_note: str = ""
    max_depth: int = 3


RULES: dict[str, CollectionRule] = {
    "clouds": CollectionRule(
        item_allowed=False,
        properties=frozenset({"metadata", "type", "point_count"}),
        item_note=(
            "'clouds.<name>' streams the raw binary PCD. Use "
            "'clouds.<name>.metadata' for the summary, or render_view() to see "
            "the geometry."
        ),
    ),
    "frames": CollectionRule(
        item_allowed=True,
        properties=frozenset(
            {"metadata", "pose", "intrinsics", "has_pose", "timestamp"}
        ),
        binary_properties=frozenset({"color", "image", "depth", "confidence"}),
    ),
    "labels": CollectionRule(
        item_allowed=False,
        properties=frozenset({"metadata"}),
        binary_properties=frozenset({"image"}),
        item_note=(
            "'labels.<id>' streams a binary PNG raster. Use "
            "'labels.<id>.metadata', or render_view(layers='labels')."
        ),
    ),
    "log": CollectionRule(item_allowed=True, properties=None),
    "materials": CollectionRule(item_allowed=True, properties=None, max_depth=4),
    "meshes": CollectionRule(
        item_allowed=True,
        properties=frozenset({"metadata", "format", "vertex_count", "polygon_count"}),
        binary_properties=frozenset({"data", "texture", "material"}),
    ),
    "panoramas": CollectionRule(
        item_allowed=False,
        properties=frozenset({"metadata"}),
        binary_properties=frozenset({"image"}),
        item_note="'panoramas.<name>' may stream a binary image. Use "
        "'panoramas.<name>.metadata'.",
    ),
    "projects": CollectionRule(item_allowed=True, properties=None),
}


def _split(path: str) -> list[str]:
    """Split a ``rux get`` path on ``.`` or ``/``, dropping empty segments."""
    return [part for part in path.replace("/", ".").split(".") if part]


def describe_allowlist() -> str:
    """Human/agent-readable rendering of the allowlist, used as an MCP resource."""
    lines = [
        "query_db() exposes the JSON-only subset of `rux get`.",
        "An empty path lists the collections.",
        "",
    ]
    for name in COLLECTIONS:
        rule = RULES[name]
        lines.append(f"{name}")
        lines.append(f"  {name}                       list items")
        if rule.item_allowed:
            lines.append(f"  {name}.<item>                item metadata (JSON)")
        if rule.properties is None:
            lines.append(f"  {name}.<item>.<property>     any property (JSON)")
        else:
            props = ", ".join(sorted(rule.properties))
            lines.append(f"  {name}.<item>.<property>     one of: {props}")
        denied: Iterable[str] = sorted(rule.binary_properties)
        if denied:
            lines.append("  denied (binary): " + ", ".join(denied))
        if rule.item_note:
            lines.append("  note: " + rule.item_note)
        lines.append("")
    lines.append(
        "Building components are not a `rux get` collection; use "
        "list_components()/get_component()."
    )
    return "\n".join(lines)


def check_query_path(path: str | None) -> list[str]:
    """Validate ``path`` and return its normalized components.

    An empty or ``None`` path is valid and means "list the collections".

    Raises:
        PathNotAllowed: the path is malformed, unknown, or would return binary.
    """
    if path is None or not path.strip():
        return []

    parts = _split(path.strip())
    if not parts:
        return []

    for part in parts:
        if not _COMPONENT.fullmatch(part):
            raise PathNotAllowed(
                f"invalid path component {part!r}: components may only contain "
                "letters, digits, '_', '-', '.' and the '*'/'?' wildcards, and "
                "may not start with '-'"
            )

    collection = parts[0]
    if collection not in RULES:
        raise PathNotAllowed(
            f"unknown collection {collection!r}. Available: " + ", ".join(COLLECTIONS)
        )
    rule = RULES[collection]

    if len(parts) > rule.max_depth:
        raise PathNotAllowed(
            f"path is too deep for '{collection}' (max {rule.max_depth} "
            f"components, got {len(parts)})"
        )

    if len(parts) == 1:
        return parts

    if len(parts) == 2 and not rule.item_allowed:
        raise PathNotAllowed(
            f"'{path}' is not exposed: {rule.item_note}"
            if rule.item_note
            else f"'{path}' is not exposed by the gateway"
        )

    if len(parts) >= 3:
        prop = parts[2]
        if prop in rule.binary_properties:
            raise PathNotAllowed(
                f"'{path}' returns binary data, which the gateway never puts "
                "into an agent's context. Use render_view() to look at "
                "geometry, or a metadata path for the numbers."
            )
        if rule.properties is not None and prop not in rule.properties:
            raise PathNotAllowed(
                f"property {prop!r} is not exposed for '{collection}'. "
                "Allowed: " + ", ".join(sorted(rule.properties))
            )

    return parts
