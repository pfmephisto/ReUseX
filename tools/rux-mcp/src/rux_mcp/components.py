# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Building-component inventory, read through ``rux export csv``.

Building components are the one part of the project that ``rux get`` cannot
address (see gap 3 in #267: ``building_component`` is C++-only today, and the
router registry has no ``components`` collection).  Until a ``rux get
components`` route exists, the inventory is recovered from ``rux export csv``,
which opens the database read-only and emits one row per component plus one
row per material passport.

This is a documented stopgap, isolated in this module so it can be swapped for
a real ``rux get`` call without touching the tool surface.
"""

from __future__ import annotations

import csv
import tempfile
from pathlib import Path
from typing import Any

from rux_mcp.runner import RuxRunner

#: Columns of the wide element CSV that belong to the passport template rather
#: than to a component.  Passport template headers are "Category / field".
_PASSPORT_PREFIXES = ("passport_",)


def _is_component_column(key: str) -> bool:
    return " / " not in key and not key.startswith(_PASSPORT_PREFIXES)


def _tidy(row: dict[str, Any]) -> dict[str, Any]:
    """Keep the component-side columns that actually carry a value."""
    return {
        key: value
        for key, value in row.items()
        if key and value not in (None, "") and _is_component_column(key)
    }


def read_elements(runner: RuxRunner, *, timeout: float | None = None) -> list[dict]:
    """Return every row of ``rux export csv`` (components *and* passports)."""
    with tempfile.TemporaryDirectory(prefix="rux-mcp-csv-") as tmp:
        target = Path(tmp) / "elements.csv"
        runner.run(["export", "csv", "-o", str(target)], timeout=timeout)
        if not target.exists():
            return []
        with target.open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))


def list_components(runner: RuxRunner, *, timeout: float | None = None) -> list[dict]:
    """Return the building-component inventory as compact dictionaries."""
    return [
        _tidy(row)
        for row in read_elements(runner, timeout=timeout)
        if row.get("kind") == "component"
    ]


def get_component(
    runner: RuxRunner, component_id: str, *, timeout: float | None = None
) -> dict:
    """Return one component by GUID or by ``component_name``.

    Raises:
        KeyError: no component matches ``component_id``.
    """
    wanted = component_id.strip()
    components = list_components(runner, timeout=timeout)
    for row in components:
        if row.get("id") == wanted or row.get("component_name") == wanted:
            return row
    known = ", ".join(
        sorted(str(row.get("component_name") or row.get("id")) for row in components)
    )
    raise KeyError(
        f"no building component with id or name {component_id!r}."
        + (f" Known components: {known}" if known else " The project has none.")
    )
