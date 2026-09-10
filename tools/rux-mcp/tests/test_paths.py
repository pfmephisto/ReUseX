# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The query_db allowlist: what gets through, and what must not."""

from __future__ import annotations

import pytest

from rux_mcp.paths import (
    COLLECTIONS,
    PathNotAllowed,
    check_query_path,
    describe_allowlist,
)


@pytest.mark.parametrize("path", ["", "   ", None])
def test_empty_path_lists_collections(path):
    assert check_query_path(path) == []


@pytest.mark.parametrize("collection", COLLECTIONS)
def test_every_collection_can_be_listed(collection):
    assert check_query_path(collection) == [collection]


@pytest.mark.parametrize(
    "path,expected",
    [
        ("clouds.cloud.metadata", ["clouds", "cloud", "metadata"]),
        ("clouds/cloud/point_count", ["clouds", "cloud", "point_count"]),
        ("frames.100", ["frames", "100"]),
        ("frames.100.pose", ["frames", "100", "pose"]),
        ("frames.100.intrinsics", ["frames", "100", "intrinsics"]),
        ("meshes.mesh", ["meshes", "mesh"]),
        ("meshes.mesh.vertex_count", ["meshes", "mesh", "vertex_count"]),
        ("labels.7.metadata", ["labels", "7", "metadata"]),
        ("log.10.status", ["log", "10", "status"]),
        ("projects", ["projects"]),
        (
            "materials.bc651919-0c95-457c-9e0b-7148509632d1.properties",
            ["materials", "bc651919-0c95-457c-9e0b-7148509632d1", "properties"],
        ),
    ],
)
def test_json_paths_are_allowed(path, expected):
    assert check_query_path(path) == expected


@pytest.mark.parametrize(
    "path",
    [
        "clouds.cloud",  # raw binary PCD
        "meshes.mesh.data",  # binary PLY/OBJ
        "meshes.mesh.texture",  # JPEG
        "meshes.mesh.material",  # MTL
        "frames.100.color",  # JPEG
        "frames.100.depth",  # PNG
        "frames.100.confidence",  # PNG
        "frames.100.image",  # JPEG alias
        "labels.7",  # PNG raster
        "panoramas.pano1",  # may be an image
        "panoramas.pano1.image",
    ],
)
def test_binary_paths_are_refused(path):
    with pytest.raises(PathNotAllowed):
        check_query_path(path)


def test_binary_refusal_names_the_alternative():
    with pytest.raises(PathNotAllowed) as excinfo:
        check_query_path("clouds.cloud")
    message = str(excinfo.value)
    assert "metadata" in message and "render_view" in message


@pytest.mark.parametrize(
    "path",
    [
        "components",  # advertised by `rux get --help`, not a real collection
        "passports",  # ditto
        "nonsense",
        "sqlite_master",
    ],
)
def test_unknown_collections_are_refused(path):
    with pytest.raises(PathNotAllowed) as excinfo:
        check_query_path(path)
    assert "Available" in str(excinfo.value)


@pytest.mark.parametrize(
    "path",
    [
        "--output",  # would be read as a CLI flag
        "clouds.--pretty",
        "clouds.../../etc/passwd",
        "clouds.a b",
        "clouds.a;rm -rf /",
        "clouds.$(id)",
        "clouds.a\nb",
    ],
)
def test_hostile_components_are_refused(path):
    with pytest.raises(PathNotAllowed):
        check_query_path(path)


def test_unknown_property_is_refused_with_the_allowed_set():
    with pytest.raises(PathNotAllowed) as excinfo:
        check_query_path("clouds.cloud.points")
    assert "metadata" in str(excinfo.value)


def test_depth_is_bounded():
    with pytest.raises(PathNotAllowed) as excinfo:
        check_query_path("clouds.cloud.metadata.x.y.z")
    assert "too deep" in str(excinfo.value)


def test_materials_allow_one_extra_level_for_property_names():
    assert check_query_path("materials.guid.properties.contact_email") == [
        "materials",
        "guid",
        "properties",
        "contact_email",
    ]


def test_wildcards_survive_in_item_position():
    assert check_query_path("meshes.*") == ["meshes", "*"]


def test_describe_allowlist_mentions_every_collection():
    text = describe_allowlist()
    for collection in COLLECTIONS:
        assert collection in text
    assert "list_components" in text
