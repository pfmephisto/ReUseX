# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run with:
#   blender --background --python apps/blender/test_addon.py

import sys
import traceback

import bpy
import numpy as np

ADDON_DIR = "/home/mephisto/repos/ReUseX/apps/blender"
if ADDON_DIR not in sys.path:
    sys.path.insert(0, ADDON_DIR)


def section(title):
    print(f"\n=== {title} ===")


def ok(msg):
    print(f"  OK  {msg}")


def fail(msg):
    print(f"  FAIL {msg}")
    raise AssertionError(msg)


# ── Registration ──────────────────────────────────────────────────────────────
section("Registration")

import reusex_panel  # noqa: E402

reusex_panel.register()
ok("register()")

for op_id in (
    "reusex.open_project",
    "reusex.close_project",
    "reusex.load_point_cloud",
    "reusex.select_cloud_object",
):
    cat, name = op_id.split(".")
    if hasattr(getattr(bpy.ops, cat), name):
        ok(f"operator {op_id}")
    else:
        fail(f"operator {op_id} missing")

if hasattr(bpy.context.scene, "reusex"):
    ok("scene.reusex property")
else:
    fail("scene.reusex property missing")

# ── PointCloud creation ───────────────────────────────────────────────────────
section("PointCloud creation")

n = 500
rng = np.random.default_rng(0)
positions = rng.random((n, 3), dtype=np.float32)
colors_u8 = (rng.random((n, 3)) * 255).astype(np.uint8)

pc_data = bpy.data.pointclouds.new("rux_test")
pc_data.resize(n)
ok("resize()")

pc_data.attributes["position"].data.foreach_set("vector", positions.flatten())
ok("position attribute set")

col_attr = pc_data.attributes.new("color", "FLOAT_COLOR", "POINT")
rgba = np.ones((n, 4), dtype=np.float32)
rgba[:, :3] = colors_u8.astype(np.float32) / 255.0
col_attr.data.foreach_set("color", rgba.flatten())
ok("color attribute set")

assert len(pc_data.points) == n, f"point count {len(pc_data.points)} != {n}"
ok(f"point count == {n}")

# ── Custom properties (label defs) ───────────────────────────────────────────
section("Custom properties")

obj = bpy.data.objects.new("rux_test", pc_data)
bpy.context.collection.objects.link(obj)

label_defs = {0: "unlabeled", 1: "floor", 2: "wall", 3: "ceiling"}
for lid, lname in label_defs.items():
    obj[f"rux_label_{lid}"] = lname
obj["rux_cloud_name"] = "test"
obj["rux_cloud_type"] = "PointXYZRGB"

assert obj["rux_label_1"] == "floor"
assert obj["rux_cloud_type"] == "PointXYZRGB"
ok("label and metadata custom properties")

# ── Hot-reload cycle ──────────────────────────────────────────────────────────
section("Hot-reload")

reusex_panel.unregister()
reusex_panel.register()
ok("unregister + register cycle")

# ── Done ──────────────────────────────────────────────────────────────────────
print("\nALL TESTS PASSED")
