# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
from bpy.props import StringProperty
from bpy_extras.io_utils import ImportHelper


# Colormap for label IDs — index 0 is unlabeled (gray), rest cycle for valid labels
_LABEL_PALETTE = [
    (128, 128, 128),  # unlabeled
    (255,  80,  80),  # red
    ( 80, 200,  80),  # green
    ( 80, 120, 255),  # blue
    (255, 200,  80),  # yellow
    (200,  80, 255),  # purple
    ( 80, 220, 220),  # cyan
    (255, 160,  80),  # orange
    (160, 255, 120),  # light green
    (255, 120, 200),  # pink
    (180, 140,  80),  # brown
    ( 80, 180, 160),  # teal
    (240, 120, 120),  # salmon
    (120, 120, 240),  # lavender
]


def _label_colors_numpy(label_ids, np):
    """Return (N, 3) uint8 array mapping label IDs to palette colors."""
    palette = np.array(_LABEL_PALETTE, dtype=np.uint8)
    n_named = len(palette) - 1  # palette[0] reserved for unlabeled
    ids = np.asarray(label_ids, dtype=np.int32)
    indices = np.where(ids < 0, 0, (ids % n_named) + 1)
    return palette[indices]


def _populate_cloud_labels(cloud_item, label_dict):
    """Fill a REUSEX_CloudItem.labels collection from a {int: str} dict."""
    cloud_item.labels.clear()
    for label_id, label_name in sorted(label_dict.items()):
        lbl = cloud_item.labels.add()
        lbl.label_id = label_id
        lbl.label_name = label_name


def _check_reusex():
    """Import and validate the reusex module. Returns (module, error_string)."""
    try:
        import reusex
    except ImportError:
        return None, "reusex Python module not available. Build with -DBUILD_PYTHON_BINDINGS=ON first."
    if reusex.__status__ != "Active":
        return None, f"reusex native module not loaded: {reusex.__status__}"
    return reusex, None


class REUSEX_OT_open_project(bpy.types.Operator, ImportHelper):
    bl_idname = "reusex.open_project"
    bl_label = "Open ReUseX Project"
    bl_description = "Open a .rux project database and load its summary"

    filter_glob: StringProperty(default="*.rux", options={"HIDDEN"})

    def execute(self, context):
        reusex, err = _check_reusex()
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}

        filepath = self.filepath
        try:
            db = reusex.ProjectDB(filepath, read_only=True)
            summary = db.project_summary()
        except Exception as e:
            self.report({"ERROR"}, f"Failed to open project: {e}")
            return {"CANCELLED"}

        props = context.scene.reusex
        props.file_path = filepath
        props.is_loaded = True
        props.schema_version = summary.schema_version

        # Sensor frames
        sf = summary.sensor_frames
        props.sensor_frame_count = sf.total_count
        props.sensor_frame_width = sf.width
        props.sensor_frame_height = sf.height
        props.segmented_frame_count = sf.segmented_count

        # Panoramics
        pi = summary.panoramic_images
        props.panoramic_count = pi.total_count
        props.panoramic_matched = pi.matched_count

        # Components
        props.component_count = summary.components.total_count

        # Clouds — populate metadata and any label definitions from summary
        props.clouds.clear()
        for c in summary.clouds:
            item = props.clouds.add()
            item.name = c.name
            item.cloud_type = c.type
            item.point_count = c.point_count
            item.organized = c.organized
            if hasattr(c, "labels") and c.labels:
                _populate_cloud_labels(item, c.labels)

        # Meshes
        props.meshes.clear()
        for m in summary.meshes:
            item = props.meshes.add()
            item.name = m.name
            item.vertex_count = m.vertex_count
            item.face_count = m.face_count

        # Materials
        props.materials.clear()
        for mat in summary.materials:
            item = props.materials.add()
            item.name = mat.id
            item.guid = mat.guid
            item.property_count = mat.property_count
            item.version_number = mat.version_number

        # Projects
        props.projects.clear()
        for p in summary.projects:
            item = props.projects.add()
            item.name = p.id
            item.project_name = p.name
            item.building_address = p.building_address
            item.year_of_construction = p.year_of_construction
            item.survey_date = p.survey_date
            item.survey_organisation = p.survey_organisation
            item.notes = p.notes

        self.report({"INFO"}, f"Opened project: {filepath}")
        return {"FINISHED"}


class REUSEX_OT_close_project(bpy.types.Operator):
    bl_idname = "reusex.close_project"
    bl_label = "Close Project"
    bl_description = "Close the current ReUseX project"

    def execute(self, context):
        props = context.scene.reusex
        props.file_path = ""
        props.is_loaded = False
        props.schema_version = 0
        props.sensor_frame_count = 0
        props.sensor_frame_width = 0
        props.sensor_frame_height = 0
        props.segmented_frame_count = 0
        props.panoramic_count = 0
        props.panoramic_matched = 0
        props.component_count = 0
        props.clouds.clear()
        props.meshes.clear()
        props.materials.clear()
        props.projects.clear()

        self.report({"INFO"}, "Project closed")
        return {"FINISHED"}


class REUSEX_OT_load_point_cloud(bpy.types.Operator):
    bl_idname = "reusex.load_point_cloud"
    bl_label = "Load Point Cloud"
    bl_description = "Load point cloud geometry into the 3D viewport with label attributes"

    cloud_name: StringProperty()

    def execute(self, context):
        try:
            import numpy as np
        except ImportError:
            self.report({"ERROR"}, "NumPy not available")
            return {"CANCELLED"}

        reusex, err = _check_reusex()
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}

        props = context.scene.reusex
        if not props.is_loaded:
            self.report({"ERROR"}, "No project loaded")
            return {"CANCELLED"}

        cloud_item = next((c for c in props.clouds if c.name == self.cloud_name), None)
        if cloud_item is None:
            self.report({"ERROR"}, f"Cloud '{self.cloud_name}' not found")
            return {"CANCELLED"}

        try:
            db = reusex.ProjectDB(props.file_path, read_only=True)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to open project: {e}")
            return {"CANCELLED"}

        # Load geometry.
        # Bindings return dicts of numpy arrays:
        #   point_cloud_xyzrgb → {"positions": (N,3) float32, "colors": (N,3) uint8}
        #   point_cloud_xyz    → {"positions": (N,3) float32}
        #   point_cloud_label  → {"labels": (N,) int32}
        # Label and Normal clouds carry no position data — we skip geometry for them.
        try:
            cloud_type = cloud_item.cloud_type
            positions = colors = label_ids = None

            if cloud_type == "PointXYZRGB":
                data = db.point_cloud_xyzrgb(self.cloud_name)
                positions = np.asarray(data["positions"], dtype=np.float32)
                colors = np.asarray(data["colors"], dtype=np.uint8)
            elif cloud_type == "PointXYZ":
                data = db.point_cloud_xyz(self.cloud_name)
                positions = np.asarray(data["positions"], dtype=np.float32)
            elif cloud_type in ("Label", "Normal"):
                self.report(
                    {"WARNING"},
                    f"'{self.cloud_name}' is a {cloud_type} cloud and has no "
                    "position data — load the paired PointXYZRGB cloud instead.",
                )
                return {"CANCELLED"}
            else:
                self.report({"ERROR"}, f"Unknown cloud type: {cloud_type}")
                return {"CANCELLED"}

            # Fetch label definitions (may not exist for all cloud types)
            label_defs = {}
            try:
                label_defs = dict(db.label_definitions(self.cloud_name))
            except Exception:
                pass

        except Exception as e:
            self.report({"ERROR"}, f"Failed to load '{self.cloud_name}': {e}")
            return {"CANCELLED"}

        n = len(positions)
        if n == 0:
            self.report({"WARNING"}, f"Cloud '{self.cloud_name}' is empty")
            return {"CANCELLED"}

        # Remove stale Blender object from a previous load
        if cloud_item.loaded_object_name:
            existing = bpy.data.objects.get(cloud_item.loaded_object_name)
            if existing is not None:
                bpy.data.objects.remove(existing, do_unlink=True)

        # Build Blender PointCloud data block.
        # resize(n) allocates n points and auto-creates the "position" attribute.
        pc_data = bpy.data.pointclouds.new(self.cloud_name)
        pc_data.resize(n)

        pc_data.attributes["position"].data.foreach_set("vector", positions.flatten())

        if colors is not None:
            col_attr = pc_data.attributes.new("color", "FLOAT_COLOR", "POINT")
            rgba = np.ones((n, 4), dtype=np.float32)
            rgba[:, :3] = colors.astype(np.float32) / 255.0
            col_attr.data.foreach_set("color", rgba.flatten())

        # Create and link the object
        obj = bpy.data.objects.new(self.cloud_name, pc_data)
        context.collection.objects.link(obj)

        # Attach cloud metadata and label definitions as custom properties
        obj["rux_cloud_name"] = self.cloud_name
        obj["rux_cloud_type"] = cloud_type
        for lid, lname in label_defs.items():
            obj[f"rux_label_{lid}"] = lname

        # Select the new object
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        context.view_layer.objects.active = obj

        # Sync label definitions back to the scene property
        _populate_cloud_labels(cloud_item, label_defs)
        cloud_item.loaded_object_name = obj.name

        self.report({"INFO"}, f"Loaded '{self.cloud_name}': {n:,} points")
        return {"FINISHED"}


class REUSEX_OT_select_cloud_object(bpy.types.Operator):
    bl_idname = "reusex.select_cloud_object"
    bl_label = "Select Cloud Object"
    bl_description = "Select the loaded point cloud object in the viewport"

    object_name: StringProperty()

    def execute(self, context):
        obj = bpy.data.objects.get(self.object_name)
        if obj is None:
            self.report({"WARNING"}, f"Object '{self.object_name}' not found in scene")
            return {"CANCELLED"}

        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({"INFO"}, f"Selected '{self.object_name}'")
        return {"FINISHED"}


_classes = (
    REUSEX_OT_open_project,
    REUSEX_OT_close_project,
    REUSEX_OT_load_point_cloud,
    REUSEX_OT_select_cloud_object,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
