# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
from bpy.props import StringProperty
from bpy_extras.io_utils import ImportHelper


class REUSEX_OT_open_project(bpy.types.Operator, ImportHelper):
    bl_idname = "reusex.open_project"
    bl_label = "Open ReUseX Project"
    bl_description = "Open a .rux project database and load its summary"

    filter_glob: StringProperty(default="*.rux", options={"HIDDEN"})

    def execute(self, context):
        try:
            import reusex
        except ImportError:
            self.report({"ERROR"}, "reusex Python module not available. "
                        "Build with -DBUILD_PYTHON_BINDINGS=ON first.")
            return {"CANCELLED"}

        if reusex.__status__ != "Active":
            self.report({"ERROR"}, f"reusex native module not loaded: {reusex.__status__}")
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

        # Clouds
        props.clouds.clear()
        for c in summary.clouds:
            item = props.clouds.add()
            item.name = c.name
            item.cloud_type = c.type
            item.point_count = c.point_count
            item.organized = c.organized

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


_classes = (
    REUSEX_OT_open_project,
    REUSEX_OT_close_project,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
