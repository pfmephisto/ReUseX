# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy


class REUSEX_PT_main_panel(bpy.types.Panel):
    bl_label = "ReUseX"
    bl_idname = "REUSEX_PT_main_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        if not props.is_loaded:
            layout.operator("reusex.open_project", icon="FILE_FOLDER")
            return

        layout.label(text=props.file_path, icon="FILE_3D")
        row = layout.row(align=True)
        row.operator("reusex.close_project", icon="X")
        row.operator("reusex.open_project", text="Reload", icon="FILE_REFRESH")
        layout.label(text=f"Schema version: {props.schema_version}")


class REUSEX_PT_projects_panel(bpy.types.Panel):
    bl_label = "Projects"
    bl_idname = "REUSEX_PT_projects_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"
    bl_parent_id = "REUSEX_PT_main_panel"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.scene.reusex.is_loaded

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        if not props.projects:
            layout.label(text="No project metadata", icon="INFO")
            return

        for p in props.projects:
            box = layout.box()
            if p.project_name:
                box.label(text=p.project_name, icon="HOME")
            if p.building_address:
                box.label(text=p.building_address)
            if p.year_of_construction > 0:
                box.label(text=f"Built: {p.year_of_construction}")
            if p.survey_date:
                box.label(text=f"Surveyed: {p.survey_date}")
            if p.survey_organisation:
                box.label(text=f"By: {p.survey_organisation}")
            if p.notes:
                box.label(text=p.notes)


class REUSEX_PT_sensor_frames_panel(bpy.types.Panel):
    bl_label = "Sensor Frames"
    bl_idname = "REUSEX_PT_sensor_frames_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"
    bl_parent_id = "REUSEX_PT_main_panel"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.scene.reusex.is_loaded

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        col = layout.column(align=True)
        col.label(text=f"Frames: {props.sensor_frame_count}")
        if props.sensor_frame_width > 0:
            col.label(text=f"Resolution: {props.sensor_frame_width} x {props.sensor_frame_height}")
        col.label(text=f"Segmented: {props.segmented_frame_count}")

        if props.panoramic_count > 0:
            col.separator()
            col.label(text=f"Panoramics: {props.panoramic_count}")
            col.label(text=f"Matched: {props.panoramic_matched}")

        layout.separator()
        if props.pose_graph_loaded:
            layout.label(text=f"Pose graph: {len(props.frames)} nodes", icon="EMPTY_ARROWS")
        layout.operator(
            "reusex.load_pose_graph",
            text="Reload Pose Graph" if props.pose_graph_loaded else "Load Pose Graph",
            icon="EMPTY_ARROWS",
        )

        # Per-node cloud loader — shown when a frame empty is selected
        obj = context.object
        if (
            obj is not None
            and obj.name.startswith("RUX_Frame_")
            and "rux_frame_id" in obj
        ):
            fid = obj["rux_frame_id"]
            cloud_loaded = bpy.data.objects.get(f"RUX_Cloud_{fid}") is not None
            layout.separator()
            layout.operator(
                "reusex.load_frame_cloud",
                text=f"Reload Cloud  (frame {fid})" if cloud_loaded else f"Load Cloud  (frame {fid})",
                icon="OUTLINER_OB_POINTCLOUD",
            )


class REUSEX_PT_clouds_panel(bpy.types.Panel):
    bl_label = "Point Clouds"
    bl_idname = "REUSEX_PT_clouds_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"
    bl_parent_id = "REUSEX_PT_main_panel"

    @classmethod
    def poll(cls, context):
        return context.scene.reusex.is_loaded

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        if not props.clouds:
            layout.label(text="No point clouds", icon="INFO")
            return

        for c in props.clouds:
            box = layout.box()

            # Header: name · type · point count
            row = box.row(align=True)
            row.label(text=c.name, icon="OUTLINER_OB_POINTCLOUD")
            row.label(text=c.cloud_type)
            row.label(text=f"{c.point_count:,} pts")

            # Load / reload button and loaded-object indicator
            action_row = box.row(align=True)
            op = action_row.operator(
                "reusex.load_point_cloud",
                text="Reload" if c.loaded_object_name else "Load",
                icon="IMPORT",
            )
            op.cloud_name = c.name

            if c.loaded_object_name:
                obj = bpy.data.objects.get(c.loaded_object_name)
                if obj is not None:
                    action_row.label(text="Loaded", icon="CHECKMARK")
                    sel = action_row.operator(
                        "reusex.select_cloud_object",
                        text="",
                        icon="RESTRICT_SELECT_OFF",
                    )
                    sel.object_name = c.loaded_object_name
                else:
                    # Object was deleted from the scene — queue stale-ref clear
                    action_row.label(text="Removed", icon="ERROR")
                    op = action_row.operator("reusex.clear_stale_cloud", text="", icon="X")
                    op.cloud_name = c.name

            # Label definitions (shown when available)
            if c.labels:
                lbl_box = box.box()
                lbl_box.label(text=f"Labels ({len(c.labels)}):", icon="BOOKMARKS")
                col = lbl_box.column(align=True)
                for lbl in c.labels:
                    col.label(text=f"[{lbl.label_id}]  {lbl.label_name}")


class REUSEX_PT_meshes_panel(bpy.types.Panel):
    bl_label = "Meshes"
    bl_idname = "REUSEX_PT_meshes_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"
    bl_parent_id = "REUSEX_PT_main_panel"

    @classmethod
    def poll(cls, context):
        return context.scene.reusex.is_loaded

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        if not props.meshes:
            layout.label(text="No meshes", icon="INFO")
            return

        for m in props.meshes:
            row = layout.row()
            row.label(text=m.name, icon="MESH_DATA")
            row.label(text=f"V:{m.vertex_count:,}")
            row.label(text=f"F:{m.face_count:,}")


class REUSEX_PT_materials_panel(bpy.types.Panel):
    bl_label = "Materials"
    bl_idname = "REUSEX_PT_materials_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ReUseX"
    bl_parent_id = "REUSEX_PT_main_panel"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return context.scene.reusex.is_loaded

    def draw(self, context):
        layout = self.layout
        props = context.scene.reusex

        if not props.materials:
            layout.label(text="No material passports", icon="INFO")
            return

        for mat in props.materials:
            box = layout.box()
            box.label(text=mat.name, icon="MATERIAL")
            row = box.row()
            row.label(text=f"v{mat.version_number}" if mat.version_number else "No version")
            row.label(text=f"{mat.property_count} props")


_classes = (
    REUSEX_PT_main_panel,
    REUSEX_PT_projects_panel,
    REUSEX_PT_sensor_frames_panel,
    REUSEX_PT_clouds_panel,
    REUSEX_PT_meshes_panel,
    REUSEX_PT_materials_panel,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
