# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
from bpy.props import StringProperty
from bpy_extras.io_utils import ImportHelper

_POSE_GRAPH_COLLECTION = "RUX_PoseGraph"
_POSE_GRAPH_EDGES = "RUX_PoseGraph_Edges"


def _rebuild_pose_graph_edges():
    """Recompute edge mesh vertices from current Empty world positions."""
    coll = bpy.data.collections.get(_POSE_GRAPH_COLLECTION)
    edges_obj = bpy.data.objects.get(_POSE_GRAPH_EDGES)
    if coll is None or edges_obj is None:
        return

    frame_objs = sorted(
        (o for o in coll.objects if o.name.startswith("RUX_Frame_")),
        key=lambda o: o.get("rux_frame_id", 0),
    )
    if len(frame_objs) < 2:
        return

    verts = [tuple(o.matrix_world.to_translation()) for o in frame_objs]
    edges = [(i, i + 1) for i in range(len(verts) - 1)]

    mesh = edges_obj.data
    if len(mesh.vertices) == len(verts):
        for i, v in enumerate(verts):
            mesh.vertices[i].co = v
        mesh.update()
    else:
        mesh.clear_geometry()
        mesh.from_pydata(verts, edges, [])
        mesh.update()


@bpy.app.handlers.persistent
def _edge_sync_handler(scene, depsgraph):
    """Rebuild pose graph edges whenever any RUX_Frame empty is moved."""
    for update in depsgraph.updates:
        if (
            isinstance(update.id, bpy.types.Object)
            and update.id.name.startswith("RUX_Frame_")
            and update.is_updated_transform
        ):
            _rebuild_pose_graph_edges()
            return


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
    n_named = len(palette) - 1
    ids = np.asarray(label_ids, dtype=np.int32)
    indices = np.where(ids < 0, 0, (ids % n_named) + 1)
    return palette[indices]


def _make_frustum_mesh(fx, fy, cx, cy, width, height, near=0.15):
    """Wireframe camera frustum in optical frame (Z+ forward, Y+ down, X+ right)."""
    def proj(u, v):
        return ((u - cx) / fx * near, (v - cy) / fy * near, near)

    verts = [
        (0.0, 0.0, 0.0),
        proj(0,     0),
        proj(width, 0),
        proj(width, height),
        proj(0,     height),
    ]
    edges = [
        (0, 1), (0, 2), (0, 3), (0, 4),
        (1, 2), (2, 3), (3, 4), (4, 1),
    ]
    mesh = bpy.data.meshes.new("RUX_FrustumTemplate")
    mesh.from_pydata(verts, edges, [])
    mesh.update()
    return mesh


def _make_color_attr_material(name):
    """Return a new material wiring the 'color' point attribute to base color."""
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    attr = nodes.new("ShaderNodeAttribute")
    attr.attribute_name = "color"
    attr.location = (-280, 0)

    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (0, 0)
    bsdf.inputs["Roughness"].default_value = 1.0
    bsdf.inputs["Specular IOR Level"].default_value = 0.0

    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (300, 0)

    links.new(attr.outputs["Color"], bsdf.inputs["Base Color"])
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


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

        sf = summary.sensor_frames
        props.sensor_frame_count = sf.total_count
        props.sensor_frame_width = sf.width
        props.sensor_frame_height = sf.height
        props.segmented_frame_count = sf.segmented_count

        pi = summary.panoramic_images
        props.panoramic_count = pi.total_count
        props.panoramic_matched = pi.matched_count

        props.component_count = summary.components.total_count

        props.clouds.clear()
        for c in summary.clouds:
            item = props.clouds.add()
            item.name = c.name
            item.cloud_type = c.type
            item.point_count = c.point_count
            item.organized = c.organized
            if hasattr(c, "labels") and c.labels:
                _populate_cloud_labels(item, c.labels)

        props.meshes.clear()
        for m in summary.meshes:
            item = props.meshes.add()
            item.name = m.name
            item.vertex_count = m.vertex_count
            item.face_count = m.face_count

        props.materials.clear()
        for mat in summary.materials:
            item = props.materials.add()
            item.name = mat.id
            item.guid = mat.guid
            item.property_count = mat.property_count
            item.version_number = mat.version_number

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
        props.frames.clear()
        props.pose_graph_loaded = False

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

        try:
            cloud_type = cloud_item.cloud_type
            positions = colors = None

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

        if cloud_item.loaded_object_name:
            existing = bpy.data.objects.get(cloud_item.loaded_object_name)
            if existing is not None:
                bpy.data.objects.remove(existing, do_unlink=True)

        pc_data = bpy.data.pointclouds.new(self.cloud_name)
        pc_data.resize(n)
        pc_data.attributes["position"].data.foreach_set("vector", positions.flatten())

        if colors is not None:
            col_attr = pc_data.attributes.new("color", "FLOAT_COLOR", "POINT")
            rgba = np.ones((n, 4), dtype=np.float32)
            rgba[:, :3] = colors.astype(np.float32) / 255.0
            col_attr.data.foreach_set("color", rgba.flatten())

        obj = bpy.data.objects.new(self.cloud_name, pc_data)
        context.collection.objects.link(obj)

        if colors is not None:
            obj.data.materials.append(_make_color_attr_material(self.cloud_name))

        obj["rux_cloud_name"] = self.cloud_name
        obj["rux_cloud_type"] = cloud_type
        for lid, lname in label_defs.items():
            obj[f"rux_label_{lid}"] = lname

        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        context.view_layer.objects.active = obj

        _populate_cloud_labels(cloud_item, label_defs)
        cloud_item.loaded_object_name = obj.name

        self.report({"INFO"}, f"Loaded '{self.cloud_name}': {n:,} points")
        return {"FINISHED"}


class REUSEX_OT_load_pose_graph(bpy.types.Operator):
    bl_idname = "reusex.load_pose_graph"
    bl_label = "Load Pose Graph"
    bl_description = "Load sensor frame poses as arrow empties with odometry edges"

    def execute(self, context):
        reusex, err = _check_reusex()
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}

        props = context.scene.reusex
        if not props.is_loaded:
            self.report({"ERROR"}, "No project loaded")
            return {"CANCELLED"}

        try:
            db = reusex.ProjectDB(props.file_path, read_only=True)
            frame_ids = sorted(db.sensor_frame_ids())
        except Exception as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}

        if not frame_ids:
            self.report({"WARNING"}, "No sensor frames found in project")
            return {"CANCELLED"}

        self._setup_scene(context, db, frame_ids)
        props.pose_graph_loaded = True
        self.report({"INFO"}, f"Loaded pose graph: {len(frame_ids)} nodes")
        return {"FINISHED"}

    def _setup_scene(self, context, db, frame_ids):
        import numpy as np
        from mathutils import Matrix

        props = context.scene.reusex

        # Remove previous pose graph collection and all its objects
        old_coll = bpy.data.collections.get(_POSE_GRAPH_COLLECTION)
        if old_coll is not None:
            for obj in list(old_coll.objects):
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(old_coll)
        for name, store in (
            ("RUX_FrustumTemplate", bpy.data.meshes),
            ("RUX_FrameCloud",      bpy.data.materials),
        ):
            old = store.get(name)
            if old is not None:
                store.remove(old)

        coll = bpy.data.collections.new(_POSE_GRAPH_COLLECTION)
        context.scene.collection.children.link(coll)
        props.frames.clear()

        # Frustum template — built once from first frame's intrinsics
        frustum_mesh = None
        frustum_local_tf = None
        try:
            intr = db.sensor_frame_intrinsics(frame_ids[0])
            frustum_mesh = _make_frustum_mesh(
                intr["fx"], intr["fy"], intr["cx"], intr["cy"],
                intr["width"], intr["height"],
            )
            frustum_local_tf = Matrix(
                np.asarray(intr["local_transform"], dtype=np.float64).tolist()
            )
        except Exception:
            pass

        poses = []
        for fid in frame_ids:
            try:
                pose_np = np.asarray(db.sensor_frame_pose(fid), dtype=np.float64)
            except Exception:
                continue

            empty = bpy.data.objects.new(f"RUX_Frame_{fid}", None)
            empty.empty_display_type = "ARROWS"
            empty.empty_display_size = 0.05
            empty.matrix_world = Matrix(pose_np.tolist())
            empty["rux_frame_id"] = fid
            coll.objects.link(empty)

            item = props.frames.add()
            item.frame_id = fid
            item.object_name = empty.name

            if frustum_mesh is not None:
                frustum = bpy.data.objects.new(f"RUX_Frustum_{fid}", frustum_mesh)
                frustum.hide_select = True
                frustum.parent = empty
                if frustum_local_tf is not None:
                    frustum.matrix_local = frustum_local_tf
                coll.objects.link(frustum)

            poses.append(pose_np)

        # Edge mesh connecting nodes in trajectory order
        if len(poses) > 1:
            verts = [(float(p[0, 3]), float(p[1, 3]), float(p[2, 3])) for p in poses]
            edges = [(i, i + 1) for i in range(len(verts) - 1)]
            mesh = bpy.data.meshes.new(_POSE_GRAPH_EDGES)
            mesh.from_pydata(verts, edges, [])
            edge_obj = bpy.data.objects.new(_POSE_GRAPH_EDGES, mesh)
            coll.objects.link(edge_obj)


class REUSEX_OT_load_frame_cloud(bpy.types.Operator):
    bl_idname = "reusex.load_frame_cloud"
    bl_label = "Load Frame Cloud"
    bl_description = "Reconstruct and attach the depth-projected point cloud for the selected pose graph node"

    step: bpy.props.IntProperty(
        name="Sampling Step",
        description="Sample every Nth depth pixel (higher = fewer points, faster)",
        default=4, min=1, max=32,
    )

    @classmethod
    def poll(cls, context):
        obj = context.object
        return (
            obj is not None
            and obj.name.startswith("RUX_Frame_")
            and "rux_frame_id" in obj
            and context.scene.reusex.is_loaded
        )

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

        empty = context.object
        fid = empty["rux_frame_id"]
        props = context.scene.reusex

        try:
            db = reusex.ProjectDB(props.file_path, read_only=True)
            frame_data = db.reconstruct_frame(fid, step=self.step)
        except Exception as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}

        positions = np.asarray(frame_data["positions"], dtype=np.float32)
        colors    = np.asarray(frame_data["colors"],    dtype=np.uint8)
        n = len(positions)
        if n == 0:
            self.report({"WARNING"}, f"Frame {fid}: no points reconstructed")
            return {"CANCELLED"}

        # Replace existing cloud child if already loaded
        cloud_name = f"RUX_Cloud_{fid}"
        for old in [bpy.data.objects.get(cloud_name),
                    bpy.data.pointclouds.get(cloud_name)]:
            if old is not None:
                (bpy.data.objects if isinstance(old, bpy.types.Object)
                 else bpy.data.pointclouds).remove(old, do_unlink=True)

        pc_data = bpy.data.pointclouds.new(cloud_name)
        pc_data.resize(n)
        pc_data.attributes["position"].data.foreach_set("vector", positions.flatten())

        col_attr = pc_data.attributes.new("color", "FLOAT_COLOR", "POINT")
        rgba = np.ones((n, 4), dtype=np.float32)
        rgba[:, :3] = colors.astype(np.float32) / 255.0
        col_attr.data.foreach_set("color", rgba.flatten())

        mat = (bpy.data.materials.get("RUX_FrameCloud")
               or _make_color_attr_material("RUX_FrameCloud"))

        cloud_obj = bpy.data.objects.new(cloud_name, pc_data)
        cloud_obj.data.materials.append(mat)
        cloud_obj.hide_select = True
        cloud_obj.parent = empty

        try:
            from mathutils import Matrix
            intr = db.sensor_frame_intrinsics(fid)
            cloud_obj.matrix_local = Matrix(
                np.asarray(intr["local_transform"], dtype=np.float64).tolist()
            )
        except Exception:
            pass

        coll = bpy.data.collections.get(_POSE_GRAPH_COLLECTION)
        (coll if coll is not None else context.collection).objects.link(cloud_obj)

        self.report({"INFO"}, f"Frame {fid}: {n:,} points (step={self.step})")
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


class REUSEX_OT_clear_stale_cloud(bpy.types.Operator):
    bl_idname = "reusex.clear_stale_cloud"
    bl_label = "Clear Stale Cloud Reference"
    bl_options = {"INTERNAL"}

    cloud_name: StringProperty()

    def execute(self, context):
        props = context.scene.reusex
        item = next((c for c in props.clouds if c.name == self.cloud_name), None)
        if item is not None:
            item.loaded_object_name = ""
        return {"FINISHED"}


_classes = (
    REUSEX_OT_open_project,
    REUSEX_OT_close_project,
    REUSEX_OT_load_point_cloud,
    REUSEX_OT_load_pose_graph,
    REUSEX_OT_load_frame_cloud,
    REUSEX_OT_select_cloud_object,
    REUSEX_OT_clear_stale_cloud,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)
    if _edge_sync_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_edge_sync_handler)


def unregister():
    if _edge_sync_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_edge_sync_handler)
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
