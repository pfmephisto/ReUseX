# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    IntProperty,
    StringProperty,
)
from bpy.types import PropertyGroup


class REUSEX_CloudItem(PropertyGroup):
    name: StringProperty(name="Name")
    cloud_type: StringProperty(name="Type")
    point_count: IntProperty(name="Points")
    organized: BoolProperty(name="Organized")


class REUSEX_MeshItem(PropertyGroup):
    name: StringProperty(name="Name")
    vertex_count: IntProperty(name="Vertices")
    face_count: IntProperty(name="Faces")


class REUSEX_MaterialItem(PropertyGroup):
    name: StringProperty(name="Name")
    guid: StringProperty(name="GUID")
    property_count: IntProperty(name="Properties")
    version_number: StringProperty(name="Version")


class REUSEX_ProjectItem(PropertyGroup):
    name: StringProperty(name="Name")
    project_name: StringProperty(name="Project Name")
    building_address: StringProperty(name="Address")
    year_of_construction: IntProperty(name="Year")
    survey_date: StringProperty(name="Survey Date")
    survey_organisation: StringProperty(name="Organisation")
    notes: StringProperty(name="Notes")


class REUSEX_ProjectProperties(PropertyGroup):
    file_path: StringProperty(name="File", subtype="FILE_PATH")
    is_loaded: BoolProperty(name="Loaded", default=False)
    schema_version: IntProperty(name="Schema Version")

    # Sensor frame stats
    sensor_frame_count: IntProperty(name="Sensor Frames")
    sensor_frame_width: IntProperty(name="Frame Width")
    sensor_frame_height: IntProperty(name="Frame Height")
    segmented_frame_count: IntProperty(name="Segmented Frames")

    # Panoramic stats
    panoramic_count: IntProperty(name="Panoramic Images")
    panoramic_matched: IntProperty(name="Matched Panoramics")

    # Component count
    component_count: IntProperty(name="Components")

    # Collection properties
    clouds: CollectionProperty(type=REUSEX_CloudItem)
    meshes: CollectionProperty(type=REUSEX_MeshItem)
    materials: CollectionProperty(type=REUSEX_MaterialItem)
    projects: CollectionProperty(type=REUSEX_ProjectItem)


_classes = (
    REUSEX_CloudItem,
    REUSEX_MeshItem,
    REUSEX_MaterialItem,
    REUSEX_ProjectItem,
    REUSEX_ProjectProperties,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.reusex = bpy.props.PointerProperty(type=REUSEX_ProjectProperties)


def unregister():
    del bpy.types.Scene.reusex
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
