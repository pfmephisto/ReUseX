# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

bl_info = {
    "name": "ReUseX Project Panel",
    "author": "Povl Filip Sonne-Frederiksen",
    "version": (0, 1, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > ReUseX",
    "description": "Open and inspect ReUseX .rux project databases",
    "category": "3D View",
}

from . import operators, panels, properties


def register():
    properties.register()
    operators.register()
    panels.register()


def unregister():
    panels.unregister()
    operators.unregister()
    properties.unregister()
