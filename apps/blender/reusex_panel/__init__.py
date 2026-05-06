# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import importlib

from . import operators, panels, properties

# Reload order matches the dependency graph: properties first (defines types used
# by operators and panels).  This makes Blender's "Edit → Reload Scripts"
# (or F3 → "Reload Scripts") pick up live code edits without restarting Blender.
_modules = (properties, operators, panels)


def register():
    for mod in _modules:
        importlib.reload(mod)
    properties.register()
    operators.register()
    panels.register()


def unregister():
    panels.unregister()
    operators.unregister()
    properties.unregister()
