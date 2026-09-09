// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { EmptyState } from 'reusex-gui';

/** No meshes yet — the normal state right after import, before `rux create mesh`. */
export const NoMeshes = () => (
  <EmptyState
    title="No meshes yet"
    detail="Meshes are produced by `rux create mesh` once planes and rooms are segmented."
  />
);

/** No material passports — same low-key treatment, different producing stage. */
export const NoMaterials = () => (
  <EmptyState
    title="No material passports yet"
    detail="Run `rux create materials` after instances are reconciled to generate MaterialEPAS records."
  />
);

/** With an action: a button that jumps to the stage that would populate this list. */
export const WithAction = () => (
  <EmptyState
    title="No building components recorded"
    detail="Components are extracted during mesh generation from the cell complex solve."
    action={
      <button type="button">Run `rux create mesh`</button>
    }
  />
);

/** Title only — the minimal contract, no detail sentence, no action. */
export const TitleOnly = () => <EmptyState title="No panoramas imported" />;
