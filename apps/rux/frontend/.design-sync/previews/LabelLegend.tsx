// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { LabelLegend } from 'reusex-gui';

const buildingClasses: Record<string, string> = {
  '1': 'wall',
  '2': 'floor',
  '3': 'ceiling',
  '4': 'window',
  '5': 'door',
  '6': 'radiator',
  '7': 'column',
  '8': 'beam',
};

const manyClasses: Record<string, string> = {
  '1': 'wall',
  '2': 'floor',
  '3': 'ceiling',
  '4': 'window',
  '5': 'door',
  '6': 'radiator',
  '7': 'column',
  '8': 'beam',
  '9': 'stair',
  '10': 'railing',
  '11': 'furniture',
  '12': 'pipe',
  '13': 'duct',
  '14': 'cable_tray',
  '15': 'sprinkler',
  '16': 'light_fixture',
  '17': 'socket',
  '18': 'switch',
  '19': 'hvac_unit',
  '20': 'slab',
  '21': 'curtain_wall',
  '22': 'facade_panel',
  '23': 'roof',
  '24': 'skylight',
  '25': 'parapet',
  '26': 'balcony',
  '27': 'canopy',
  '28': 'terrace',
  '29': 'plinth',
  '30': 'sunroom_annotated_class_extending_the_swatch_name_width',
};

/** The default annotate-360 class set: eight labels, all shown, no truncation. */
export const Default = () => <LabelLegend labels={buildingClasses} />;

/** A 30-class segmentation run capped at 10 — the "+N more" summary kicks in. */
export const ManyLabels = () => <LabelLegend labels={manyClasses} limit={10} />;

/** Just after `create project`: only walls and floors have been labelled yet. */
export const MinimalSet = () => (
  <LabelLegend labels={{ '1': 'wall', '2': 'floor', '3': 'ceiling' }} />
);

/** A single instance class — no swatch list chrome, no "more" line. */
export const SingleClass = () => <LabelLegend labels={{ '6': 'radiator' }} />;
