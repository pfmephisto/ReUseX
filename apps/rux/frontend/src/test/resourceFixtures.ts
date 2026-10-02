// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Builders for resource keys, resources and templates in the Phase 3 (resources) tests. */

import type { Resource, ResourceKey, Template } from '../api/types';

export function resourceKey(over: Partial<ResourceKey> = {}): ResourceKey {
  return {
    id: 'sys:note',
    label: 'Note',
    category: 'Kortlægning',
    scope: 'part',
    data_type: 'text',
    unit: null,
    options: [],
    editable: true,
    ...over,
  };
}

export function resource(over: Partial<Resource> = {}): Resource {
  return { code: 'RX-008', type_id: 6, manual: true, values: {}, ...over };
}

export function template(over: Partial<Template> = {}): Template {
  return {
    id: 1,
    name: 'Hurtig genbrugsscreening',
    members: [],
    csv: {},
    seed: 'screening',
    resolved_keys: [],
    missing: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

/** The screening seed's built-in keys, in catalogue order (spec §4.3). */
export const SYS_KEYS: ResourceKey[] = [
  resourceKey({ id: 'sys:name', label: 'Betegnelse', scope: 'type' }),
  resourceKey({ id: 'sys:quantity', label: 'Mængde', scope: 'part', data_type: 'number' }),
  resourceKey({ id: 'sys:unit', label: 'Enhed', scope: 'type' }),
  resourceKey({ id: 'sys:eak', label: 'EAK', scope: 'type' }),
  resourceKey({ id: 'sys:treatment', label: 'Behandling', scope: 'type', data_type: 'enum', options: ['bevaring', 'genbrug', 'genanvendelse', 'nyttiggoerelse', 'bortskaffelse'] }),
  resourceKey({ id: 'sys:environment', label: 'Miljøstatus', scope: 'type', data_type: 'enum', editable: false }),
  resourceKey({ id: 'sys:room', label: 'Rum', scope: 'part' }),
  resourceKey({ id: 'sys:mass_t', label: 'Tons', scope: 'type', data_type: 'number', unit: 't' }),
  resourceKey({ id: 'sys:note', label: 'Note', scope: 'part' }),
  resourceKey({ id: 'sys:starred', label: 'Vigtig', scope: 'part', data_type: 'boolean' }),
];
