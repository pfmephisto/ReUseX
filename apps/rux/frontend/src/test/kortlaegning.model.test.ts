// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType } from '../api/types';
import {
  flattenRows,
  initialViewFor,
  moveSelection,
  nextInQueue,
  NO_FILTERS,
  partLabel,
  partOf,
  removePart,
  removeType,
  reopenedView,
  replacePart,
  replaceType,
  roomName,
  roomOptions,
  tabCounts,
  typeOf,
  visibleTypes,
} from '../kortlaegning/model';
import { surveyType } from './surveyFixtures';

function part(code: string, typeId: number, room: [number, string] | null, quantity: number): SurveyPart {
  return {
    code,
    type_id: typeId,
    cloud: null,
    instance_id: null,
    room_id: room ? room[0] : null,
    room_name: room ? room[1] : '',
    quantity,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: null,
    orphaned: false,
  };
}

function type(id: number, name: string, over: Partial<SurveyType> = {}): SurveyType {
  return {
    id,
    name,
    eak_code: '17.01.01',
    eak_name: 'Beton',
    bim7aa_code: '',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.9,
    mass_t: 1,
    note: '',
    starred: false,
    semantic_class: -2,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 0,
    parts: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

const TYPES: SurveyType[] = [
  type(1, 'Betonsøjler, bærende', { parts: [part('RX-001', 1, [1, 'Production Hall'], 18), part('RX-002', 1, [4, 'Entrance'], 6)], quantity: 24 }),
  type(2, 'Vinduespartier, aluminium', { environment_status: 'afventer', parts: [part('RX-008', 2, [2, 'Office Zone'], 26)], quantity: 26 }),
  type(3, 'Isolering, mineraluld', { review_status: 'approved', parts: [part('RX-016', 3, [5, 'Roof'], 480)], quantity: 480 }),
  type(4, 'Fejldetektion', { review_status: 'rejected' }),
  type(5, 'Indvendige murvægge', { environment_status: 'forurenet', parts: [part('RX-017', 5, [1, 'Production Hall'], 170)], quantity: 170 }),
];

describe('kortlægning model', () => {
  it('counts rejected types in their own tab, never in Alle', () => {
    expect(tabCounts(TYPES)).toEqual({ queue: 3, approved: 1, all: 4, rejected: 1 });
  });

  it('lists only rejected types in the Afvist tab, and them in no other', () => {
    expect(visibleTypes(TYPES, 'rejected', NO_FILTERS).map((t) => t.id)).toEqual([4]);
    for (const tab of ['queue', 'approved', 'all'] as const)
      expect(visibleTypes(TYPES, tab, NO_FILTERS).map((t) => t.id)).not.toContain(4);
    expect(visibleTypes(TYPES, 'rejected', { ...NO_FILTERS, search: 'vindue' })).toEqual([]);
  });

  it('never picks a rejected type as the next in the queue', () => {
    const list = [type(1, 'a'), type(2, 'b', { review_status: 'rejected' }), type(3, 'c')];
    expect(nextInQueue(list, 1)).toEqual({ typeId: 3, partCode: null });
    expect(nextInQueue([type(2, 'b', { review_status: 'rejected' })], 2)).toBeNull();
  });

  it('removes a deleted type, and a deleted part with its quantity', () => {
    expect(removeType(TYPES, 2).map((t) => t.id)).toEqual([1, 3, 4, 5]);
    const p = removePart(TYPES, 'RX-002');
    expect(p[0].parts.map((x) => x.code)).toEqual(['RX-001']);
    expect(p[0].quantity).toBe(18);
    expect(p[1]).toBe(TYPES[1]);
    expect(removePart(TYPES, 'RX-404')).toEqual(TYPES);
  });

  it('filters by tab, search, room and miljø', () => {
    expect(visibleTypes(TYPES, 'queue', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 5]);
    expect(visibleTypes(TYPES, 'all', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 3, 5]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, search: 'VINDUE' }).map((t) => t.id)).toEqual([2]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, roomId: 1 }).map((t) => t.id)).toEqual([1, 5]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, env: 'ren' }).map((t) => t.id)).toEqual([1, 3]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, env: 'afventer' }).map((t) => t.id)).toEqual([2]);
  });

  it('flattens open groups into type + part rows', () => {
    const rows = flattenRows(visibleTypes(TYPES, 'queue', NO_FILTERS), new Set([1]));
    expect(rows).toEqual([
      { kind: 'type', typeId: 1 },
      { kind: 'part', typeId: 1, partCode: 'RX-001' },
      { kind: 'part', typeId: 1, partCode: 'RX-002' },
      { kind: 'type', typeId: 2 },
      { kind: 'type', typeId: 5 },
    ]);
  });

  it('moves the selection through rows and clamps at the ends', () => {
    const rows = flattenRows(visibleTypes(TYPES, 'queue', NO_FILTERS), new Set([1]));
    expect(moveSelection(rows, null, 1)).toEqual({ typeId: 1, partCode: null });
    expect(moveSelection(rows, { typeId: 1, partCode: null }, 1)).toEqual({ typeId: 1, partCode: 'RX-001' });
    expect(moveSelection(rows, { typeId: 5, partCode: null }, 1)).toEqual({ typeId: 5, partCode: null });
    expect(moveSelection(rows, { typeId: 1, partCode: null }, -1)).toEqual({ typeId: 1, partCode: null });
    expect(moveSelection([], null, 1)).toBeNull();
  });

  it('lists rooms that have parts, by name', () => {
    expect(roomOptions(TYPES)).toEqual([
      { id: 4, name: 'Entrance' },
      { id: 2, name: 'Office Zone' },
      { id: 1, name: 'Production Hall' },
      { id: 5, name: 'Roof' },
    ]);
  });

  it('picks the next approvable queued type, then any queued, then none', () => {
    expect(nextInQueue(TYPES, 1)).toEqual({ typeId: 5, partCode: null }); // 2 is afventer
    expect(nextInQueue(TYPES, 5)).toEqual({ typeId: 1, partCode: null }); // wraps
    const onlyPending = [type(2, 'x', { environment_status: 'afventer' })];
    expect(nextInQueue(onlyPending, 2)).toEqual({ typeId: 2, partCode: null });
    expect(nextInQueue([type(3, 'y', { review_status: 'approved' })], 3)).toBeNull();
  });

  it('picks the next queued type among the ones the tab and filters show', () => {
    const approved = replaceType(TYPES, { ...TYPES[0], review_status: 'approved' });
    const shown = (f: Partial<typeof NO_FILTERS>) => visibleTypes(approved, 'queue', { ...NO_FILTERS, ...f });
    // Unfiltered, 5 comes first (2 is afventer); filtered to Office Zone only 2 is shown.
    expect(nextInQueue(approved, 1, shown({}))).toEqual({ typeId: 5, partCode: null });
    expect(nextInQueue(approved, 1, shown({ roomId: 2 }))).toEqual({ typeId: 2, partCode: null });
    expect(nextInQueue(approved, 1, shown({ env: 'afventer' }))).toEqual({ typeId: 2, partCode: null });
    expect(nextInQueue(approved, 1, shown({ search: 'ingen sådan type' }))).toBeNull();
  });

  it('keeps its place in the list when the approved type has left the view', () => {
    const list = [type(10, 'a'), type(11, 'b', { review_status: 'approved' }), type(12, 'c')];
    const shown = visibleTypes(list, 'queue', NO_FILTERS); // 11 is no longer in it
    expect(nextInQueue(list, 11, shown)).toEqual({ typeId: 12, partCode: null });
  });

  it('switches to the queue tab on "Genåbn" so the reopened type stays selected and visible', () => {
    expect(reopenedView(4, null)).toEqual({ tab: 'queue', selection: { typeId: 4, partCode: null } });
    // A selected part under the reopened type stays selected too.
    expect(reopenedView(4, 'RX-030')).toEqual({ tab: 'queue', selection: { typeId: 4, partCode: 'RX-030' } });
  });

  it('picks a queued type to select after "Slet type", the same composition the page uses', () => {
    // "Slet type" removes the type entirely (unlike approve/reject, which only
    // change its review_status), so `afterTypeId` is gone from `next` too —
    // nextInQueue must still land on a queued type instead of nothing.
    const list = [type(1, 'a'), type(2, 'b'), type(3, 'c', { review_status: 'approved' })];
    const next = removeType(list, 1);
    expect(nextInQueue(next, 1, visibleTypes(next, 'queue', NO_FILTERS))).toEqual({
      typeId: 2,
      partCode: null,
    });
    // Deleting the last queued type leaves nothing to select.
    const onlyOne = removeType(next, 2);
    expect(nextInQueue(onlyOne, 2, visibleTypes(onlyOne, 'queue', NO_FILTERS))).toBeNull();
  });

  it('labels a part by code and room, falling back to the room id', () => {
    expect(partLabel(part('RX-001', 1, [1, 'Production Hall'], 1))).toBe('RX-001 · Production Hall');
    const unnamed = { ...part('RX-002', 1, [7, ''], 1) };
    expect(roomName(unnamed)).toBe('Rum 7');
    expect(partLabel(unnamed)).toBe('RX-002 · Rum 7');
    expect(partLabel(part('RX-003', 1, null, 1))).toBe('RX-003');
  });

  it('resolves the selected type and part', () => {
    expect(typeOf(TYPES, { typeId: 2, partCode: null })?.name).toBe('Vinduespartier, aluminium');
    expect(partOf(TYPES, { typeId: 1, partCode: 'RX-002' })?.quantity).toBe(6);
    expect(partOf(TYPES, { typeId: 1, partCode: null })).toBeNull();
  });

  it('replaces a type and a part from server responses', () => {
    const t = replaceType(TYPES, { ...TYPES[0], name: 'Søjler' });
    expect(t[0].name).toBe('Søjler');
    expect(t).not.toBe(TYPES);
    const p = replacePart(TYPES, { ...TYPES[0].parts[1], quantity: 10 });
    expect(p[0].parts[1].quantity).toBe(10);
    expect(p[0].quantity).toBe(28);
  });
});

describe('initialViewFor', () => {
  const types = [
    surveyType({ id: 2, review_status: 'queue' }),
    surveyType({ id: 3, review_status: 'approved' }),
    surveyType({ id: 4, review_status: 'rejected' }),
  ];

  it('opens a deep-linked type in the tab it lives in', () => {
    expect(initialViewFor(types, 2)).toEqual({ tab: 'queue', selection: { typeId: 2, partCode: null } });
    expect(initialViewFor(types, 3)).toEqual({ tab: 'approved', selection: { typeId: 3, partCode: null } });
  });

  it('opens a rejected type in the Afvist tab', () => {
    expect(initialViewFor(types, 4)).toEqual({ tab: 'rejected', selection: { typeId: 4, partCode: null } });
  });

  it('ignores unknown types', () => {
    expect(initialViewFor(types, 99)).toBeNull();
  });
});

describe('the ★ filter (Phase 6 R13)', () => {
  it('keeps starred types and types with a starred part', () => {
    const types = [
      type(1, 'Stålspær', { starred: true }),
      type(2, 'Vinduespartier', { parts: [{ ...part('RX-008', 2, [2, 'Office Zone'], 26), starred: true }] }),
      type(3, 'Betondæk', { parts: [part('RX-003', 3, [1, 'Production Hall'], 980)] }),
    ];
    expect(NO_FILTERS.starred).toBe(false);
    expect(visibleTypes(types, 'all', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 3]);
    expect(visibleTypes(types, 'all', { ...NO_FILTERS, starred: true }).map((t) => t.id)).toEqual([1, 2]);
  });
});
