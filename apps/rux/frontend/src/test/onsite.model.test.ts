// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyType } from '../api/types';
import type { GateChange } from '../miljoe/model';
import {
  chipDetail,
  currentStop,
  NO_ROOM,
  nextLabel,
  onsiteSampleBody,
  partAt,
  PHOTO_TEXT,
  photoView,
  pickerGroups,
  reticleBox,
  sampleToast,
  starButton,
  stopAfter,
  typeSamples,
  unknownNotice,
  walkOrder,
} from '../onsite/model';
import { sample, surveyPart, surveyType } from './surveyFixtures';

const room = (id: number, name: string) => ({ room_id: id, room_name: name });

const TYPES: SurveyType[] = [
  surveyType({
    id: 1,
    name: 'Fundamenter & terrændæk, beton',
    parts: [
      surveyPart({ code: 'RX-010', type_id: 1, ...room(1, 'Production Hall') }),
      surveyPart({ code: 'RX-011', type_id: 1, ...room(2, 'Office Zone') }),
    ],
  }),
  surveyType({
    id: 6,
    name: 'Vinduespartier, aluminium',
    parts: [
      surveyPart({ code: 'RX-009', type_id: 6, ...room(1, 'Production Hall') }),
      surveyPart({ code: 'RX-008', type_id: 6, ...room(2, 'Office Zone') }),
    ],
  }),
  surveyType({
    id: 2,
    name: 'Betonsøjler, bærende',
    parts: [surveyPart({ code: 'RX-002', type_id: 2, ...room(4, 'Entrance') })],
  }),
  surveyType({
    id: 3,
    name: 'Løse dele',
    parts: [surveyPart({ code: 'RX-030', type_id: 3, room_id: null, room_name: '' })],
  }),
  surveyType({
    id: 4,
    name: 'Kælder',
    parts: [surveyPart({ code: 'RX-040', type_id: 4, ...room(7, 'Ældre fløj') })],
  }),
  surveyType({
    id: 9,
    name: 'Fejldetektion',
    review_status: 'rejected',
    parts: [surveyPart({ code: 'RX-020', type_id: 9, ...room(2, 'Office Zone') })],
  }),
];

const NO_GATE: GateChange = {
  unblocked: [],
  contaminated: [],
  blocked: [],
  reblocked: [],
  released: [],
};

describe('walk order (R6)', () => {
  it('walks rooms in Danish order, then codes; roomless last; rejected types left out', () => {
    expect(walkOrder(TYPES).map((s) => s.code)).toEqual([
      'RX-002',
      'RX-008',
      'RX-011',
      'RX-009',
      'RX-010',
      'RX-040',
      'RX-030',
    ]);
    expect(walkOrder(TYPES).at(-1)).toEqual({
      code: 'RX-030',
      typeId: 3,
      room: '',
    });
  });

  it('starts at the asked part, else the first, and says when the asked one is unknown', () => {
    const order = walkOrder(TYPES);
    expect(currentStop(order, 'RX-009')?.code).toBe('RX-009');
    expect(currentStop(order, null)?.code).toBe('RX-002');
    expect(currentStop(order, 'RX-404')?.code).toBe('RX-002');
    expect(currentStop([], 'RX-002')).toBeNull();
    expect(unknownNotice('RX-404', currentStop(order, 'RX-404'))).toBe('RX-404 findes ikke — viser RX-002.');
    expect(unknownNotice('RX-009', currentStop(order, 'RX-009'))).toBeNull();
    expect(unknownNotice(null, currentStop(order, null))).toBeNull();
    expect(unknownNotice('RX-020', currentStop(order, 'RX-020'))).toBe('RX-020 findes ikke — viser RX-002.');
  });

  it('moves on, wrapping at the end, and names the next part', () => {
    const order = walkOrder(TYPES);
    expect(stopAfter(order, 'RX-008')?.code).toBe('RX-011');
    expect(stopAfter(order, 'RX-030')?.code).toBe('RX-002');
    expect(stopAfter(order.slice(0, 1), 'RX-002')).toBeNull();
    expect(nextLabel(stopAfter(order, 'RX-008'))).toBe('Videre → RX-011');
    expect(nextLabel(null)).toBe('Ingen flere bygningsdele');
  });

  it('finds the stop’s type and part', () => {
    const at = partAt(TYPES, currentStop(walkOrder(TYPES), 'RX-008'));
    expect(at?.type.id).toBe(6);
    expect(at?.part.code).toBe('RX-008');
    expect(partAt(TYPES, null)).toBeNull();
  });

  it('groups the picker by room, in walk order', () => {
    const groups = pickerGroups(walkOrder(TYPES), TYPES);
    expect(groups.map((g) => g.room)).toEqual(['Entrance', 'Office Zone', 'Production Hall', 'Ældre fløj', 'Uden rum']);
    expect(groups[1].options).toEqual([
      { code: 'RX-008', label: 'RX-008 · Vinduespartier, aluminium' },
      { code: 'RX-011', label: 'RX-011 · Fundamenter & terrændæk, beton' },
    ]);
  });
});

describe('the stage (R6)', () => {
  it('words the detection chip from the part and its type', () => {
    const t = surveyType({ confidence: 0.82 });
    expect(chipDetail(t, surveyPart())).toBe('RX-008 · Office Zone · sikkerhed 82 %');
    expect(chipDetail(surveyType({ confidence: null }), surveyPart({ room_id: null, room_name: '' }))).toBe('RX-008');
  });

  it('shows the photo only for the current part’s own lookup', () => {
    const frame = {
      frame_id: 41,
      centrality: 0.1,
      score: 0.9,
      depth: 2.1,
      u: 320,
      v: 240,
    };
    expect(photoView(null, undefined)).toEqual({ kind: 'unlinked' });
    expect(photoView('instances/7', undefined)).toEqual({ kind: 'loading' });
    expect(
      photoView('instances/7', {
        key: 'instances/6',
        frames: [frame],
        failed: false,
      }),
    ).toEqual({ kind: 'loading' });
    expect(
      photoView('instances/7', {
        key: 'instances/7',
        frames: [],
        failed: true,
      }),
    ).toEqual({ kind: 'failed' });
    expect(
      photoView('instances/7', {
        key: 'instances/7',
        frames: [],
        failed: false,
      }),
    ).toEqual({ kind: 'none' });
    expect(
      photoView('instances/7', {
        key: 'instances/7',
        frames: [frame],
        failed: false,
      }),
    ).toEqual({ kind: 'photo', frame });
    expect(PHOTO_TEXT.unlinked).toBe('Intet foto — bygningsdelen er ikke koblet til en instans.');
  });

  it('centres the reticle on the projected centroid, inside the stage', () => {
    expect(reticleBox({ u: 320, v: 240 }, { width: 640, height: 480 })).toEqual({
      left: 26,
      top: 30,
      width: 48,
      height: 40,
    });
    expect(reticleBox({ u: 0, v: 0 }, { width: 640, height: 480 })).toEqual({
      left: 0,
      top: 0,
      width: 48,
      height: 40,
    });
    expect(reticleBox({ u: 640, v: 480 }, { width: 640, height: 480 })).toEqual({
      left: 52,
      top: 60,
      width: 48,
      height: 40,
    });
    expect(reticleBox({ u: -5, v: 10 }, { width: 640, height: 480 })).toBeNull();
    expect(reticleBox({ u: 10, v: 10 }, {})).toBeNull();
    expect(reticleBox(undefined, { width: 640, height: 480 })).toBeNull();
  });
});

describe('the sheet (R7)', () => {
  it('words the star toggle', () => {
    expect(starButton(false)).toEqual({ icon: '☆', text: 'Markér som vigtig' });
    expect(starButton(true)).toEqual({
      icon: '★',
      text: 'Vigtig — tryk for at fjerne',
    });
  });

  it('registers a sample at the part, already taken', () => {
    expect(onsiteSampleBody('  Asbest i fugemasse ', ' Fuge mod nord ', surveyPart())).toEqual({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
      part_code: 'RX-008',
      stage: 'udtaget',
    });
    expect(onsiteSampleBody('   ', 'x', surveyPart())).toBeNull();
  });

  it('toasts the new code, and the gate effect only when the type just started to wait', () => {
    expect(sampleToast('P-04', 'RX-008', NO_GATE)).toBe('✓ P-04 registreret ved RX-008');
    const t = surveyType({ id: 2, name: 'Betonsøjler, bærende' });
    expect(sampleToast('P-04', 'RX-002', { ...NO_GATE, blocked: [t] })).toBe(
      '✓ P-04 registreret ved RX-002 — Betonsøjler, bærende afventer nu prøvesvar',
    );
    expect(sampleToast('P-04', 'RX-002', { ...NO_GATE, reblocked: [t] })).toBe(
      '✓ P-04 registreret ved RX-002 — Betonsøjler, bærende er godkendt, men afventer nu prøvesvar',
    );
  });

  it('lists the type’s samples and marks the ones taken here', () => {
    const t = surveyType({ id: 6 });
    const list = [
      sample({ id: 1, type_ids: [6] }),
      sample({ id: 2, code: 'P-02', type_ids: [11] }),
      sample({
        id: 4,
        code: 'P-04',
        type_ids: [6],
        part_code: 'RX-008',
        stage: 'udtaget',
      }),
    ];
    expect(typeSamples(t, surveyPart(), list).map((r) => [r.sample.code, r.here])).toEqual([
      ['P-01', false],
      ['P-04', true],
    ]);
  });
});

describe('edges', () => {
  it('never throws on an empty or unknown walk', () => {
    expect(walkOrder([])).toEqual([]);
    expect(pickerGroups([], TYPES)).toEqual([]);
    expect(stopAfter([], 'RX-002')).toBeNull();
    expect(stopAfter(walkOrder(TYPES), 'RX-404')).toBeNull();
    expect(unknownNotice('RX-404', null)).toBeNull();
    expect(partAt(TYPES, { code: 'RX-404', typeId: 6, room: '' })).toBeNull();
    expect(partAt(TYPES, { code: 'RX-008', typeId: 99, room: '' })).toBeNull();
  });

  it('names a numbered room and sorts codes numerically', () => {
    const types = [
      surveyType({
        id: 1,
        parts: [
          surveyPart({ code: 'RX-10', type_id: 1, room_id: 3, room_name: '' }),
          surveyPart({ code: 'RX-9', type_id: 1, room_id: 3, room_name: '' }),
        ],
      }),
    ];
    expect(walkOrder(types)).toEqual([
      { code: 'RX-9', typeId: 1, room: 'Rum 3' },
      { code: 'RX-10', typeId: 1, room: 'Rum 3' },
    ]);
    expect(NO_ROOM).toBe('Uden rum');
  });

  it('reads an id-tagged lookup the way Kortlægning does', () => {
    const frame = {
      frame_id: 41,
      centrality: 0.1,
      score: 0.9,
      depth: 2.1,
      u: 320,
      v: 240,
    };
    expect(
      photoView('instances/7', {
        key: 'instances/6',
        frames: [],
        failed: true,
      }),
    ).toEqual({ kind: 'loading' });
    expect(photoView(null, { key: '', frames: [frame], failed: false })).toEqual({ kind: 'unlinked' });
    expect(PHOTO_TEXT.loading).toBe('Indlæser foto…');
  });

  it('keeps the reticle inside the stage at the edges and drops a NaN projection', () => {
    const size = { width: 640, height: 480 };
    expect(reticleBox({ u: 640, v: 0 }, size)).toEqual({
      left: 52,
      top: 0,
      width: 48,
      height: 40,
    });
    expect(reticleBox({ u: 641, v: 10 }, size)).toBeNull();
    expect(reticleBox({ u: 10, v: 481 }, size)).toBeNull();
    expect(reticleBox({ u: Number.NaN, v: 10 }, size)).toBeNull();
    expect(reticleBox({ u: 10, v: Number.NaN }, size)).toBeNull();
    expect(reticleBox({ u: 10, v: 10 }, { width: Number.NaN, height: 480 })).toBeNull();
    expect(reticleBox({ u: 10, v: 10 }, { width: 0, height: 480 })).toBeNull();
  });

  it('words both gate effects together, and ignores the ones a new sample cannot cause', () => {
    const a = surveyType({ id: 2, name: 'Betonsøjler, bærende' });
    const b = surveyType({ id: 6, name: 'Vinduespartier, aluminium' });
    expect(
      sampleToast('P-04', 'RX-002', {
        ...NO_GATE,
        blocked: [a],
        reblocked: [b],
      }),
    ).toBe(
      '✓ P-04 registreret ved RX-002 — Betonsøjler, bærende afventer nu prøvesvar; Vinduespartier, aluminium er godkendt, men afventer nu prøvesvar',
    );
    expect(
      sampleToast('P-04', 'RX-002', {
        ...NO_GATE,
        unblocked: [a],
        released: [b],
        contaminated: [a],
      }),
    ).toBe('✓ P-04 registreret ved RX-002');
  });

  it('sends the part’s own type, with an empty what', () => {
    expect(onsiteSampleBody('Bly', '', surveyPart({ code: 'RX-002', type_id: 2 }))).toEqual({
      title: 'Bly',
      what: '',
      type_ids: [2],
      part_code: 'RX-002',
      stage: 'udtaget',
    });
  });
});
