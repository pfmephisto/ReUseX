// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { NO_FILTERS } from '../kortlaegning/model';
import {
  addableTypes,
  allPropertyGroups,
  cellModel,
  isManual,
  patchedResources,
  replaceResources,
  resourceColumnKeyId,
  resourceIndex,
  templateColumns,
  touchesSurvey,
  typeCarrier,
  valueOf,
  viewForNewResource,
} from '../kortlaegning/resources';
import { resource, resourceKey, SYS_KEYS, template } from './resourceFixtures';
import { surveyPart, surveyType } from './surveyFixtures';

const lex = resourceKey({ id: 'lex:abc', label: 'Producent', category: 'Identifikation', scope: 'part' });
const col = resourceKey({ id: 'col:4', label: 'Stand', category: 'Egne felter', scope: 'part', data_type: 'enum', options: ['God', 'Dårlig'] });
const catalogue = [...SYS_KEYS, lex, col];
const byId = (id: string) => catalogue.find((k) => k.id === id)!;

describe('templateColumns', () => {
  it('follows resolved_keys in order, skipping the tree column (sys:name)', () => {
    const cols = templateColumns(template({ resolved_keys: ['sys:name', 'sys:eak', 'col:4', 'sys:quantity'] }), catalogue);
    expect(cols.map((c) => c.key.id)).toEqual(['sys:eak', 'col:4', 'sys:quantity']);
  });

  it('marks type-scoped columns', () => {
    const cols = templateColumns(template({ resolved_keys: ['sys:eak', 'sys:note'] }), catalogue);
    expect(cols.map((c) => c.typeScoped)).toEqual([true, false]);
  });

  it('skips keys the catalogue does not know and duplicates', () => {
    const cols = templateColumns(template({ resolved_keys: ['lex:gone', 'sys:eak', 'sys:eak'] }), catalogue);
    expect(cols.map((c) => c.key.id)).toEqual(['sys:eak']);
  });

  it('is empty without a template', () => {
    expect(templateColumns(null, catalogue)).toEqual([]);
  });
});

describe('cellModel', () => {
  const t = surveyType({ id: 6, parts: [surveyPart({ code: 'RX-009' }), surveyPart({ code: 'RX-008' })] });
  const index = resourceIndex([
    resource({ code: 'RX-008', values: { 'sys:eak': '17.04.02', 'sys:note': 'Ren fuge' } }),
    resource({ code: 'RX-009', values: { 'sys:eak': '17.04.02' } }),
  ]);
  const column = (id: string) => ({ key: byId(id), typeScoped: byId(id).scope === 'type' });

  it('a part row edits its own value; a blank is null', () => {
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-008' }, t, column('sys:note'), index)).toEqual({ kind: 'value', value: 'Ren fuge', target: 'RX-008' });
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-009' }, t, column('sys:note'), index)).toEqual({ kind: 'value', value: null, target: 'RX-009' });
  });

  it('a read-only key has no target', () => {
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-008' }, t, column('sys:environment'), index).target).toBeNull();
  });

  it("a type row reads and writes a type-scoped key through its first part by code", () => {
    expect(typeCarrier(t)).toBe('RX-008');
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:eak'), index)).toEqual({ kind: 'value', value: '17.04.02', target: 'RX-008' });
  });

  it('a type row shows the Mængde aggregate and nothing for other part keys', () => {
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:quantity'), index).kind).toBe('aggregate');
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:note'), index)).toEqual({ kind: 'none', value: null, target: null });
  });

  it('a type without parts is read-only and blank', () => {
    const empty = surveyType({ id: 9, parts: [] });
    expect(typeCarrier(empty)).toBeNull();
    expect(cellModel({ kind: 'type', typeId: 9 }, empty, column('sys:eak'), index)).toEqual({ kind: 'value', value: null, target: null });
  });

  it('valueOf is null for an unknown code', () => {
    expect(valueOf(index, 'RX-999', 'sys:eak')).toBeNull();
  });
});

describe('merging patch responses', () => {
  it('replaces the patched resource and its type siblings, keeping the rest', () => {
    const list = [resource({ code: 'RX-008' }), resource({ code: 'RX-009' }), resource({ code: 'RX-001', type_id: 2 })];
    const result = {
      resource: resource({ code: 'RX-008', values: { 'sys:treatment': 'genbrug' } }),
      siblings: [resource({ code: 'RX-009', values: { 'sys:treatment': 'genbrug' } })],
    };
    const next = replaceResources(list, patchedResources(result));
    expect(next.map((r) => r.code)).toEqual(['RX-008', 'RX-009', 'RX-001']);
    expect(next[1].values['sys:treatment']).toBe('genbrug');
    expect(next[2]).toBe(list[2]);
  });

  it('appends a resource it did not have', () => {
    expect(replaceResources([], [resource({ code: 'RX-019' })]).map((r) => r.code)).toEqual(['RX-019']);
  });

  it('only sys: writes need the survey re-read', () => {
    expect(touchesSurvey(['col:4', 'lex:abc'])).toBe(false);
    expect(touchesSurvey(['col:4', 'sys:treatment'])).toBe(true);
  });
});

describe('resource helpers', () => {
  it('a part without an instance is manual', () => {
    expect(isManual(surveyPart({ instance_guid: null }))).toBe(true);
    expect(isManual(surveyPart({ instance_guid: 'g-1', orphaned: true }))).toBe(false);
  });

  it('a user column id becomes its key id', () => {
    expect(resourceColumnKeyId('4')).toBe('col:4');
  });

  it('Alle egenskaber groups the keys with a value by category, in catalogue order', () => {
    const r = resource({ values: { 'col:4': 'God', 'sys:note': 'x', 'lex:abc': 'Velux', 'sys:eak': null } });
    const groups = allPropertyGroups(r, catalogue);
    expect(groups.map((g) => [g.category, g.keys.map((k) => k.id)])).toEqual([
      ['Kortlægning', ['sys:note']],
      ['Identifikation', ['lex:abc']],
      ['Egne felter', ['col:4']],
    ]);
    expect(allPropertyGroups(null, catalogue)).toEqual([]);
  });

  it('a resource can be added to any type that is not rejected, by name', () => {
    const types = [surveyType({ id: 1, name: 'Ø' }), surveyType({ id: 2, name: 'A' }), surveyType({ id: 3, name: 'B', review_status: 'rejected' })];
    expect(addableTypes(types).map((t) => t.id)).toEqual([2, 1]);
  });

  it('shows a new resource whose type the current tab or filters hide', () => {
    const types = [surveyType({ id: 1, review_status: 'approved' })];
    expect(viewForNewResource(types, 1, 'queue', NO_FILTERS)).toEqual({ tab: 'all', filters: NO_FILTERS });
    const filters = { ...NO_FILTERS, search: 'vindue' };
    expect(viewForNewResource(types, 1, 'approved', filters)).toEqual({ tab: 'approved', filters });
  });
});
