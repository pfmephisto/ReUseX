// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  BLANK,
  booleanChoice,
  cellCommit,
  choiceCommit,
  cellDisplay,
  cellInputText,
  enumOptions,
  isTrue,
  isTriStateBoolean,
  optionLabel,
  shownChoice,
  toggleValue,
} from '../kortlaegning/cellDraft';
import { resourceKey } from './resourceFixtures';

const text = resourceKey({ id: 'col:1', label: 'Producent' });
const num = resourceKey({ id: 'sys:mass_t', label: 'Tons', scope: 'type', data_type: 'number', unit: 't' });
const date = resourceKey({ id: 'col:2', data_type: 'date' });
const treat = resourceKey({ id: 'sys:treatment', data_type: 'enum', options: ['genbrug', 'bortskaffelse'] });
const env = resourceKey({ id: 'sys:environment', data_type: 'enum', editable: false });
const stand = resourceKey({ id: 'col:4', data_type: 'enum', options: ['God', 'Dårlig'] });
const starred = resourceKey({ id: 'sys:starred', data_type: 'boolean' });
const flag = resourceKey({ id: 'col:5', data_type: 'boolean' });

describe('blank-to-filled and back', () => {
  it('a blank text cell filled sends the trimmed text', () => {
    expect(cellCommit(text, '  Velux ', null)).toEqual({ send: true, value: 'Velux' });
  });

  it('an untouched blank never sends', () => {
    expect(cellCommit(text, '', null)).toEqual({ send: false, invalid: false });
    expect(cellCommit(num, '   ', null)).toEqual({ send: false, invalid: false });
  });

  it('emptying a filled cell sends null (clear)', () => {
    expect(cellCommit(text, '', 'Velux')).toEqual({ send: true, value: null });
    expect(cellCommit(num, '', '3.1')).toEqual({ send: true, value: null });
  });

  it('a Danish number becomes a wire number', () => {
    expect(cellCommit(num, '12,5', null)).toEqual({ send: true, value: '12.5' });
    expect(cellCommit(num, '1.240', null)).toEqual({ send: true, value: '1240' });
  });

  it('an untouched filled number never sends, nor does an equal one', () => {
    expect(cellCommit(num, cellInputText(num, '3.1'), '3.1')).toEqual({ send: false, invalid: false });
    expect(cellCommit(num, '3,10', '3.1')).toEqual({ send: false, invalid: false });
  });

  it('a draft that does not parse is invalid and sends nothing', () => {
    expect(cellCommit(num, 'abc', null)).toEqual({ send: false, invalid: true });
    expect(cellCommit(num, '3.1', '2')).toEqual({ send: false, invalid: true }); // English decimal
    expect(cellCommit(date, '1/2/2026', null)).toEqual({ send: false, invalid: true });
  });

  it('a date sends ISO text', () => {
    expect(cellCommit(date, '2026-10-02', null)).toEqual({ send: true, value: '2026-10-02' });
  });

  it('an enum sends a listed option or null, nothing for the same value', () => {
    expect(cellCommit(stand, 'God', null)).toEqual({ send: true, value: 'God' });
    expect(cellCommit(stand, '', 'God')).toEqual({ send: true, value: null });
    expect(cellCommit(stand, 'God', 'God')).toEqual({ send: false, invalid: false });
    expect(cellCommit(stand, 'Middel', null)).toEqual({ send: false, invalid: true });
  });
});

describe('input text', () => {
  it('shows a wire number with a Danish comma, and keeps legacy text', () => {
    expect(cellInputText(num, '12.5')).toBe('12,5');
    expect(cellInputText(num, 'ca. 5')).toBe('ca. 5');
    expect(cellInputText(num, null)).toBe('');
    expect(cellInputText(text, 'x')).toBe('x');
  });
});

describe('display', () => {
  it('is empty for a blank, so the cell can draw the muted placeholder', () => {
    expect(cellDisplay(text, null)).toBe('');
    expect(cellDisplay(text, '  ')).toBe('');
    expect(BLANK).toBe('—');
  });

  it('formats numbers in Danish with the key unit', () => {
    expect(cellDisplay(num, '1240.5')).toBe('1.240,5 t');
  });

  it('labels treatment and miljøstatus in Danish', () => {
    expect(cellDisplay(treat, 'genbrug')).toBe('Genbrug');
    expect(cellDisplay(env, 'afventer')).toBe('Afventer prøve');
    expect(optionLabel(treat, 'bortskaffelse')).toBe('Bortskaffelse');
    expect(optionLabel(stand, 'God')).toBe('God');
  });

  it('shows ★ for Vigtig and Ja/Nej for other booleans', () => {
    expect(cellDisplay(starred, 'true')).toBe('★');
    expect(cellDisplay(starred, 'false')).toBe('');
    expect(cellDisplay(flag, '1')).toBe('Ja');
    expect(cellDisplay(flag, 'false')).toBe('Nej');
  });
});

describe('booleans and enum options', () => {
  it('reads tolerant truthy strings and toggles to wire booleans', () => {
    expect(['true', '1', 'Ja', 'YES'].map(isTrue)).toEqual([true, true, true, true]);
    expect([null, 'false', '0', ''].map(isTrue)).toEqual([false, false, false, false]);
    expect(toggleValue(null)).toBe('true');
    expect(toggleValue('true')).toBe('false');
  });

  it('keeps a legacy value the options no longer list, so the select can show it', () => {
    expect(enumOptions(stand, 'Middel')).toEqual(['God', 'Dårlig', 'Middel']);
    expect(enumOptions(stand, 'God')).toEqual(['God', 'Dårlig']);
    expect(enumOptions(stand, null)).toEqual(['God', 'Dårlig']);
  });
});

// -------------------------------------------------------------- R3-D1 ----

describe('non-clearable keys (R3-D1)', () => {
  const quantity = resourceKey({ id: 'sys:quantity', data_type: 'number', scope: 'part' });
  const name = resourceKey({ id: 'sys:name', scope: 'type' });

  it('an empty draft on a non-clearable key is invalid, not a clear', () => {
    expect(cellCommit(quantity, '', '26')).toEqual({ send: false, invalid: true });
    expect(cellCommit(name, '', 'Dør')).toEqual({ send: false, invalid: true });
    expect(cellCommit(treat, '', 'genbrug')).toEqual({ send: false, invalid: true });
    expect(cellCommit(starred, '', 'true')).toEqual({ send: false, invalid: true });
  });

  it('an untouched blank on a non-clearable key still sends nothing (not invalid)', () => {
    expect(cellCommit(quantity, '', null)).toEqual({ send: false, invalid: false });
  });

  it('a clearable key still clears on empty', () => {
    expect(cellCommit(stand, '', 'God')).toEqual({ send: true, value: null });
  });
});

// ------------------------------------------------------- R3-A1 / R3-D7 ----

describe('multiselect display (R3-A1): joined option labels, never an editable draft', () => {
  const material = resourceKey({
    id: 'lex:abc',
    label: 'Materialer',
    data_type: 'multiselect',
    options: ['concrete', 'steel'],
  });

  it('joins the option labels for display', () => {
    expect(cellDisplay(material, '["concrete","steel"]')).toBe('concrete, steel');
  });

  it('shows one item plainly, and falls back to the raw value when it is not a JSON array', () => {
    expect(cellDisplay(material, '["concrete"]')).toBe('concrete');
    expect(cellDisplay(material, '[]')).toBe('');
    expect(cellDisplay(material, 'concrete')).toBe('concrete');
  });

  it('a commit attempt is always invalid — there is no editor to produce one', () => {
    expect(cellCommit(material, 'concrete, steel', '["concrete"]')).toEqual({ send: false, invalid: true });
  });

  it('leaves an untouched draft alone', () => {
    expect(cellCommit(material, cellInputText(material, '["concrete"]'), '["concrete"]')).toEqual({
      send: false,
      invalid: false,
    });
  });
});

describe('string-array display (R3-D7): a leksikon StringArray field joined, not raw JSON', () => {
  const docs = resourceKey({ id: 'lex:def', label: 'Dokumenter', data_type: 'text', editable: false });

  it('joins a JSON array of strings for display', () => {
    expect(cellDisplay(docs, '["a.pdf","b.pdf"]')).toBe('a.pdf, b.pdf');
  });

  it('shows ordinary text and malformed JSON as-is', () => {
    expect(cellDisplay(text, 'Velux')).toBe('Velux');
    expect(cellDisplay(docs, '[not json')).toBe('[not json');
  });
});

// ------------------------------------------------------------- R3-D2 ----

describe('yes/no/unknown display (R3-D2): a leksikon TriState field in Danish', () => {
  const triState = resourceKey({ id: 'lex:ghi', label: 'Bærende', data_type: 'enum', options: ['yes', 'no', 'unknown'] });

  it('labels yes/no/unknown as Ja/Nej/Ukendt', () => {
    expect(cellDisplay(triState, 'yes')).toBe('Ja');
    expect(cellDisplay(triState, 'no')).toBe('Nej');
    expect(cellDisplay(triState, 'unknown')).toBe('Ukendt');
    expect(optionLabel(triState, 'yes')).toBe('Ja');
  });
});

// ------------------------------------------------------ final fix wave ----

describe('ISO dates are checked for month and day bounds', () => {
  it('rejects a month or day that does not exist', () => {
    expect(cellCommit(date, '2026-13-40', null)).toEqual({ send: false, invalid: true });
    expect(cellCommit(date, '2026-00-10', null)).toEqual({ send: false, invalid: true });
    expect(cellCommit(date, '2026-02-30', null)).toEqual({ send: false, invalid: true });
    expect(cellCommit(date, '2025-02-29', null)).toEqual({ send: false, invalid: true });
  });

  it('accepts real dates, leap days included', () => {
    expect(cellCommit(date, '2024-02-29', null)).toEqual({ send: true, value: '2024-02-29' });
    expect(cellCommit(date, '2026-12-31', null)).toEqual({ send: true, value: '2026-12-31' });
  });
});

describe('JSON-array joining is for read-only keys only', () => {
  it('an editable text key shows its stored text verbatim, brackets and all', () => {
    expect(cellDisplay(text, '["a","b"]')).toBe('["a","b"]');
  });
});

describe('tri-state booleans', () => {
  it('a clearable boolean is tri-state; Vigtig is not', () => {
    expect(isTriStateBoolean(flag)).toBe(true);
    expect(isTriStateBoolean(starred)).toBe(false);
    expect(isTriStateBoolean(text)).toBe(false);
  });

  it('maps a stored value onto the select choice, blank included', () => {
    expect(booleanChoice(null)).toBe('');
    expect(booleanChoice('  ')).toBe('');
    expect(booleanChoice('1')).toBe('true');
    expect(booleanChoice('false')).toBe('false');
    expect(booleanChoice('nej')).toBe('false');
  });

  it('blank → Ja sends true, Ja → — clears, the same choice sends nothing', () => {
    expect(cellCommit(flag, 'true', null)).toEqual({ send: true, value: 'true' });
    expect(cellCommit(flag, '', 'true')).toEqual({ send: true, value: null });
    expect(cellCommit(flag, 'false', 'true')).toEqual({ send: true, value: 'false' });
    expect(cellCommit(flag, 'true', '1')).toEqual({ send: false, invalid: false });
  });

  it('a blank boolean displays the muted placeholder, not Nej', () => {
    expect(cellDisplay(flag, null)).toBe('');
  });
});

describe('optimistic choices', () => {
  it('shows the pending choice while its PATCH is in flight, else the stored value', () => {
    expect(shownChoice('false', undefined)).toBe('false');
    expect(shownChoice('false', { value: 'true' })).toBe('true');
    expect(shownChoice('true', { value: null })).toBeNull();
  });

  it('a rapid double toggle sends true then false, never the same value twice', () => {
    const first = toggleValue(shownChoice('false', undefined));
    const second = toggleValue(shownChoice('false', { value: first }));
    expect([first, second]).toEqual(['true', 'false']);
  });

  it('choosing what is already shown (pending included) sends nothing', () => {
    expect(choiceCommit(stand, 'God', 'Dårlig', { value: 'God' })).toEqual({ send: false, invalid: false });
    expect(choiceCommit(stand, 'Dårlig', 'Dårlig', { value: 'God' })).toEqual({ send: true, value: 'Dårlig' });
  });
});
