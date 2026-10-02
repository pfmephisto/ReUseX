// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { CSV_DEFAULTS, DELIMITER_OPTIONS, downloadState, readCsvOptions, writeCsvOptions } from '../rapport/csvOptions';

describe('readCsvOptions', () => {
  it('fills defaults for an empty or foreign csv object', () => {
    expect(readCsvOptions({})).toEqual(CSV_DEFAULTS);
    expect(readCsvOptions(null)).toEqual(CSV_DEFAULTS);
    expect(readCsvOptions('nonsense')).toEqual(CSV_DEFAULTS);
  });

  it('reads valid values and drops invalid ones field by field', () => {
    expect(readCsvOptions({ delimiter: ';', encoding: 'utf-8-bom', header: 'key' })).toEqual({
      delimiter: ';',
      encoding: 'utf-8-bom',
      header: 'key',
    });
    expect(readCsvOptions({ delimiter: '|', header: 'key' })).toEqual({ ...CSV_DEFAULTS, header: 'key' });
  });
});

describe('writeCsvOptions (R11)', () => {
  it('merges the change and keeps keys it does not own', () => {
    expect(writeCsvOptions({ columns: ['a'], delimiter: ',' }, { delimiter: ';' })).toEqual({
      columns: ['a'],
      delimiter: ';',
    });
  });

  it('starts from an empty object when csv is not one', () => {
    expect(writeCsvOptions(undefined, { header: 'key' })).toEqual({ header: 'key' });
  });
});

describe('option lists', () => {
  it('offers semicolon, comma and tab', () => {
    expect(DELIMITER_OPTIONS.map((o) => o.value)).toEqual([';', ',', '\t']);
  });
});

describe('downloadState (R12)', () => {
  it('needs a template with fields and no write in flight', () => {
    expect(downloadState(null, false)).toEqual({ enabled: false, reason: 'Vælg en skabelon.' });
    expect(downloadState({ resolved_keys: [] }, false)).toEqual({
      enabled: false,
      reason: 'Skabelonen har ingen felter — tilføj nogle under Skabeloner.',
    });
    expect(downloadState({ resolved_keys: ['sys:name'] }, true)).toEqual({
      enabled: false,
      reason: 'Gemmer CSV-indstillingerne…',
    });
    expect(downloadState({ resolved_keys: ['sys:name'] }, false)).toEqual({ enabled: true, reason: null });
  });
});
