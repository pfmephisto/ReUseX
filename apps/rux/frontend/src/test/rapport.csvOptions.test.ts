// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  csvCountLine,
  csvDownloadErrorMessage,
  csvFilename,
  CSV_DEFAULTS,
  CSV_FALLBACK_FILENAME,
  DELIMITER_OPTIONS,
  downloadState,
  readCsvOptions,
  writeCsvOptions,
} from '../rapport/csvOptions';

const err = (status: number, message: string) => new ApiRequestError(status, message, '/api/v1/resources/export.csv');

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

describe('csvCountLine', () => {
  it('counts the template fields and names the leading code column', () => {
    expect(csvCountLine(11)).toBe('11 felter + kode · én række pr. ressource');
    expect(csvCountLine(1)).toBe('1 felt + kode · én række pr. ressource');
    expect(csvCountLine(0)).toBe('0 felter + kode · én række pr. ressource');
  });
});

describe('csvFilename', () => {
  it('falls back to ressourcer.csv with no header', () => {
    expect(csvFilename(null)).toBe(CSV_FALLBACK_FILENAME);
    expect(csvFilename('')).toBe(CSV_FALLBACK_FILENAME);
  });

  it('reads a plain filename, quoted or not', () => {
    expect(csvFilename('attachment; filename="ressourcer.csv"')).toBe('ressourcer.csv');
    expect(csvFilename('attachment; filename=ressourcer.csv')).toBe('ressourcer.csv');
  });

  it('prefers the RFC 5987 filename* form and decodes it', () => {
    expect(csvFilename(`attachment; filename="r.csv"; filename*=UTF-8''ressourcer%20(2).csv`)).toBe(
      'ressourcer (2).csv',
    );
  });

  it('falls back on a header with neither form, or malformed percent-encoding', () => {
    expect(csvFilename('attachment')).toBe(CSV_FALLBACK_FILENAME);
    expect(csvFilename(`attachment; filename*=UTF-8''%`)).toBe(CSV_FALLBACK_FILENAME);
  });
});

describe('csvDownloadErrorMessage', () => {
  it('says a deleted template by name', () => {
    expect(csvDownloadErrorMessage(err(404, "no template '9'"))).toBe('Skabelonen findes ikke længere.');
  });
  it('says the server is not ready for a 503', () => {
    expect(csvDownloadErrorMessage(err(503, 'locked'))).toBe(
      'Kunne ikke hente CSV-filen — serveren er ikke klar. Prøv igen om lidt.',
    );
  });
  it('falls back to the server message for other errors', () => {
    expect(csvDownloadErrorMessage(err(500, 'boom'))).toBe('Kunne ikke hente CSV-filen: boom');
  });
  it('handles a non-ApiRequestError cause (e.g. a network failure)', () => {
    expect(csvDownloadErrorMessage(new TypeError('Failed to fetch'))).toBe('Kunne ikke hente CSV-filen: Failed to fetch');
  });
});
