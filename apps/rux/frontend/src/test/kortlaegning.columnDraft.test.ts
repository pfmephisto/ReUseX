// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  COLUMN_KIND_LABEL,
  columnCreateBody,
  columnCreateConflict,
  columnDraftError,
  columnPartialFailureMessage,
  duplicateFirst,
  EMPTY_COLUMN_DRAFT,
  parseOptions,
  seedNote,
} from '../kortlaegning/columnDraft';
import { template } from './resourceFixtures';

describe('parseOptions', () => {
  it('splits on commas and new lines, trims, drops blanks and case-insensitive repeats', () => {
    expect(parseOptions(' God, Dårlig\nGod\n\n middel ,god')).toEqual(['God', 'Dårlig', 'middel']);
  });
});

describe('columnDraftError', () => {
  it('needs a name', () => {
    expect(columnDraftError({ ...EMPTY_COLUMN_DRAFT, name: '  ' }, [])).toBe('Angiv et navn.');
  });

  it('refuses a name a key already uses, ignoring case and spaces', () => {
    expect(columnDraftError({ ...EMPTY_COLUMN_DRAFT, name: ' stand ' }, ['Stand'])).toBe(
      'Der findes allerede et felt med det navn.',
    );
  });

  it('needs an option for a select list', () => {
    expect(columnDraftError({ name: 'Stand', kind: 'select', optionsText: ' , ' }, [])).toBe(
      'Angiv mindst én valgmulighed.',
    );
    expect(columnDraftError({ name: 'Stand', kind: 'select', optionsText: 'God' }, [])).toBeNull();
  });
});

describe('columnCreateBody', () => {
  it('sends options only for a select list', () => {
    expect(columnCreateBody({ name: ' Stand ', kind: 'select', optionsText: 'God,Dårlig' })).toEqual({
      name: 'Stand',
      type: 'select',
      options: ['God', 'Dårlig'],
    });
    expect(columnCreateBody({ name: 'Leverandør', kind: 'text', optionsText: 'ignored' })).toEqual({
      name: 'Leverandør',
      type: 'text',
    });
  });

  it('labels every kind in Danish', () => {
    expect(COLUMN_KIND_LABEL).toEqual({ text: 'Tekst', number: 'Tal', date: 'Dato', boolean: 'Ja/nej', select: 'Valgliste' });
  });
});

describe('seed templates', () => {
  it('warns that the column goes into a seed, and only then offers a copy', () => {
    expect(seedNote(template({ name: 'Hurtig genbrugsscreening', seed: 'screening' }))).toBe(
      'Kolonnen føjes til standardskabelonen »Hurtig genbrugsscreening«.',
    );
    expect(seedNote(template({ seed: null }))).toBeNull();
    expect(seedNote(null)).toBeNull();
    expect(duplicateFirst(template({ seed: 'screening' }), true)).toBe(true);
    expect(duplicateFirst(template({ seed: 'screening' }), false)).toBe(false);
    expect(duplicateFirst(template({ seed: null }), true)).toBe(false);
  });

  it('says the column exists when only the template update failed', () => {
    expect(columnPartialFailureMessage('Stand', 'busy')).toBe(
      'Kolonnen »Stand« er oprettet, men kunne ikke føjes til skabelonen: busy',
    );
  });
});

describe('columnCreateConflict', () => {
  it('shows a 409 in the dialog with the server reason, not the pipeline-job copy', () => {
    const cause = new ApiRequestError(409, "a column named 'Stand' already exists", '/api/v1/resources/columns');
    expect(columnCreateConflict(cause)).toBe("Navnet kan ikke bruges: a column named 'Stand' already exists");
  });

  it('leaves every other failure to the toast', () => {
    expect(columnCreateConflict(new ApiRequestError(400, 'bad', '/x'))).toBeNull();
    expect(columnCreateConflict(new ApiRequestError(503, 'locked', '/x'))).toBeNull();
    expect(columnCreateConflict(new Error('network'))).toBeNull();
  });
});
