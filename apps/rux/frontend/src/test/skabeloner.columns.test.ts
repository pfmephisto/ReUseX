// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  columnDeleteConfirm,
  columnErrorMessage,
  columnKindLabel,
  hasOptions,
  optionsChanged,
  optionsError,
  optionsText,
  renameError,
} from '../skabeloner/columns';

const err = (status: number, message: string) => new ApiRequestError(status, message, '/api/v1/resources/columns/c1');
const labels = ['Farve', 'Tilstand', 'Antal'];

describe('renameError', () => {
  it('allows keeping the same name', () => {
    expect(renameError('Farve', 'Farve', labels)).toBeNull();
  });
  it('allows a case-only rename of the column itself', () => {
    expect(renameError('farve', 'Farve', labels)).toBeNull();
    expect(renameError('  FARVE ', 'Farve', labels)).toBeNull();
  });
  it('refuses a name another field already has', () => {
    expect(renameError('tilstand', 'Farve', labels)).toBe('Der findes allerede et felt med det navn.');
  });
  it('refuses an empty name', () => {
    expect(renameError('   ', 'Farve', labels)).toBe('Angiv et navn.');
  });
  it('accepts a new name', () => {
    expect(renameError('Kulør', 'Farve', labels)).toBeNull();
  });
});

describe('options', () => {
  it('is null when nothing changed', () => {
    expect(optionsChanged('Rød\nGrøn', ['Rød', 'Grøn'])).toBeNull();
    expect(optionsChanged('Rød, Grøn,', ['Rød', 'Grøn'])).toBeNull();
  });
  it('returns the parsed list when it changed', () => {
    expect(optionsChanged('Rød\nGrøn\nBlå', ['Rød', 'Grøn'])).toEqual(['Rød', 'Grøn', 'Blå']);
  });
  it('counts a reorder as a change', () => {
    expect(optionsChanged('Grøn\nRød', ['Rød', 'Grøn'])).toEqual(['Grøn', 'Rød']);
  });
  it('shows options one per line', () => {
    expect(optionsText(['Rød', 'Grøn'])).toBe('Rød\nGrøn');
    expect(optionsText(undefined)).toBe('');
  });
  it('requires at least one option', () => {
    expect(optionsError(' , \n')).toBe('Angiv mindst én valgmulighed.');
    expect(optionsError('Rød')).toBeNull();
  });
});

describe('copy', () => {
  it('says the column and its stored values are deleted', () => {
    const text = columnDeleteConfirm('Farve');
    expect(text).toContain('Farve');
    expect(text).toMatch(/gemte værdier/);
  });
  it('labels column kinds in Danish, falling back to the raw type', () => {
    expect(columnKindLabel('select')).toBe('Valgliste');
    expect(columnKindLabel('text')).toBe('Tekst');
    expect(columnKindLabel('multiselect')).toBe('Flervalg');
    expect(columnKindLabel('url')).toBe('Link');
    expect(columnKindLabel('weird')).toBe('weird');
  });
  it('edits options for choice columns only', () => {
    expect(hasOptions('select')).toBe(true);
    expect(hasOptions('multiselect')).toBe(true);
    expect(hasOptions('text')).toBe(false);
  });
});

describe('columnErrorMessage', () => {
  it('maps a name-conflict 409 to Danish', () => {
    expect(columnErrorMessage(err(409, "a column named 'Farve' already exists"))).toBe(
      'Der findes allerede en kolonne med det navn.',
    );
    expect(columnErrorMessage(err(409, "'Mængde' is a leksikon field name; choose another column name"))).toBe(
      'Navnet bruges af et felt i materialepasset.',
    );
  });
  it('leaves a pipeline-busy 409 to saveErrorMessage', () => {
    expect(columnErrorMessage(err(409, 'a pipeline job is running; try again when it finishes'))).toBe(
      'Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.',
    );
  });
  it('falls back to the server message for other errors', () => {
    expect(columnErrorMessage(err(404, "no column 'c9'"))).toBe("Kunne ikke gemme: no column 'c9'");
  });
});
