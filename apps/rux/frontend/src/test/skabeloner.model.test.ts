// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  countLine,
  createLatestGate,
  deleteConfirmText,
  isNameConflict,
  missingSeeds,
  nextTemplateName,
  replaceTemplate,
  restoreSeedsTitle,
  seedLabel,
  selectAfterDelete,
  templateErrorMessage,
  withMembers,
} from '../skabeloner/model';

const err = (status: number, message: string) => new ApiRequestError(status, message, '/api/v1/templates/1');

describe('copy', () => {
  it('counts fields in Danish', () => {
    expect(countLine(0)).toBe('0 felter');
    expect(countLine(1)).toBe('1 felt');
    expect(countLine(42)).toBe('42 felter');
  });

  it('marks seeds', () => {
    expect(seedLabel('screening')).toBe('Standard');
    expect(seedLabel(null)).toBeNull();
  });

  it('asks before deleting, and says how a seed comes back', () => {
    expect(deleteConfirmText({ name: 'Min skabelon', seed: null })).toBe(
      'Slet skabelonen "Min skabelon"? Det kan ikke fortrydes.',
    );
    expect(deleteConfirmText({ name: 'Hurtig genbrugsscreening', seed: 'screening' })).toBe(
      'Slet skabelonen "Hurtig genbrugsscreening"? Den kan hentes tilbage med "Gendan standardskabeloner".',
    );
  });
});

describe('names', () => {
  it('finds the first free "Ny skabelon"', () => {
    expect(nextTemplateName([])).toBe('Ny skabelon');
    expect(nextTemplateName(['Ny skabelon'])).toBe('Ny skabelon 2');
    expect(nextTemplateName(['Ny skabelon', 'Ny skabelon 2', 'Ny skabelon 4'])).toBe('Ny skabelon 3');
  });
});

describe('seeds', () => {
  it('lists the seed tags no row carries', () => {
    expect(missingSeeds([{ seed: 'screening' }, { seed: null }])).toEqual(['materialepas']);
    expect(missingSeeds([{ seed: 'screening' }, { seed: 'materialepas' }])).toEqual([]);
  });

  it('titles the restore button', () => {
    expect(restoreSeedsTitle([])).toBe('Begge standardskabeloner findes i projektet.');
    expect(restoreSeedsTitle(['materialepas'])).toBe('Genopretter Materialepas (fuld).');
    expect(restoreSeedsTitle(['materialepas', 'screening'])).toBe(
      'Genopretter Materialepas (fuld) og Hurtig genbrugsscreening.',
    );
  });
});

describe('errors (R5)', () => {
  it('reads a name clash as a name clash', () => {
    const e = err(409, 'template name already exists');
    expect(isNameConflict(e)).toBe(true);
    expect(templateErrorMessage(e)).toBe('Der findes allerede en skabelon med det navn.');
  });

  it('reads the pipeline lock as the lock, not as a name clash', () => {
    const e = err(409, 'a pipeline job is running or queued; edits are refused while a stage is writing the project');
    expect(isNameConflict(e)).toBe(false);
    expect(templateErrorMessage(e)).toBe('Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.');
  });

  it('explains a vanished template and a bad member', () => {
    expect(templateErrorMessage(err(404, 'no such template'))).toBe(
      'Skabelonen findes ikke længere — listen er hentet igen.',
    );
    expect(templateErrorMessage(err(400, 'unknown key id col:99'))).toBe('Ugyldig skabelon: unknown key id col:99');
  });
});

describe('selection after delete (R7)', () => {
  it('picks the next, else the previous, else none', () => {
    expect(selectAfterDelete([1, 2, 3], 2)).toBe(3);
    expect(selectAfterDelete([1, 2, 3], 3)).toBe(2);
    expect(selectAfterDelete([5], 5)).toBeNull();
    expect(selectAfterDelete([1, 2], 9)).toBe(1);
  });
});

describe('list state', () => {
  const list = [
    { id: 1, name: 'A', members: [] },
    { id: 2, name: 'B', members: [{ key: 'sys:name' }] },
  ];

  it('replaces one template by id and keeps order', () => {
    expect(replaceTemplate(list, { id: 2, name: 'B2', members: [] }).map((t) => t.name)).toEqual(['A', 'B2']);
    expect(replaceTemplate(list, { id: 9, name: 'X', members: [] })).toEqual(list);
  });

  it('sets members on one template', () => {
    expect(withMembers(list, 1, [{ category: 'Brand' }])[0].members).toEqual([{ category: 'Brand' }]);
    expect(list[0].members).toEqual([]);
  });
});

describe('createLatestGate (R3)', () => {
  it('lets only the newest edit per template apply its response', () => {
    const gate = createLatestGate();
    const a = gate.next(1);
    const b = gate.next(1);
    const other = gate.next(2);
    expect(gate.isLatest(1, a)).toBe(false);
    expect(gate.isLatest(1, b)).toBe(true);
    expect(gate.isLatest(2, other)).toBe(true);
  });
});
