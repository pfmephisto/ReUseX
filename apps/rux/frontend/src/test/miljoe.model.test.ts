// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  addSample,
  advancePatch,
  answeredNote,
  cardAction,
  chainSteps,
  createBody,
  deleteConfirmText,
  editorKeyAction,
  gateChanges,
  gateMessage,
  linkableTypes,
  linkedTypes,
  nextStage,
  removeSample,
  replaceSample,
  resultPatch,
  resultToast,
  statusPill,
  textCommit,
  toggleLink,
  UNDO_RESULT_PATCH,
} from '../miljoe/model';
import { sample, surveyType } from './surveyFixtures';

describe('stage chain', () => {
  it('marks steps before the stage done, the stage current, the rest todo', () => {
    expect(chainSteps({ stage: 'sendt' }).map((s) => [s.label, s.state])).toEqual([
      ['Planlagt', 'done'],
      ['Udtaget', 'done'],
      ['Sendt til lab', 'current'],
      ['Svar modtaget', 'todo'],
    ]);
    expect(chainSteps({ stage: 'svar' }).map((s) => s.state)).toEqual(['done', 'done', 'done', 'current']);
    expect(chainSteps({ stage: 'planlagt' }).map((s) => s.state)).toEqual(['current', 'todo', 'todo', 'todo']);
  });

  it('advances one stage at a time and stops at svar', () => {
    expect(nextStage('planlagt')).toBe('udtaget');
    expect(nextStage('udtaget')).toBe('sendt');
    expect(nextStage('svar')).toBeNull();
    expect(advancePatch({ stage: 'udtaget' })).toEqual({ stage: 'sendt' });
    expect(advancePatch({ stage: 'svar' })).toBeNull();
  });
});

describe('card status and action', () => {
  it('pills the stage until a result exists, then the result', () => {
    expect(statusPill({ stage: 'sendt', result: null })).toEqual({ label: 'Sendt til lab', tone: 'wait' });
    expect(statusPill({ stage: 'udtaget', result: null })).toEqual({ label: 'Udtaget', tone: 'wait' });
    expect(statusPill({ stage: 'svar', result: 'forurenet' })).toEqual({ label: 'Forurenet', tone: 'crit' });
    expect(statusPill({ stage: 'svar', result: 'ren' })).toEqual({ label: 'Ren', tone: 'good' });
    expect(statusPill({ stage: 'svar', result: null })).toEqual({ label: 'Svar modtaget', tone: 'accent' });
  });

  it('offers Næste trin before the lab, the result buttons once sent, a note after', () => {
    expect(cardAction({ stage: 'planlagt', result: null })).toBe('advance');
    expect(cardAction({ stage: 'udtaget', result: null })).toBe('advance');
    expect(cardAction({ stage: 'sendt', result: null })).toBe('answer');
    expect(cardAction({ stage: 'svar', result: null })).toBe('answer');
    expect(cardAction({ stage: 'svar', result: 'ren' })).toBe('answered');
  });

  it('records a result together with the svar stage, and undoes back to sendt', () => {
    expect(resultPatch('ren')).toEqual({ stage: 'svar', result: 'ren' });
    expect(resultPatch('forurenet')).toEqual({ stage: 'svar', result: 'forurenet' });
    expect(UNDO_RESULT_PATCH).toEqual({ stage: 'sendt', result: null });
  });

  it('words the answered note like the prototype', () => {
    expect(answeredNote({ type_ids: [11] })).toBe(
      'Svar registreret — miljøstatus opdateret på 1 type(r) i kortlægningen.',
    );
    expect(answeredNote({ type_ids: [] })).toBe('Svar registreret — prøven er ikke koblet til nogen type.');
  });
});

describe('links', () => {
  const types = [
    surveyType({ id: 6, name: 'Vinduespartier, aluminium' }),
    surveyType({ id: 8, name: 'Gulvbelægning, linoleum' }),
    surveyType({ id: 4, name: 'Fejldetektion', review_status: 'rejected' }),
  ];

  it('resolves linked ids to types in link order, skipping unknown ones', () => {
    expect(linkedTypes([8, 6, 99], types).map((t) => t.id)).toEqual([8, 6]);
  });

  it('offers non-rejected types by name, plus rejected ones already linked', () => {
    expect(linkableTypes(types, []).map((t) => t.id)).toEqual([8, 6]);
    expect(linkableTypes(types, [4]).map((t) => t.id)).toEqual([4, 8, 6]);
  });

  it('toggles a link and keeps the set sorted', () => {
    expect(toggleLink([8], 6)).toEqual([6, 8]);
    expect(toggleLink([6, 8], 6)).toEqual([8]);
    expect(toggleLink([], 3)).toEqual([3]);
  });
});

describe('sample lists', () => {
  const list = [sample({ id: 1, stage: 'sendt' }), sample({ id: 2, code: 'P-02', stage: 'svar', result: 'forurenet' })];

  it('replaces, adds in id order and removes', () => {
    expect(replaceSample(list, sample({ id: 1, stage: 'svar', result: 'ren' }))[0].result).toBe('ren');
    expect(addSample(list, sample({ id: 0, code: 'P-00' })).map((s) => s.id)).toEqual([0, 1, 2]);
    expect(addSample(list, sample({ id: 2, title: 'ny' })).find((s) => s.id === 2)?.title).toBe('ny');
    expect(removeSample(list, 1).map((s) => s.id)).toEqual([2]);
  });
});

describe('drafts and the create body', () => {
  it('commits only a real change, trimmed; a required field never empties', () => {
    expect(textCommit('PCB i fugemasse', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('  PCB i fugemasse ', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('PCB i fuger', 'PCB i fugemasse', true)).toBe('PCB i fuger');
    expect(textCommit('   ', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('', 'Fugemasse', false)).toBe('');
  });

  it('builds a POST /samples body, or null without a title', () => {
    expect(createBody(' Bly i maling ', ' Indervægge ', [11, 2])).toEqual({
      title: 'Bly i maling',
      what: 'Indervægge',
      type_ids: [2, 11],
    });
    expect(createBody('  ', 'x', [])).toBeNull();
  });
});

describe('gate feedback', () => {
  const before = [
    surveyType({ id: 6, name: 'Vinduespartier, aluminium', environment_status: 'afventer' }),
    surveyType({ id: 11, name: 'Indvendige murvægge, malet', environment_status: 'afventer' }),
    surveyType({ id: 7, name: 'Trapezplader, tag', review_status: 'approved', environment_status: 'ren_screening' }),
    surveyType({ id: 2, name: 'Betonsøjler, bærende', environment_status: 'ren_screening' }),
    surveyType({ id: 4, name: 'Fejldetektion', review_status: 'rejected', environment_status: 'afventer' }),
  ];

  it('sorts status changes into unblocked, contaminated, blocked and re-blocked', () => {
    const after = [
      { ...before[0], environment_status: 'ren_proevesvar' as const },
      { ...before[1], environment_status: 'forurenet' as const },
      { ...before[2], environment_status: 'afventer' as const },
      { ...before[3], environment_status: 'afventer' as const },
      { ...before[4], environment_status: 'ren_screening' as const },
    ];
    const c = gateChanges(before, after);
    expect(c.unblocked.map((t) => t.id)).toEqual([6]);
    expect(c.contaminated.map((t) => t.id)).toEqual([11]);
    expect(c.reblocked.map((t) => t.id)).toEqual([7]);
    expect(c.blocked.map((t) => t.id)).toEqual([2]);
  });

  it('words the toast, or says nothing when nothing changed', () => {
    const unblocked = gateChanges(before, [{ ...before[0], environment_status: 'ren_proevesvar' }]);
    expect(gateMessage('P-01', unblocked)).toBe('P-01: 1 type kan nu godkendes (Vinduespartier, aluminium)');
    const mixed = gateChanges(before, [
      { ...before[1], environment_status: 'forurenet' },
      { ...before[2], environment_status: 'afventer' },
      { ...before[3], environment_status: 'afventer' },
    ]);
    expect(gateMessage('P-02', mixed)).toBe(
      'P-02: Indvendige murvægge, malet er nu forurenet · Betonsøjler, bærende afventer nu prøvesvar · ' +
        'Trapezplader, tag er godkendt, men afventer nu prøvesvar',
    );
    expect(gateMessage('P-03', gateChanges(before, before))).toBeNull();
    const two = gateChanges(before, [
      { ...before[0], environment_status: 'ren_proevesvar' },
      { ...before[1], environment_status: 'ren_screening' },
    ]);
    expect(gateMessage('P-04', two)).toBe(
      'P-04: 2 typer kan nu godkendes (Vinduespartier, aluminium, Indvendige murvægge, malet)',
    );
  });

  it('has fallback toasts for a result and a delete prompt', () => {
    expect(resultToast('P-01', 'ren')).toBe('✓ P-01 · svar registreret: Ren');
    expect(deleteConfirmText({ code: 'P-01', title: 'PCB i fugemasse', type_ids: [6] })).toBe(
      'Slet P-01 · PCB i fugemasse? 1 koblet type mister prøven, og dens miljøstatus beregnes igen.',
    );
    expect(deleteConfirmText({ code: 'P-05', title: 'PAH i tagpap', type_ids: [] })).toBe('Slet P-05 · PAH i tagpap?');
  });
});

describe('editor keys', () => {
  it('reverts a text field on Esc and closes the editor from anywhere else', () => {
    expect(editorKeyAction({ key: 'Escape', kind: 'text' })).toBe('revert');
    expect(editorKeyAction({ key: 'Escape', kind: 'choice' })).toBe('close');
    expect(editorKeyAction({ key: 'Escape', kind: 'control' })).toBe('close');
  });

  it('commits a text field on Enter, submits on Ctrl/⌘+Enter, leaves controls alone', () => {
    expect(editorKeyAction({ key: 'Enter', kind: 'text' })).toBe('commit');
    expect(editorKeyAction({ key: 'Enter', kind: 'text', ctrlKey: true })).toBe('submit');
    expect(editorKeyAction({ key: 'Enter', kind: 'control', metaKey: true })).toBe('submit');
    expect(editorKeyAction({ key: 'Enter', kind: 'control' })).toBeNull();
    expect(editorKeyAction({ key: ' ', kind: 'choice' })).toBeNull();
    expect(editorKeyAction({ key: 'g', kind: 'other' })).toBeNull();
  });
});
