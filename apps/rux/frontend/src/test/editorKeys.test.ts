// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { editorKeyAction, fieldKeyAction, formKeyDown } from '../app/editorKeys';
import { editorKeyAction as miljoeEditorKeyAction, textCommit as miljoeTextCommit } from '../miljoe/model';
import { draftCommit, textCommit, textDraftCommit } from '../app/textDraft';

describe('field keys (R10)', () => {
  it('reverts on Esc and commits on Enter in a single-line field', () => {
    expect(fieldKeyAction({ key: 'Escape' })).toBe('revert');
    expect(fieldKeyAction({ key: 'Enter' })).toBe('commit');
    expect(fieldKeyAction({ key: 'Enter', altKey: true })).toBeNull();
  });

  it('leaves Enter to a multi-line field and Ctrl/⌘+Enter to the editor', () => {
    expect(fieldKeyAction({ key: 'Enter' }, true)).toBeNull();
    expect(fieldKeyAction({ key: 'Escape' }, true)).toBe('revert');
    expect(fieldKeyAction({ key: 'Enter', ctrlKey: true })).toBeNull();
    expect(fieldKeyAction({ key: 'Enter', metaKey: true }, true)).toBeNull();
    expect(fieldKeyAction({ key: 'a' })).toBeNull();
  });

  it('is the one editorKeyAction Miljø uses', () => {
    expect(miljoeEditorKeyAction).toBe(editorKeyAction);
    expect(miljoeTextCommit).toBe(textCommit);
  });
});

describe('validated drafts', () => {
  const int = (d: string) => (/^\d+$/.test(d.trim()) ? { value: Number(d.trim()) } : d.trim() === '' ? { value: null } : null);

  it('sends nothing for an untouched blur, even when the stored text would not parse', () => {
    expect(draftCommit('x', 'x', int)).toEqual({ send: false, invalid: false });
    expect(draftCommit(' 12 ', '12', int)).toEqual({ send: false, invalid: false });
  });

  it('sends nothing when the draft parses to the stored value', () => {
    expect(draftCommit('012', '12', int)).toEqual({ send: false, invalid: false });
  });

  it('flags an invalid draft and sends a changed valid one', () => {
    expect(draftCommit('1x', '12', int)).toEqual({ send: false, invalid: true });
    expect(draftCommit('13', '12', int)).toEqual({ send: true, value: 13 });
    expect(draftCommit('', '12', int)).toEqual({ send: true, value: null });
  });
});

describe('plain text drafts', () => {
  it('flags an emptied required field so the caller can say why', () => {
    expect(textDraftCommit('  ', 'Måløv', true)).toEqual({ send: false, invalid: true });
    expect(textDraftCommit('', '', true)).toEqual({ send: false, invalid: false });
    expect(textDraftCommit('Måløv ', 'Måløv', true)).toEqual({ send: false, invalid: false });
    expect(textDraftCommit('Ballerup', 'Måløv', true)).toEqual({ send: true, value: 'Ballerup' });
  });

  it('clears an optional field without flagging it', () => {
    expect(textDraftCommit('', 'Måløv', false)).toEqual({ send: true, value: '' });
  });
});

describe('formKeyDown (a create form whose fields are not saved yet)', () => {
  function press(key: string, mods: { ctrlKey?: boolean; metaKey?: boolean } = {}) {
    const calls: string[] = [];
    const e = {
      key,
      target: { tagName: 'INPUT', type: 'text' } as unknown as EventTarget,
      ctrlKey: mods.ctrlKey ?? false,
      metaKey: mods.metaKey ?? false,
      altKey: false,
      preventDefault: () => calls.push('prevent'),
      stopPropagation: () => calls.push('stop'),
    };
    formKeyDown(e, { onCancel: () => calls.push('cancel'), onSubmit: () => calls.push('submit') });
    return calls;
  }

  it('cancels on Esc, even in a text field, and stops it there', () => {
    expect(press('Escape')).toEqual(['prevent', 'stop', 'cancel']);
  });

  it('submits on Ctrl/⌘+Enter', () => {
    expect(press('Enter', { ctrlKey: true })).toEqual(['prevent', 'stop', 'submit']);
    expect(press('Enter', { metaKey: true })).toEqual(['prevent', 'stop', 'submit']);
  });

  it('leaves plain Enter to the native submit', () => {
    expect(press('Enter')).toEqual([]);
  });
});
