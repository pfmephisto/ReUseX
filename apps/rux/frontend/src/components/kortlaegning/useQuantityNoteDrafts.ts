// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The mængde and note drafts that DetailPanel and EditDialog both edit, in one
 * place: the draft state, the selection-keyed reset, the re-sync from the
 * server while the quantity field is not focused, and the two blur commits.
 *
 * The quantity draft is seeded with `formatQuantityInput` (lossless), never
 * the one-decimal display format, so focusing and leaving the field without
 * typing sends nothing — see `quantityCommitValue`.
 */

import { useEffect, useRef, useState } from 'react';

import type { SurveyPart, SurveyType } from '../../api/types';
import { formatQuantityInput, parseDanishNumber } from '../../kortlaegning/vocab';

/**
 * Parses a quantity draft against the value currently on the server. Returns
 * the number to send, or `null` when nothing should be sent: the text is the
 * current value's own input format (the field was focused and left as it
 * was), it doesn't parse, or it parses to the unchanged value. On `null` the
 * caller reverts the draft to `formatQuantityInput(current)`.
 */
export function quantityCommitValue(draftText: string, current: number): number | null {
  if (draftText.trim() === formatQuantityInput(current)) return null;
  const parsed = parseDanishNumber(draftText);
  if (parsed === null || parsed === current) return null;
  return parsed;
}

/** What the drafts are reset on: the selected part, or the type itself. */
export function selectionKey(current: SurveyType | SurveyPart | null): string {
  if (!current) return '';
  return 'code' in current ? `p:${current.code}` : `t:${current.id}`;
}

/**
 * Whether a blur may commit: only when the drafts were last reset for the
 * selection being edited now. Between a selection change and the render that
 * resets the drafts, a blur would otherwise send the old row's text to the
 * new row.
 */
export function draftMatchesSelection(draftKey: string, currentKey: string): boolean {
  return draftKey === currentKey;
}

export interface DraftCommits {
  /** type → redistributes across its parts; part → that part only. */
  onQuantity: (q: number) => void;
  onNote: (note: string) => void;
}

/**
 * Drafts for the entity being edited (`current`: the selected part, else the
 * type). Spread `quantityProps` on the quantity `<input>` and `noteProps` on
 * the note `<textarea>`; both commit on blur, so Enter commits by blurring;
 * `revertQuantity` / `revertNote` are Esc, which drops the draft and blurs
 * without committing.
 */
export function useQuantityNoteDrafts(current: SurveyType | SurveyPart | null, commits: DraftCommits) {
  const [quantityDraft, setQuantityDraft] = useState(() => (current ? formatQuantityInput(current.quantity) : ''));
  const [noteDraft, setNoteDraft] = useState(() => current?.note ?? '');
  // Whether the quantity input is focused, so the re-sync below never
  // clobbers what the user is mid-typing.
  const quantityFocusedRef = useRef(false);
  // Set by a revert so the blur it triggers commits nothing (Esc, R10).
  const skipQuantityCommit = useRef(false);
  const skipNoteCommit = useRef(false);
  const key = selectionKey(current);
  // The selection the drafts were last reset for. State, not a ref: a blur
  // handler sees the value of the render it came from, so in the render where
  // `current` has already moved but the drafts have not been reset yet, this
  // still names the old selection and `draftMatchesSelection` refuses.
  const [draftKey, setDraftKey] = useState(key);

  // A new selection starts from its own values.
  useEffect(() => {
    setQuantityDraft(current ? formatQuantityInput(current.quantity) : '');
    setNoteDraft(current?.note ?? '');
    setDraftKey(key);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key]);

  // The server value changed under us (a redistribution from editing the
  // type's aggregate changes this part's share): follow it unless focused.
  useEffect(() => {
    if (current && !quantityFocusedRef.current) setQuantityDraft(formatQuantityInput(current.quantity));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [current?.quantity]);

  function commitQuantity() {
    if (!current || !draftMatchesSelection(draftKey, key)) return;
    const value = quantityCommitValue(quantityDraft, current.quantity);
    if (value !== null) commits.onQuantity(value);
    else setQuantityDraft(formatQuantityInput(current.quantity));
  }

  function commitNote() {
    if (!current || !draftMatchesSelection(draftKey, key)) return;
    if (noteDraft !== current.note) commits.onNote(noteDraft);
  }

  return {
    quantityProps: {
      value: quantityDraft,
      onChange: (e: { target: { value: string } }) => setQuantityDraft(e.target.value),
      onFocus: () => {
        quantityFocusedRef.current = true;
      },
      onBlur: () => {
        quantityFocusedRef.current = false;
        if (skipQuantityCommit.current) {
          skipQuantityCommit.current = false;
          return;
        }
        commitQuantity();
      },
    },
    noteProps: {
      value: noteDraft,
      onChange: (e: { target: { value: string } }) => setNoteDraft(e.target.value),
      onBlur: () => {
        if (skipNoteCommit.current) {
          skipNoteCommit.current = false;
          return;
        }
        commitNote();
      },
    },
    /** Esc: drop the quantity draft and leave the field without committing. */
    revertQuantity: (el: HTMLElement) => {
      skipQuantityCommit.current = true;
      if (current) setQuantityDraft(formatQuantityInput(current.quantity));
      el.blur();
    },
    /** Esc: drop the note draft and leave the field without committing. */
    revertNote: (el: HTMLElement) => {
      skipNoteCommit.current = true;
      setNoteDraft(current?.note ?? '');
      el.blur();
    },
  };
}
