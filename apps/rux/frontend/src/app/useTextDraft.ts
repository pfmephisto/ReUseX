// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState, type ChangeEvent, type KeyboardEvent, type RefObject } from 'react';

import { fieldKeyAction } from './editorKeys';
import { decideDraft, draftResets, leaveCommit, type DraftCommit, type DraftValidate } from './textDraft';

export interface TextDraft {
  props: {
    value: string;
    onChange: (e: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => void;
    onFocus: () => void;
    onBlur: () => void;
  };
  /** Esc: drop the draft and leave the field without committing. */
  revert: (el: HTMLElement) => void;
}

/** A field whose draft must parse before it is sent (e.g. a year). */
export interface DraftOptions<T> {
  validate: DraftValidate<T>;
  /** Called when a blur finds the draft invalid; the draft then snaps back. */
  onInvalid?: () => void;
  /** Bump to drop the draft after a refused commit (`draftResets`). */
  reset?: number;
}

/** A plain text field; `onInvalid` fires when a required field was emptied. */
export interface TextOptions {
  required?: boolean;
  onInvalid?: () => void;
  /** Bump to drop the draft after a refused commit (`draftResets`). */
  reset?: number;
}

/**
 * A text field that commits on blur. An untouched blur sends nothing
 * (`textCommit` / `draftCommit`); a required field that was emptied, or a
 * validated field whose draft does not parse, snaps back. While the field is
 * not focused it follows the server value, so a response that changed it
 * shows up; while focused it never clobbers what is being typed.
 *
 * The third argument is `required` or `TextOptions` (plain text), or
 * `DraftOptions` (a validated field that commits the parsed value).
 *
 * A field that unmounts while focused (no blur ran) commits its dirty draft
 * on the way out (`leaveCommit`), so a field commit is never dropped.
 *
 * If the commit fails, the caller shows an error toast and this hook does
 * nothing extra: the unsaved draft simply stays in the field (it was never
 * reset, because `current` on the server did not change), and the next blur
 * retries the same commit (D14, accepted for v1) — unless the caller bumps
 * `reset`, which snaps an unfocused field back to `current`.
 */
export function useTextDraft(
  current: string,
  onCommit: (value: string) => void,
  options?: boolean | TextOptions,
): TextDraft;
export function useTextDraft<T>(current: string, onCommit: (value: T) => void, options: DraftOptions<T>): TextDraft;
export function useTextDraft<T>(
  current: string,
  onCommit: (value: T) => void,
  options: boolean | TextOptions | DraftOptions<T> = false,
): TextDraft {
  const [draft, setDraft] = useState(current);
  const focused = useRef(false);
  const skipNextCommit = useRef(false);

  useEffect(() => {
    if (!focused.current) setDraft(current);
  }, [current]);

  // A refused commit: show the server value again, unless the user is typing (draftResets).
  const resetToken = typeof options === 'object' ? (options.reset ?? 0) : 0;
  const seenReset = useRef(resetToken);
  useEffect(() => {
    if (draftResets(seenReset.current, resetToken, focused.current)) setDraft(current);
    seenReset.current = resetToken;
  }, [resetToken]);

  function decide(): DraftCommit<T> {
    return decideDraft<T>(draft, current, options);
  }

  // The latest state, for the unmount commit below (its closure is the first render's).
  const latest = useRef({ draft, current, options, onCommit });
  latest.current = { draft, current, options, onCommit };
  useEffect(
    () => () => {
      const l = latest.current;
      const c = leaveCommit<T>(
        { focused: focused.current, reverted: skipNextCommit.current, draft: l.draft, current: l.current },
        l.options,
      );
      if (c.send) l.onCommit(c.value);
    },
    [],
  );

  return {
    props: {
      value: draft,
      onChange: (e) => setDraft(e.target.value),
      onFocus: () => {
        focused.current = true;
      },
      onBlur: () => {
        focused.current = false;
        if (skipNextCommit.current) {
          skipNextCommit.current = false;
          return;
        }
        const c = decide();
        if (c.send) {
          onCommit(c.value);
          return;
        }
        if (c.invalid && typeof options === 'object') options.onInvalid?.();
        setDraft(current);
      },
    },
    revert: (el) => {
      skipNextCommit.current = true;
      setDraft(current);
      el.blur();
    },
  };
}

/**
 * A committing field's own keys (`fieldKeyAction`): Enter commits by leaving
 * the field (single-line only), Esc reverts it without committing and parks
 * focus on the editor itself (`home`, a `tabIndex={-1}` container), so focus
 * never drops to <body> and a second Esc closes the editor. Anything else,
 * Ctrl/⌘+Enter included, bubbles to the editor.
 */
export function fieldKeys(draft: TextDraft, home: RefObject<HTMLElement | null>, multiline = false) {
  return (e: KeyboardEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const action = fieldKeyAction(
      { key: e.key, ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey },
      multiline,
    );
    if (action === 'revert') {
      e.preventDefault();
      e.stopPropagation(); // the editor's Esc would otherwise close it
      draft.revert(e.currentTarget);
      home.current?.focus();
    } else if (action === 'commit') {
      e.preventDefault();
      e.currentTarget.blur();
    }
  };
}
