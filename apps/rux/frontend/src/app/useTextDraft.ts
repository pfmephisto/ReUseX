// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState, type ChangeEvent, type KeyboardEvent, type RefObject } from 'react';

import { fieldKeyAction } from './editorKeys';
import { draftCommit, textCommit, type DraftCommit, type DraftValidate } from './textDraft';

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
}

/**
 * A text field that commits on blur. An untouched blur sends nothing
 * (`textCommit` / `draftCommit`); a required field that was emptied, or a
 * validated field whose draft does not parse, snaps back. While the field is
 * not focused it follows the server value, so a response that changed it
 * shows up; while focused it never clobbers what is being typed.
 *
 * The third argument is either `required` (plain text) or `DraftOptions`
 * (a validated field that commits the parsed value).
 *
 * If the commit fails, the caller shows an error toast and this hook does
 * nothing extra: the unsaved draft simply stays in the field (it was never
 * reset, because `current` on the server did not change), and the next blur
 * retries the same commit (D14, accepted for v1).
 */
export function useTextDraft(current: string, onCommit: (value: string) => void, required?: boolean): TextDraft;
export function useTextDraft<T>(current: string, onCommit: (value: T) => void, options: DraftOptions<T>): TextDraft;
export function useTextDraft<T>(
  current: string,
  onCommit: (value: T) => void,
  options: boolean | DraftOptions<T> = false,
): TextDraft {
  const [draft, setDraft] = useState(current);
  const focused = useRef(false);
  const skipNextCommit = useRef(false);

  useEffect(() => {
    if (!focused.current) setDraft(current);
  }, [current]);

  function decide(): DraftCommit<T> {
    if (typeof options === 'object') return draftCommit(draft, current, options.validate);
    const value = textCommit(draft, current, options);
    // Plain text: T is string (the overloads guarantee it).
    return value === null ? { send: false, invalid: false } : { send: true, value: value as T };
  }

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
    const action = fieldKeyAction(e, multiline);
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
