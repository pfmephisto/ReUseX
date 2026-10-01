// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState, type ChangeEvent } from 'react';

import { textCommit } from './model';

export interface TextDraft {
  props: {
    value: string;
    onChange: (e: ChangeEvent<HTMLInputElement>) => void;
    onFocus: () => void;
    onBlur: () => void;
  };
  /** Esc: drop the draft and leave the field without committing. */
  revert: (el: HTMLElement) => void;
}

/**
 * A text field that commits on blur. An untouched blur sends nothing
 * (`textCommit`); a required field that was emptied snaps back. While the
 * field is not focused it follows the server value, so a response that
 * changed it shows up; while focused it never clobbers what is being typed.
 *
 * If the commit fails, the caller shows an error toast and this hook does
 * nothing extra: the unsaved draft simply stays in the field (it was never
 * reset, because `current` on the server did not change), and the next blur
 * retries the same commit (D14, accepted for v1).
 */
export function useTextDraft(current: string, onCommit: (value: string) => void, required = false): TextDraft {
  const [draft, setDraft] = useState(current);
  const focused = useRef(false);
  const skipNextCommit = useRef(false);

  useEffect(() => {
    if (!focused.current) setDraft(current);
  }, [current]);

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
        const value = textCommit(draft, current, required);
        if (value === null) setDraft(current);
        else onCommit(value);
      },
    },
    revert: (el) => {
      skipNextCommit.current = true;
      setDraft(current);
      el.blur();
    },
  };
}
