// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { disarms } from './armedConfirm';

export interface ArmedConfirm<K> {
  /** The armed item, or null. */
  armed: K | null;
  /** Arm `key`; `button` is the armed button, so a press on it is not "outside". */
  arm: (key: K, button: HTMLElement) => void;
  disarm: () => void;
}

/**
 * A two-click confirm like Kortlægning's DetailPanel, armed per item. It
 * disarms on Escape, on a pointer press outside the armed button, when the
 * page turns busy and whenever `selection` changes (`disarms`). Callers also
 * pass `onBlur={disarm}` for keyboard users.
 */
export function useArmedConfirm<K>(busy: boolean, selection: unknown): ArmedConfirm<K> {
  const [armed, setArmed] = useState<K | null>(null);
  const button = useRef<HTMLElement | null>(null);

  useEffect(() => setArmed(null), [selection]);
  useEffect(() => {
    if (disarms({ kind: 'busy', busy })) setArmed(null);
  }, [busy]);
  useEffect(() => {
    if (armed === null) return;
    const onKey = (e: KeyboardEvent) => {
      if (disarms({ kind: 'key', key: e.key })) setArmed(null);
    };
    const onDown = (e: PointerEvent) => {
      const inside = !!button.current && e.target instanceof Node && button.current.contains(e.target);
      if (disarms({ kind: 'pointerdown', inside })) setArmed(null);
    };
    document.addEventListener('keydown', onKey, true);
    document.addEventListener('pointerdown', onDown, true);
    return () => {
      document.removeEventListener('keydown', onKey, true);
      document.removeEventListener('pointerdown', onDown, true);
    };
  }, [armed]);

  return {
    armed,
    arm: (key, el) => {
      button.current = el;
      setArmed(key);
    },
    disarm: () => setArmed(null),
  };
}
