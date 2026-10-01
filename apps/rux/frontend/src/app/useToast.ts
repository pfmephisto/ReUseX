// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

/** One transient message at a time; a new one replaces the old and restarts the timer. */
export function useToast(ms = 1800): { message: string | null; show: (m: string) => void } {
  const [message, setMessage] = useState<string | null>(null);
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
  const show = useCallback(
    (m: string) => {
      setMessage(m);
      clearTimeout(timer.current);
      timer.current = setTimeout(() => setMessage(null), ms);
    },
    [ms],
  );
  useEffect(() => () => clearTimeout(timer.current), []);
  return { message, show };
}
