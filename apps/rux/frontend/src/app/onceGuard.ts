// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A synchronous once-at-a-time guard for a busy-gated button (RapportPage's
 * generate guard, as a helper). `busy` only disables the button on the next
 * render, so a fast double tap inside one frame would otherwise send twice —
 * and burn a server-assigned code. While a run's promise is pending, further
 * calls are dropped; the guard reopens when it settles. A run that returns
 * nothing (it did not send) or throws reopens at once; the throw is rethrown.
 */
export interface OnceGuard {
  /** Runs `run` unless a previous run is still pending. True when it ran. */
  run: (run: () => Promise<unknown> | void) => boolean;
  readonly pending: boolean;
}

export function createOnceGuard(): OnceGuard {
  let pending = false;
  return {
    get pending() {
      return pending;
    },
    run(run) {
      if (pending) return false;
      pending = true;
      let result: Promise<unknown> | void;
      try {
        result = run();
      } catch (cause) {
        pending = false;
        throw cause;
      }
      if (result && typeof (result as Promise<unknown>).finally === 'function') {
        // Reopen on settle; a rejection is the caller's to handle, not ours to surface twice.
        result.finally(() => {
          pending = false;
        }).catch(() => {});
      } else {
        pending = false;
      }
      return true;
    },
  };
}
