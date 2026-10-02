// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One promise chain for a page's writes: each task starts only after every
 * task enqueued before it has settled, so requests reach the server — and
 * their responses are applied — in the order the user made them. A failing
 * task rejects its own promise but never stops the chain.
 */

export interface SerialQueue {
  enqueue(task: () => Promise<void>): Promise<void>;
  /**
   * Settles once every task enqueued so far has settled, failed ones
   * included. Never rejects. Tasks enqueued later are not waited for.
   *
   * A task must never await `idle()` of its own chain: the tail it would wait
   * for includes that task itself, so the chain would deadlock.
   */
  idle(): Promise<void>;
}

export function createSerialQueue(): SerialQueue {
  // Never rejects: it is re-pointed at a settled-either-way copy of each task.
  let tail: Promise<void> = Promise.resolve();
  return {
    enqueue(task) {
      const result = tail.then(task);
      tail = result.then(
        () => undefined,
        () => undefined,
      );
      return result;
    },
    idle() {
      return tail;
    },
  };
}
