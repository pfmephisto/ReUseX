title: On-site keeps writes made without signal
labels: gui, enhancement, frontend

## Problem
On-site is used in basements, plant rooms and stairwells, where the phone
loses the network. A ★, note or sample registered then fails, and the
toast says so. The note draft stays in its field until the next blur, but
moving on with `Videre →` remounts the sheet (R6) and the unsent draft is
gone. Nothing is queued for later.

Separately, every page's writes now join one app-wide chain
(`appWriteChain`, Task 6), and each case screen's first load awaits
`appWriteChain.idle()` before its own `GET`. The chain has no timeout: one
write stuck behind a lost connection — exactly the On-site scenario this
issue is about — stalls every other case screen's first load, not just
On-site's own.

## Proposed fix
- [ ] A persisted outbox (IndexedDB) behind `useMutationQueue` for On-site's
      three writes, replayed in order when the connection returns, with a
      visible "n ændringer venter på forbindelse" line.
- [ ] Idempotency for the sample POST (a client-generated key), so a replay
      after a lost response does not register the sample twice.
- [ ] A conflict rule for a note edited both on the phone and in Kortlægning
      meanwhile (last write wins, with a toast naming the overwritten text).
- [ ] A timeout (or an abort control) on `appWriteChain`'s in-flight entry,
      so a write that outlives it no longer blocks every case screen's first
      load behind it.

category=I/O estimate=3d
