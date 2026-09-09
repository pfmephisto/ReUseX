// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useMemo, useState } from 'react';

import type { StageCard as StageCardModel } from '../pipeline/stageModel';
import {
  buildParameters,
  formErrors,
  initialFormState,
  type FormState,
} from '../pipeline/params';
import { ParameterForm } from './ParameterForm';
import { StageProgress } from './StageProgress';
import styles from './StageCard.module.css';

export interface StageCardProps {
  card: StageCardModel;
  /** Rejected submissions surface here; the card does not own the request. */
  submitError?: string;
  onRun: (stage: string, parameters: Record<string, unknown>) => void;
  onCancel: (jobId: string) => void;
}

const STATE_LABEL: Record<StageCardModel['state'], string> = {
  idle: 'Not run',
  queued: 'Queued',
  running: 'Running',
  succeeded: 'Succeeded',
  failed: 'Failed',
  cancelled: 'Cancelled',
};

/**
 * One runnable stage: what it does, whether it can run, its knobs, and what
 * happened last time.
 *
 * The card never decides anything itself — `card` is a view model built by
 * `buildStageCards()`, which is where the awkward combinations (blocked while
 * running, cancel-requested-but-succeeded) are resolved and tested.
 */
export function StageCard({ card, submitError, onRun, onCancel }: StageCardProps) {
  const [state, setState] = useState<FormState>(() => initialFormState(card.parameters));
  const [open, setOpen] = useState(false);

  const errors = useMemo(
    () => formErrors(card.parameters, state),
    [card.parameters, state],
  );
  const invalid = Object.keys(errors).length > 0;

  const change = useCallback((key: string, value: string | boolean) => {
    setState((previous) => ({ ...previous, [key]: value }));
  }, []);

  const reset = useCallback(
    () => setState(initialFormState(card.parameters)),
    [card.parameters],
  );

  const run = useCallback(() => {
    if (invalid) return;
    onRun(card.stage, buildParameters(card.parameters, state));
  }, [card.parameters, card.stage, invalid, onRun, state]);

  const busy = card.state === 'queued' || card.state === 'running';

  return (
    <section className={`${styles.card} ${styles[card.state]}`} aria-label={card.stage}>
      <header className={styles.head}>
        <h3 className={styles.title}>{card.stage}</h3>
        <span className={styles.state}>{STATE_LABEL[card.state]}</span>
        <code className={`${styles.command} mono`}>{card.command}</code>
      </header>

      <p className={styles.summary}>{card.summary}</p>

      {card.outputs.length > 0 && (
        <p className={styles.outputs}>
          Writes <span className="mono">{card.outputs.join(', ')}</span>
        </p>
      )}

      {/* The blocked reason names the inputs AND the command that produces
          them, so the card is a next step rather than a dead end. */}
      {card.blocked && (
        <p className={styles.blocked} role="note">
          {card.blocked}
        </p>
      )}

      {card.warnings.map((warning) => (
        <p key={warning} className={styles.warning}>
          {warning}
        </p>
      ))}

      {!card.runnable && (
        <p className={styles.blocked} role="note">
          No runner yet — run <span className="mono">{card.command}</span> from the command
          line.
        </p>
      )}

      {card.progress && <StageProgress progress={card.progress} />}

      {card.state === 'queued' && <p className={styles.queued}>Waiting for the worker…</p>}

      {card.queuedCount > 0 && (
        <p className={styles.outcome}>
          {card.queuedCount} more run{card.queuedCount === 1 ? '' : 's'} queued behind this
          one.
        </p>
      )}

      {/* Honest cancel semantics (#274 S4): a stage that finished despite the
          cancel says so, rather than claiming it was cancelled. */}
      {card.outcome && (
        <p
          className={`${styles.outcome} ${card.state === 'failed' ? styles.outcomeBad : ''}`}
          role="status"
        >
          {card.outcome}
        </p>
      )}

      {card.job?.cancel_requested && busy && (
        <p className={styles.outcome}>
          Cancel requested
          {card.cancellable
            ? ' — stopping at the next checkpoint.'
            : ' — this stage cannot stop mid-run, so it will finish and keep its output.'}
        </p>
      )}

      {submitError && <p className={styles.error}>{submitError}</p>}

      {card.runnable && card.parameters.length > 0 && (
        <>
          <button
            type="button"
            className={styles.disclose}
            aria-expanded={open}
            onClick={() => setOpen((value) => !value)}
          >
            {open ? 'Hide parameters' : 'Parameters'}
          </button>
          {open && (
            <ParameterForm
              parameters={card.parameters}
              state={state}
              errors={errors}
              disabled={busy}
              onChange={change}
              onReset={reset}
            />
          )}
        </>
      )}

      <footer className={styles.actions}>
        <button
          type="button"
          className={styles.run}
          disabled={!card.canRun || invalid}
          onClick={run}
          title={
            invalid
              ? 'Fix the highlighted parameters first'
              : (card.blocked ?? undefined)
          }
        >
          Run
        </button>
        {card.canCancel && card.job && (
          <button
            type="button"
            className={styles.cancel}
            disabled={card.job.cancel_requested === true}
            onClick={() => onCancel(card.job!.id)}
          >
            {card.job.cancel_requested ? 'Cancelling…' : 'Cancel'}
          </button>
        )}
        {card.job && <span className={`${styles.jobId} mono`}>{card.job.id.slice(0, 8)}</span>}
      </footer>
    </section>
  );
}
