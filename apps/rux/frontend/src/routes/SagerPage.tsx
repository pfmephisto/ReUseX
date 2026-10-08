// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId, useRef, useState, type FormEvent } from 'react';

import { casesApi } from '../api/cases';
import type { CaseList, CaseSummary } from '../api/types';
import { caseHref, formatBytes, nameFromFile } from '../app/cases';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { CaseCard } from '../components/sager/CaseCard';
import controls from '../components/controls.module.css';
import { danishDate } from '../overblik/model';
import {
  cardFigures,
  cardFileLine,
  cardSubline,
  cardTitle,
  caseStats,
  caseThumbUrl,
  caseWriteErrorText,
  SERVE_DIRECTORY_COMMAND,
  sortCases,
  uploadPercent,
} from '../sager/model';
import styles from './SagerPage.module.css';

/**
 * Sager — every case the server serves (R1, ruxd multi-case phase S2), with
 * create and upload. A card opens its case with a page load (`app/cases.ts`).
 */
export function SagerPage() {
  const list = useAsync<CaseList>((s) => casesApi.list(s), []);

  if (list.error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={list.error} onRetry={list.reload} context="sagslisten" />
      </div>
    );
  }
  if (list.loading && !list.data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser sager…" />
      </div>
    );
  }
  if (!list.data) return null;

  const cases = sortCases(list.data.cases);
  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Sager</h1>
        <span className={styles.count}>{cases.length}</span>
      </header>

      {cases.length > 0 ? (
        <ul className={styles.cards} aria-label="Sager">
          {cases.map((c) => (
            <li key={c.id}>
              <CaseTile summary={c} />
            </li>
          ))}
        </ul>
      ) : (
        <p className={styles.muted}>Serveren har ingen sager endnu.</p>
      )}

      <NewCasePanel list={list.data} />
    </div>
  );
}

/**
 * One card. Its figures come with the list (`Case.summary`, read by the
 * server without opening the case), so the page makes no request per case;
 * only the plan thumbnail is fetched, lazily, and the server caches it.
 */
function CaseTile({ summary }: { summary: CaseSummary }) {
  const card = cardFigures(summary);
  return (
    <CaseCard
      href={caseHref(summary.id)}
      name={cardTitle(summary, card.record)}
      subline={card.unreadable ? 'Projektet kunne ikke læses' : cardSubline(card.record)}
      stats={card.survey ? caseStats(card.survey) : null}
      status={card.status}
      date={summary.created_at ? `Oprettet ${danishDate(summary.created_at.slice(0, 10))}` : '—'}
      fileLine={cardFileLine(summary)}
      archived={summary.archived}
      thumbUrl={caseThumbUrl(summary.id)}
    />
  );
}

/** Create an empty case, or upload a `.rux` as one. */
function NewCasePanel({ list }: { list: CaseList }) {
  const id = useId();
  const [name, setName] = useState('');
  const [file, setFile] = useState<File | null>(null);
  const [uploadName, setUploadName] = useState('');
  const [progress, setProgress] = useState<{ sent: number; total: number } | null>(null);
  const [failure, setFailure] = useState<string | null>(null);
  const action = useRef<'create' | 'upload'>('create');
  const abort = useRef<AbortController | null>(null);

  // Uploads run for minutes and change no survey state: a chain of their own.
  const { busy, mutate } = useMutationQueue({
    scope: 'page',
    onError: (cause) => {
      setProgress(null);
      setFailure(caseWriteErrorText(cause, action.current));
    },
  });

  if (!list.writable) {
    return (
      <section className={styles.panel} aria-labelledby={`${id}-ro`}>
        <h2 id={`${id}-ro`} className={styles.panelHeading}>
          Flere sager
        </h2>
        <p className={styles.text}>
          Serveren er startet med én projektfil, så den kan ikke oprette eller modtage sager. Start den med en mappe
          — hver .rux i mappen bliver en sag, og nye sager gemmes der:
        </p>
        <code className={styles.command}>{SERVE_DIRECTORY_COMMAND}</code>
      </section>
    );
  }

  const create = (e: FormEvent) => {
    e.preventDefault();
    const trimmed = name.trim();
    if (!trimmed || busy) return;
    action.current = 'create';
    setFailure(null);
    void mutate(async () => {
      const created = await casesApi.create(trimmed);
      window.location.assign(caseHref(created.id));
    });
  };

  const upload = (e: FormEvent) => {
    e.preventDefault();
    if (!file || busy) return;
    const caseName = uploadName.trim() || nameFromFile(file.name) || file.name;
    action.current = 'upload';
    setFailure(null);
    const controller = new AbortController();
    abort.current = controller;
    setProgress({ sent: 0, total: file.size });
    void mutate(async () => {
      const created = await casesApi.uploadFile(file, caseName, setProgress, controller.signal);
      window.location.assign(caseHref(created.id));
    });
  };

  const tooLarge = file !== null && file.size > list.upload.max_bytes;
  return (
    <section className={styles.panel} aria-labelledby={`${id}-new`}>
      <h2 id={`${id}-new`} className={styles.panelHeading}>
        Ny sag
      </h2>

      <form className={styles.row} onSubmit={create}>
        <label className={controls.field}>
          <span className={controls.fieldLabel}>Navn</span>
          <input
            className={controls.input}
            value={name}
            maxLength={200}
            placeholder="fx Rådhusstræde 4"
            onChange={(e) => setName(e.target.value)}
          />
        </label>
        <button type="submit" className={controls.btnPrimary} disabled={busy || name.trim() === ''}>
          Opret tom sag
        </button>
      </form>

      <h3 className={styles.subHeading}>Upload en projektfil</h3>
      <form className={styles.row} onSubmit={upload}>
        <label className={controls.field}>
          <span className={controls.fieldLabel}>Fil (.rux)</span>
          {/* The native picker's own copy is in the browser's language; this
              one speaks Danish and shows what was picked. */}
          <span className={styles.filePick}>
            <input
              className={styles.fileInput}
              type="file"
              accept=".rux"
              disabled={busy}
              onChange={(e) => {
                const picked = e.target.files?.[0] ?? null;
                setFile(picked);
                if (picked) setUploadName(nameFromFile(picked.name));
              }}
            />
            <span className={styles.fileButton} aria-hidden="true">
              Vælg fil…
            </span>
            <span className={styles.fileName}>
              {file ? `${file.name} · ${formatBytes(file.size)}` : 'Ingen fil valgt'}
            </span>
          </span>
        </label>
        <label className={controls.field}>
          <span className={controls.fieldLabel}>Navn</span>
          <input
            className={controls.input}
            value={uploadName}
            maxLength={200}
            disabled={!file}
            onChange={(e) => setUploadName(e.target.value)}
          />
        </label>
        {progress ? (
          <button type="button" className={controls.btnGhost} onClick={() => abort.current?.abort()}>
            Afbryd
          </button>
        ) : (
          <button type="submit" className={controls.btnPrimary} disabled={busy || !file || tooLarge}>
            Upload
          </button>
        )}
      </form>

      {progress && (
        <div className={styles.progress}>
          <progress className={styles.bar} max={progress.total} value={progress.sent} />
          <span className="mono">
            {uploadPercent(progress.sent, progress.total)} · {formatBytes(progress.sent)} af{' '}
            {formatBytes(progress.total)}
          </span>
        </div>
      )}
      {tooLarge && (
        <p className={styles.error} role="alert">
          Filen er {formatBytes(file.size)}; serveren tager højst imod {formatBytes(list.upload.max_bytes)}.
        </p>
      )}
      {failure && (
        <p className={styles.error} role="alert">
          {failure}
        </p>
      )}
      <p className={styles.muted}>
        Filen sendes i bidder og bliver en ny sag på serveren. Originalen på din maskine røres ikke.
      </p>
    </section>
  );
}
