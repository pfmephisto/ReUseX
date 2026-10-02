// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { ResourceKey, Template, TemplateMember } from '../api/types';
import { createOnceGuard } from '../app/onceGuard';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { appWriteChain } from '../app/writeChain';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { TemplateEditor } from '../components/skabeloner/TemplateEditor';
import { TemplateList } from '../components/skabeloner/TemplateList';
import {
  createLatestGate,
  missingSeeds,
  nextTemplateName,
  replaceTemplate,
  selectAfterDelete,
  templateErrorMessage,
  withMembers,
} from '../skabeloner/model';
import styles from './SkabelonerPage.module.css';

/**
 * Skabeloner — the project's templates (spec §6.2). List on the left, editor
 * on the right (stacked on a phone). Every write joins the app write chain;
 * member edits save on change as full snapshots, and only the newest edit's
 * response is applied (R3). A failed write re-reads the list in the same
 * queued task, so optimistic state never outlives a failure.
 */
export function SkabelonerPage() {
  const loaded = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.templates(s), api.resourceKeys(s)])),
    [],
  );
  const [templates, setTemplatesState] = useState<Template[] | null>(null);
  const [keys, setKeys] = useState<ResourceKey[]>([]);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const nameRef = useRef<HTMLInputElement | null>(null);
  const [focusName, setFocusName] = useState(false);
  // Bumped when a rename fails, so the name field drops its draft and shows the server name (R4-D1).
  const [nameEpoch, setNameEpoch] = useState(0);
  const [gate] = useState(createLatestGate);
  const [creating] = useState(createOnceGuard);
  // The newest list, for queued tasks whose render-time closure may be stale (R4-D7).
  const latest = useRef<Template[] | null>(null);
  const toast = useToast(3200);
  const { busy, mutate } = useMutationQueue({ onError: (cause) => toast.show(templateErrorMessage(cause)) });

  const setTemplates = (fn: (prev: Template[] | null) => Template[] | null) => {
    latest.current = fn(latest.current);
    setTemplatesState(latest.current);
  };

  const adopt = (list: Template[]) => {
    setTemplates(() => list);
    setSelectedId((id) => (id !== null && list.some((t) => t.id === id) ? id : (list[0]?.id ?? null)));
  };

  useEffect(() => {
    if (!loaded.data) return;
    const [list, catalogue] = loaded.data;
    adopt(list);
    setKeys(catalogue);
  }, [loaded.data]);

  // After "Ny skabelon" / "Omdøb", focus the name field once it is rendered.
  useEffect(() => {
    if (focusName && nameRef.current) {
      nameRef.current.focus();
      nameRef.current.select();
      setFocusName(false);
    }
  }, [focusName, selectedId, templates]);

  const relist = async () => adopt(await api.templates());

  /** Run a write; on failure re-read the list before the error surfaces. */
  const write = (run: () => Promise<void>) =>
    mutate(async () => {
      try {
        await run();
      } catch (cause) {
        await relist().catch(() => undefined);
        throw cause;
      }
    });

  const onMembers = (id: number, next: TemplateMember[]) => {
    setTemplates((prev) => (prev ? withMembers(prev, id, next) : prev));
    const ticket = gate.next(id);
    void write(async () => {
      const saved = await api.patchTemplate(id, { members: next });
      if (gate.isLatest(id, ticket)) setTemplates((prev) => (prev ? replaceTemplate(prev, saved) : prev));
    });
  };

  // Only name + updated_at are merged: queued member edits must not be overwritten (R4-D2).
  const onRename = (id: number, name: string) =>
    void write(async () => {
      try {
        const saved = await api.patchTemplate(id, { name });
        setTemplates((prev) =>
          prev ? prev.map((t) => (t.id === id ? { ...t, name: saved.name, updated_at: saved.updated_at } : t)) : prev,
        );
      } catch (cause) {
        setNameEpoch((n) => n + 1);
        throw cause;
      }
    });

  const onNew = () =>
    void creating.run(() =>
      write(async () => {
        const names = (latest.current ?? []).map((t) => t.name);
        const created = await api.createTemplate({ name: nextTemplateName(names), members: [] });
        setTemplates((prev) => [...(prev ?? []), created]);
        setSelectedId(created.id);
        setFocusName(true);
      }),
    );

  const onDuplicate = (id: number) =>
    void write(async () => {
      const copy = await api.duplicateTemplate(id);
      await relist();
      setSelectedId(copy.id);
    });

  // TemplateList arms the button first (two-click confirm, R4-D14).
  const onDelete = (id: number) =>
    void write(async () => {
      await api.deleteTemplate(id);
      const ids = (latest.current ?? []).map((x) => x.id);
      setTemplates((prev) => (prev ? prev.filter((x) => x.id !== id) : prev));
      setSelectedId((sel) => (sel === id ? selectAfterDelete(ids, id) : sel));
    });

  const onRestoreSeeds = () =>
    void write(async () => {
      await api.restoreSeedTemplates();
      await relist();
    });

  if (loaded.error && !templates) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={loaded.error} onRetry={loaded.reload} context="skabelonerne" />
      </div>
    );
  }
  if (!templates) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser skabeloner…" />
      </div>
    );
  }

  const selected = templates.find((t) => t.id === selectedId) ?? null;

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Skabeloner</h2>
        <span className={styles.sub}>Feltudvalg til Kortlægning og Rapport</span>
      </header>
      <div className={styles.layout}>
        <TemplateList
          templates={templates}
          keys={keys}
          selectedId={selectedId}
          busy={busy}
          missingSeeds={missingSeeds(templates)}
          onSelect={setSelectedId}
          onNew={onNew}
          onRename={(id) => {
            setSelectedId(id);
            setFocusName(true);
          }}
          onDuplicate={onDuplicate}
          onDelete={onDelete}
          onRestoreSeeds={onRestoreSeeds}
        />
        {selected ? (
          <TemplateEditor
            key={selected.id}
            template={selected}
            keys={keys}
            nameRef={nameRef}
            nameEpoch={nameEpoch}
            onRename={(name) => onRename(selected.id, name)}
            onMembers={(next) => onMembers(selected.id, next)}
          />
        ) : (
          <EmptyState
            title="Ingen skabelon valgt"
            detail={templates.length === 0 ? 'Opret en ny, eller gendan standardskabelonerne.' : 'Vælg en skabelon i listen.'}
          />
        )}
      </div>
      <Toast message={toast.message} />
    </div>
  );
}
