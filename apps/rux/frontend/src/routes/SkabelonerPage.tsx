// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { PropertyDefinition, ResourceKey, Template, TemplateMember } from '../api/types';
import { createOnceGuard } from '../app/onceGuard';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { appWriteChain } from '../app/writeChain';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { ColumnList } from '../components/skabeloner/ColumnList';
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
import { columnErrorMessage, optionsChanged, optionsError, renameError } from '../skabeloner/columns';
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
    (s) =>
      appWriteChain.idle().then(() => Promise.all([api.templates(s), api.resourceKeys(s), api.resourceColumns(s)])),
    [],
  );
  const [templates, setTemplatesState] = useState<Template[] | null>(null);
  const [keys, setKeys] = useState<ResourceKey[]>([]);
  const [columns, setColumns] = useState<PropertyDefinition[]>([]);
  // Egne felter: an inline error per column id, and a reset counter per refused field (R4-D1).
  const [columnErrors, setColumnErrors] = useState<Record<string, string>>({});
  const [columnResets, setColumnResets] = useState<Record<string, number>>({});
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const nameRef = useRef<HTMLInputElement | null>(null);
  const [focusName, setFocusName] = useState(false);
  // Bumped per template when its rename fails, so its name field shows the server name again
  // unless the user is typing in it (draftResets, R4-D1).
  const [nameResets, setNameResets] = useState<Record<number, number>>({});
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
    const [list, catalogue, cols] = loaded.data;
    adopt(list);
    setKeys(catalogue);
    setColumns(cols);
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
        setNameResets((prev) => ({ ...prev, [id]: (prev[id] ?? 0) + 1 }));
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

  // ---- Egne felter (R4-EF) ----

  const setColumnError = (id: string, message: string | null) =>
    setColumnErrors((prev) => {
      const next = { ...prev };
      if (message === null) delete next[id];
      else next[id] = message;
      return next;
    });

  /** Reset one column field so it drops its draft and shows the server value (R4-D1). */
  const snapBack = (id: string, field: 'name' | 'options') =>
    setColumnResets((prev) => ({ ...prev, [`${id}:${field}`]: (prev[`${id}:${field}`] ?? 0) + 1 }));

  /** A refused field commit: say why next to the row and snap the field back. */
  const refuse = (id: string, field: 'name' | 'options', message: string) => {
    setColumnError(id, message);
    snapBack(id, field);
  };

  /**
   * A column write, on the page's chain. Afterwards — success or not — the
   * columns, the catalogue and the templates are re-read: a rename changes
   * member labels, and a delete turns `col:<id>` members into missing ones.
   * A refused rename/options edit is shown next to its row; a failed delete
   * goes to the toast.
   */
  const columnWrite = (id: string, field: 'name' | 'options' | null, run: () => Promise<void>) =>
    void write(async () => {
      try {
        await run();
        setColumnError(id, null);
      } catch (cause) {
        if (field === null) toast.show(columnErrorMessage(cause));
        else refuse(id, field, columnErrorMessage(cause));
      }
      const [cols, catalogue, list] = await Promise.all([api.resourceColumns(), api.resourceKeys(), api.templates()]);
      setColumns(cols);
      setKeys(catalogue);
      adopt(list);
    });

  const onRenameColumn = (column: PropertyDefinition, name: string) => {
    const trimmed = name.trim();
    if (trimmed === column.name) return;
    const invalid = renameError(trimmed, column.name, keys.map((k) => k.label));
    if (invalid) return refuse(column.id, 'name', invalid);
    columnWrite(column.id, 'name', async () => {
      await api.updateResourceColumn(column.id, { name: trimmed });
    });
  };

  const onColumnOptions = (column: PropertyDefinition, text: string) => {
    const invalid = optionsError(text);
    if (invalid) return refuse(column.id, 'options', invalid);
    const next = optionsChanged(text, column.options ?? []);
    if (next === null) {
      // Same list, other spelling ("a, b" for "a⏎b"): show it the server's way.
      snapBack(column.id, 'options');
      return;
    }
    columnWrite(column.id, 'options', async () => {
      await api.updateResourceColumn(column.id, { options: next });
    });
  };

  // ColumnList arms the button first (two-click confirm, R4-D14).
  const onDeleteColumn = (column: PropertyDefinition) =>
    columnWrite(column.id, null, async () => {
      await api.deleteResourceColumn(column.id);
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
        <div className={styles.listArea}>
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
        </div>
        <div className={styles.editorArea}>
          {selected ? (
            <TemplateEditor
              key={selected.id}
              template={selected}
              keys={keys}
              nameRef={nameRef}
              nameReset={nameResets[selected.id] ?? 0}
              onRename={(name) => onRename(selected.id, name)}
              onMembers={(next) => onMembers(selected.id, next)}
            />
          ) : (
            <EmptyState
              title="Ingen skabelon valgt"
              detail={
                templates.length === 0
                  ? 'Opret en ny, eller gendan standardskabelonerne.'
                  : 'Vælg en skabelon i listen.'
              }
            />
          )}
        </div>
        <div className={styles.columnsArea}>
          <ColumnList
            columns={columns}
            busy={busy}
            errors={columnErrors}
            resets={columnResets}
            onRename={onRenameColumn}
            onOptions={onColumnOptions}
            onDelete={onDeleteColumn}
          />
        </div>
      </div>
      <Toast message={toast.message} />
    </div>
  );
}
