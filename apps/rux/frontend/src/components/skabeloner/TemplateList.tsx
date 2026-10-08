// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ResourceKey, Template } from '../../api/types';
import { useArmedConfirm } from '../../app/useArmedConfirm';
import { resolveMembers } from '../../skabeloner/members';
import { countLine, deleteConfirmText, restoreSeedsTitle, seedLabel } from '../../skabeloner/model';
import { Pill } from '../Pill';
import styles from './TemplateList.module.css';
import { useCanEdit } from '../../app/CaseRoleContext';

export interface TemplateListProps {
  templates: Template[];
  /** The catalogue, so the counts match the editor's local resolution (R4-D12). */
  keys: ResourceKey[];
  selectedId: number | null;
  busy: boolean;
  missingSeeds: string[];
  onSelect: (id: number) => void;
  onNew: () => void;
  onRename: (id: number) => void;
  onDuplicate: (id: number) => void;
  onDelete: (id: number) => void;
  onRestoreSeeds: () => void;
}

/**
 * The template list and its actions (spec §6.2). Actions act on the selected
 * row. "Slet" is a two-click confirm like Kortlægning's DetailPanel: the
 * first click arms it and says what will happen, the second deletes. Escape,
 * a press elsewhere, a busy page or another selection disarms it.
 */
export function TemplateList(p: TemplateListProps) {
  const canEdit = useCanEdit();
  const sel = p.selectedId;
  const selected = p.templates.find((t) => t.id === sel) ?? null;
  const confirm = useArmedConfirm<number>(p.busy, sel);
  const armed = sel !== null && confirm.armed === sel;

  return (
    <aside className={styles.panel} aria-label="Skabeloner">
      <div className={styles.toolbar}>
        {canEdit && (
          <button type="button" className={styles.btnPrimary} onClick={p.onNew} disabled={p.busy}>
            Ny skabelon
          </button>
        )}
      </div>
      <ul className={styles.list}>
        {p.templates.map((t) => (
          <li key={t.id}>
            <button
              type="button"
              className={`${styles.row} ${t.id === sel ? styles.active : ''}`}
              aria-current={t.id === sel ? 'true' : undefined}
              onClick={() => p.onSelect(t.id)}
            >
              <span className={styles.name}>{t.name}</span>
              <span className={styles.meta}>
                {countLine(resolveMembers(t.members, p.keys).keys.length)}
                {seedLabel(t.seed) && <Pill tone="accent">{seedLabel(t.seed)}</Pill>}
              </span>
            </button>
          </li>
        ))}
      </ul>
      <div className={styles.actions}>
        <button
          type="button"
          className={styles.btnGhost}
          disabled={sel === null || p.busy}
          onClick={() => sel !== null && p.onRename(sel)}
        >
          Omdøb
        </button>
        <button
          type="button"
          className={styles.btnGhost}
          disabled={sel === null || p.busy}
          onClick={() => sel !== null && p.onDuplicate(sel)}
        >
          Dupliker
        </button>
        <button
          type="button"
          className={styles.btnDanger}
          disabled={sel === null || p.busy}
          onClick={(e) => {
            if (sel === null) return;
            if (!armed) {
              confirm.arm(sel, e.currentTarget);
              return;
            }
            confirm.disarm();
            p.onDelete(sel);
          }}
          onBlur={confirm.disarm}
        >
          {armed ? 'Bekræft: slet' : 'Slet'}
        </button>
      </div>
      {armed && selected && (
        <p className={styles.confirm} role="status">
          {deleteConfirmText(selected)}
        </p>
      )}
      <button
        type="button"
        className={styles.textBtn}
        onClick={p.onRestoreSeeds}
        disabled={p.busy || p.missingSeeds.length === 0}
        title={restoreSeedsTitle(p.missingSeeds)}
      >
        Gendan standardskabeloner
      </button>
    </aside>
  );
}
