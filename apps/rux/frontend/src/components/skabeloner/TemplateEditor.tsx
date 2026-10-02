// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useRef, useState, type RefObject } from 'react';

import type { ResourceKey, Template, TemplateMember } from '../../api/types';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import {
  addKey,
  categoryCounts,
  hasCategory,
  memberId,
  memberLabel,
  moveMember,
  removeMemberAt,
  resolveMembers,
  searchKeys,
  toggleCategory,
} from '../../skabeloner/members';
import { countLine, moveAnnouncement } from '../../skabeloner/model';
import styles from './TemplateEditor.module.css';

export interface TemplateEditorProps {
  template: Template;
  keys: ResourceKey[];
  nameRef: RefObject<HTMLInputElement | null>;
  /**
   * Bumped by the page when this template's rename failed: the name field
   * shows the server name again, unless it is focused (`draftResets`, R4-D1).
   */
  nameReset: number;
  onRename: (name: string) => void;
  onMembers: (next: TemplateMember[]) => void;
}

interface NameFieldProps {
  name: string;
  inputRef: RefObject<HTMLInputElement | null>;
  home: RefObject<HTMLElement | null>;
  reset: number;
  onRename: (name: string) => void;
}

/** The name input. A failed rename snaps it back through `reset`, never by a remount. */
function NameField({ name, inputRef, home, reset, onRename }: NameFieldProps) {
  const draft = useTextDraft(name, onRename, { required: true, reset });
  return (
    <label className={styles.field}>
      <span className={styles.fieldLabel}>Navn</span>
      <input ref={inputRef} className={styles.input} {...draft.props} onKeyDown={fieldKeys(draft, home)} />
    </label>
  );
}

/** A row's React key: its member id, plus an occurrence count should the server hold duplicates. */
function rowKeys(members: readonly TemplateMember[]): string[] {
  const seen = new Map<string, number>();
  return members.map((m) => {
    const id = memberId(m);
    const n = seen.get(id) ?? 0;
    seen.set(id, n + 1);
    return n === 0 ? id : `${id}#${n}`;
  });
}

/**
 * One template's editor (spec §6.2): name, Kategorier, Enkelte felter,
 * Rækkefølge and the resolved count. Every change calls `onMembers` with the
 * whole new list; the page saves it. Reorder works by drag and by the ↑/↓
 * buttons, and focus follows the moved row's button (the other arrow once
 * the row reaches an end of the list).
 */
export function TemplateEditor({ template, keys, nameRef, nameReset, onRename, onMembers }: TemplateEditorProps) {
  const members = template.members;
  const home = useRef<HTMLElement | null>(null);
  const [query, setQuery] = useState('');
  // The dragged row, by its row key (not its index, which a response could shift mid-drag).
  const [dragId, setDragId] = useState<string | null>(null);
  const [announcement, setAnnouncement] = useState('');
  const moveRefs = useRef(new Map<string, HTMLButtonElement | null>());

  const resolved = resolveMembers(members, keys);
  const missing = new Set(resolved.missing.map(memberId));
  const hits = searchKeys(keys, query, members);
  const ids = rowKeys(members);

  const move = (from: number, to: number, dir: 'up' | 'down') => {
    const next = moveMember(members, from, to);
    if (next === members) return;
    const id = ids[from];
    onMembers(next);
    setAnnouncement(moveAnnouncement(memberLabel(members[from], keys).label, to, members.length));
    requestAnimationFrame(() => {
      const other = dir === 'up' ? 'down' : 'up';
      const target = moveRefs.current.get(`${id}:${dir}`);
      (target && !target.disabled ? target : moveRefs.current.get(`${id}:${other}`))?.focus();
    });
  };

  const drop = (to: number) => {
    const from = dragId === null ? -1 : ids.indexOf(dragId);
    setDragId(null);
    if (from === -1) return;
    const next = moveMember(members, from, to);
    if (next !== members) onMembers(next); // a drop on its own place sends nothing
  };

  return (
    <section ref={home} tabIndex={-1} className={styles.panel} aria-label={`Skabelon ${template.name}`}>
      <NameField name={template.name} inputRef={nameRef} home={home} reset={nameReset} onRename={onRename} />

      <p className={styles.count} aria-live="polite">
        {countLine(resolved.keys.length)}
      </p>

      <fieldset className={styles.group}>
        <legend className={styles.heading}>Kategorier</legend>
        <div className={styles.categories}>
          {categoryCounts(keys).map(({ category, count }) => (
            <label key={category} className={styles.check}>
              <input
                type="checkbox"
                className={styles.checkbox}
                checked={hasCategory(members, category)}
                onChange={() => onMembers(toggleCategory(members, category))}
              />
              <span>{category}</span>
              <span className={styles.muted}>{count}</span>
            </label>
          ))}
        </div>
      </fieldset>

      <div className={styles.group}>
        <h3 className={styles.heading}>Enkelte felter</h3>
        <input
          type="search"
          className={styles.input}
          placeholder="Søg i felter…"
          aria-label="Søg i felter"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        {hits.length > 0 && (
          <ul className={styles.hits}>
            {hits.map((k) => (
              <li key={k.id} className={styles.hit}>
                <span>
                  {k.label} <span className={styles.muted}>· {k.category}</span>
                </span>
                <button type="button" className={styles.textBtn} onClick={() => onMembers(addKey(members, k.id))}>
                  Tilføj
                </button>
              </li>
            ))}
          </ul>
        )}
        {query.trim() !== '' && hits.length === 0 && <p className={styles.muted}>Ingen felter matcher.</p>}
      </div>

      <div className={styles.group}>
        <h3 className={styles.heading}>Rækkefølge</h3>
        {members.length === 0 ? (
          <p className={styles.muted}>Skabelonen er tom — vælg kategorier eller tilføj felter.</p>
        ) : (
          <ol className={styles.order}>
            {members.map((m, i) => {
              const id = ids[i];
              const l = memberLabel(m, keys);
              const gone = missing.has(memberId(m));
              return (
                <li
                  key={id}
                  className={`${styles.member} ${dragId === id ? styles.dragging : ''}`}
                  draggable
                  onDragStart={(e) => {
                    setDragId(id);
                    e.dataTransfer.effectAllowed = 'move';
                    // Firefox starts no drag without data (R4-D5).
                    e.dataTransfer.setData('text/plain', memberId(m));
                  }}
                  onDragOver={(e) => e.preventDefault()}
                  onDrop={(e) => {
                    e.preventDefault();
                    drop(i);
                  }}
                  onDragEnd={() => setDragId(null)}
                >
                  <span className={styles.grip} aria-hidden="true">
                    ⋮⋮
                  </span>
                  <span className={`${styles.memberMain} ${gone ? styles.gone : ''}`}>
                    <span className={styles.memberName}>{l.label}</span>
                    <span className={styles.muted}>
                      {l.kind} · {l.detail}
                    </span>
                  </span>
                  <span className={styles.memberActions}>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Flyt ${l.label} op`}
                      disabled={i === 0}
                      ref={(el) => void moveRefs.current.set(`${id}:up`, el)}
                      onClick={() => move(i, i - 1, 'up')}
                    >
                      ↑
                    </button>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Flyt ${l.label} ned`}
                      disabled={i === members.length - 1}
                      ref={(el) => void moveRefs.current.set(`${id}:down`, el)}
                      onClick={() => move(i, i + 1, 'down')}
                    >
                      ↓
                    </button>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Fjern ${l.label}`}
                      onClick={() => onMembers(removeMemberAt(members, i))}
                    >
                      ×
                    </button>
                  </span>
                </li>
              );
            })}
          </ol>
        )}
        <p className={styles.srOnly} aria-live="polite">
          {announcement}
        </p>
      </div>
    </section>
  );
}
