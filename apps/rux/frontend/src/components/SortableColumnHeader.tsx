// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useSortable } from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import type React from 'react';

import styles from './MaterialTable.module.css';

interface SortableColumnHeaderProps {
  colId: string;
  width: number;
  hasActiveFilter?: boolean;
  onClick: (e: React.MouseEvent<HTMLTableCellElement>) => void;
  onResizerMouseDown: (e: React.MouseEvent) => void;
  children: React.ReactNode;
}

/**
 * A `<th>` that can be dragged to reorder its column.
 *
 * Wraps `useSortable` from @dnd-kit/sortable. The drag handle (⠿) is a
 * separate span so that clicking the rest of the header still opens the
 * column menu. We stop propagation on the handle's click so the outer
 * `onClick` (which opens the column menu) is not triggered during a drag.
 */
export function SortableColumnHeader({
  colId,
  width,
  hasActiveFilter,
  onClick,
  onResizerMouseDown,
  children,
}: SortableColumnHeaderProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({ id: colId });

  const style: React.CSSProperties = {
    width,
    position: 'relative',
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.4 : 1,
    zIndex: isDragging ? 10 : undefined,
  };

  return (
    <th
      ref={setNodeRef}
      className={`${styles.headerCell}${hasActiveFilter ? ` ${styles.headerFiltered}` : ''}`}
      style={style}
      onClick={onClick}
    >
      <span
        className={styles.dragHandle}
        {...attributes}
        {...listeners}
        data-no-menu="true"
        onClick={(e) => e.stopPropagation()}
        aria-label="Drag to reorder column"
        title="Drag to reorder"
      >
        ⠿
      </span>
      <span className={styles.headerLabel}>{children}</span>
      {hasActiveFilter && (
        <span className={styles.filterIndicator} aria-label="Column has active filter">
          ◈
        </span>
      )}
      <div
        className={styles.resizer}
        data-no-menu="true"
        onMouseDown={(e) => {
          e.stopPropagation();
          onResizerMouseDown(e);
        }}
      />
    </th>
  );
}
