// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useRef, useState } from 'react';

import { api } from '../api/client';
import styles from './ThumbnailCell.module.css';

export interface ThumbnailCellProps {
  guid: string;
  hasThumbnail: boolean;
  onUploaded: () => void;
}

/**
 * The passport's thumbnail, in a single narrow table cell.
 *
 * Both the image and the empty placeholder are the click target: a Notion-style
 * table has no separate "edit" affordance, so the picture *is* the button. The
 * file input is hidden and driven from the click, because a bare
 * `<input type="file">` cannot be styled into a 48px square that matches the
 * placeholder it replaces.
 */
export function ThumbnailCell({ guid, hasThumbnail, onUploaded }: ThumbnailCellProps) {
  const inputRef = useRef<HTMLInputElement>(null);
  const [uploading, setUploading] = useState(false);

  const pick = () => {
    if (uploading) return;
    inputRef.current?.click();
  };

  const onChange = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    // Reset the input so picking the same file twice still fires a change.
    event.target.value = '';
    if (!file) return;
    setUploading(true);
    try {
      await api.uploadThumbnail(guid, file);
      onUploaded();
    } finally {
      setUploading(false);
    }
  };

  return (
    <button
      type="button"
      className={styles.cell}
      onClick={pick}
      title={hasThumbnail ? 'Replace thumbnail' : 'Add a thumbnail'}
      aria-label={hasThumbnail ? 'Replace thumbnail' : 'Add a thumbnail'}
    >
      {hasThumbnail ? (
        <img className={styles.image} src={api.materialThumbnail(guid)} alt="" />
      ) : (
        <span className={styles.placeholder} aria-hidden="true">
          <svg
            viewBox="0 0 24 24"
            width="16"
            height="16"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z" />
            <circle cx="12" cy="13" r="4" />
          </svg>
        </span>
      )}
      {uploading && (
        <span className={styles.spinnerOverlay} aria-hidden="true">
          <span className={styles.spinner} />
        </span>
      )}
      <input
        ref={inputRef}
        className={styles.input}
        type="file"
        accept="image/*"
        onChange={onChange}
        tabIndex={-1}
      />
    </button>
  );
}
