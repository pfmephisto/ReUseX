// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';
import { useNavigate } from 'react-router-dom';

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
 * Clicking the picture no longer opens the file picker directly: it opens a
 * small action menu offering "Upload image" (the file picker) or "Capture from
 * viewport", which routes to `/viewport?captureFor=<guid>` where the viewport
 * grows a "Set as thumbnail" button. The file input stays hidden and is driven
 * from the menu, because a bare `<input type="file">` cannot be styled into the
 * square the placeholder occupies.
 */
export function ThumbnailCell({ guid, hasThumbnail, onUploaded }: ThumbnailCellProps) {
  const navigate = useNavigate();
  const inputRef = useRef<HTMLInputElement>(null);
  const rootRef = useRef<HTMLDivElement>(null);
  const [uploading, setUploading] = useState(false);
  const [menuOpen, setMenuOpen] = useState(false);

  // Close the menu on any outside click.
  useEffect(() => {
    if (!menuOpen) return;
    const handler = (event: MouseEvent) => {
      if (rootRef.current && !rootRef.current.contains(event.target as Node)) {
        setMenuOpen(false);
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [menuOpen]);

  const openMenu = () => {
    if (uploading) return;
    setMenuOpen((open) => !open);
  };

  const pickFile = () => {
    setMenuOpen(false);
    inputRef.current?.click();
  };

  const captureFromViewport = () => {
    setMenuOpen(false);
    navigate(`/viewport?captureFor=${encodeURIComponent(guid)}`);
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
    <div className={styles.root} ref={rootRef}>
      <button
        type="button"
        className={styles.cell}
        onClick={openMenu}
        title={hasThumbnail ? 'Change thumbnail' : 'Add a thumbnail'}
        aria-label={hasThumbnail ? 'Change thumbnail' : 'Add a thumbnail'}
        aria-haspopup="menu"
        aria-expanded={menuOpen}
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
      </button>

      {menuOpen && (
        <div className={styles.menu} role="menu">
          <button type="button" className={styles.menuItem} role="menuitem" onClick={pickFile}>
            Upload image
          </button>
          <button
            type="button"
            className={styles.menuItem}
            role="menuitem"
            onClick={captureFromViewport}
          >
            Capture from viewport
          </button>
        </div>
      )}

      <input
        ref={inputRef}
        className={styles.input}
        type="file"
        accept="image/*"
        onChange={onChange}
        tabIndex={-1}
      />
    </div>
  );
}
