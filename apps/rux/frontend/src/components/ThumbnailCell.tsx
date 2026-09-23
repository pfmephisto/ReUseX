// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';
import { useNavigate } from 'react-router-dom';

import { api } from '../api/client';
import styles from './ThumbnailCell.module.css';

export interface ThumbnailCellProps {
  guid: string;
  /** Hint from the server — used only for the first render. The cell tracks
   *  actual image availability itself via onLoad/onError, and bumps a version
   *  counter on upload so the browser re-fetches without a page reload. */
  hasThumbnail: boolean;
  onUploaded: () => void;
}

/**
 * The passport's thumbnail, in a single narrow table cell.
 *
 * The cell always tries to load the thumbnail URL and shows the image on
 * success, the camera-icon placeholder on failure (404 or no thumbnail yet).
 * A local version counter is bumped on every successful upload to force the
 * browser to re-fetch rather than show a stale cached response.
 */
export function ThumbnailCell({ guid, hasThumbnail, onUploaded }: ThumbnailCellProps) {
  const navigate = useNavigate();
  const inputRef = useRef<HTMLInputElement>(null);
  const rootRef = useRef<HTMLDivElement>(null);
  const [uploading, setUploading] = useState(false);
  const [menuOpen, setMenuOpen] = useState(false);
  // `loaded` tracks whether the img element actually loaded successfully.
  // Initialise from the server hint so the first render is correct.
  const [loaded, setLoaded] = useState(hasThumbnail);
  // Incremented on every successful upload to bust the browser image cache.
  const [version, setVersion] = useState(0);

  // Re-probe when the server hint changes (e.g. after details refresh).
  useEffect(() => {
    setLoaded(hasThumbnail);
  }, [hasThumbnail]);

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
    event.target.value = '';
    if (!file) return;
    setUploading(true);
    try {
      await api.uploadThumbnail(guid, file);
      setVersion((v) => v + 1); // force img refetch
      setLoaded(true);
      onUploaded();
    } finally {
      setUploading(false);
    }
  };

  const thumbnailSrc = `${api.materialThumbnail(guid)}${version > 0 ? `?v=${version}` : ''}`;

  return (
    <div className={styles.root} ref={rootRef}>
      <button
        type="button"
        className={styles.cell}
        onClick={openMenu}
        title={loaded ? 'Change thumbnail' : 'Add a thumbnail'}
        aria-label={loaded ? 'Change thumbnail' : 'Add a thumbnail'}
        aria-haspopup="menu"
        aria-expanded={menuOpen}
      >
        {/* Always render the img; hide it when load fails so the placeholder shows */}
        <img
          className={styles.image}
          style={{ display: loaded ? 'block' : 'none' }}
          src={thumbnailSrc}
          alt=""
          onLoad={() => setLoaded(true)}
          onError={() => setLoaded(false)}
        />
        {!loaded && (
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
