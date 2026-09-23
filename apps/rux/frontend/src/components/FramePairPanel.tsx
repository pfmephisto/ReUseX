// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import { api } from '../api/client';
import type { FrameImageKind } from '../api/types';
import { IMAGE_KIND_LABELS, IMAGE_KINDS, NORMALIZED_KINDS } from '../data/framePair';
import styles from './FramePairPanel.module.css';

export type PanelViewMode = 'image' | 'cloud';

export interface FramePairPanelProps {
  /** Which side of the pair this panel represents. */
  label: 'A' | 'B';
  /** The frame being shown; null when nothing is selected yet. */
  frameId: number | null;
  viewMode: PanelViewMode;
  imageKind: FrameImageKind;
  onViewModeChange: (mode: PanelViewMode) => void;
  onKindChange: (kind: FrameImageKind) => void;
  /**
   * When set, renders the panel's image as an absolute overlay on top of the
   * preceding panel. The parent positions an `.overlayAnchor` relative container
   * and mounts panel B with these props so the two images can be compared
   * by blending.
   */
  overlayStyle?: React.CSSProperties;
}

/**
 * One half of the frame-pair inspector.
 *
 * Manages image-kind selection and a "cloud mode" stub. In image mode the
 * panel displays whichever of the four frame image kinds the user has selected,
 * using `normalize=true` for the three 16-bit kinds so they are visible.
 *
 * Cloud mode is explicitly deferred: per-frame point clouds are not available
 * from the current API (there is no `GET /frames/{id}/depth-cloud` endpoint).
 * The stub shows a clear placeholder rather than hiding the feature, so the
 * user can see what is planned. See issue #446 for the backend follow-up.
 */
export function FramePairPanel({
  label,
  frameId,
  viewMode,
  imageKind,
  onViewModeChange,
  onKindChange,
  overlayStyle,
}: FramePairPanelProps) {
  return (
    <div className={`${styles.panel} ${overlayStyle ? styles.overlay : ''}`} style={overlayStyle}>
      <header className={styles.head}>
        <span className={styles.panelLabel} aria-label={`Panel ${label}`}>
          {label}
        </span>
        {frameId !== null && (
          <span className={`${styles.frameId} mono`}>Frame {frameId}</span>
        )}
        <div className={styles.viewToggle} role="group" aria-label="View mode">
          <button
            type="button"
            className={`${styles.modeBtn} ${viewMode === 'image' ? styles.modeBtnActive : ''}`}
            onClick={() => onViewModeChange('image')}
          >
            Image
          </button>
          <button
            type="button"
            className={`${styles.modeBtn} ${viewMode === 'cloud' ? styles.modeBtnActive : ''}`}
            onClick={() => onViewModeChange('cloud')}
          >
            Cloud
          </button>
        </div>
      </header>

      {viewMode === 'image' && (
        <nav className={styles.kindBar} aria-label="Image kind">
          {IMAGE_KINDS.map((kind) => (
            <button
              key={kind}
              type="button"
              className={`${styles.kindBtn} ${kind === imageKind ? styles.kindBtnActive : ''}`}
              onClick={() => onKindChange(kind)}
            >
              {IMAGE_KIND_LABELS[kind]}
            </button>
          ))}
        </nav>
      )}

      <div className={styles.body}>
        {viewMode === 'image' ? (
          frameId === null ? (
            <EmptySlot label={label} />
          ) : (
            <FrameImage id={frameId} kind={imageKind} />
          )
        ) : (
          <CloudStub />
        )}
      </div>
    </div>
  );
}

// -------------------------------------------------------------------------- //

function EmptySlot({ label }: { label: 'A' | 'B' }) {
  return (
    <div className={styles.emptySlot} aria-label={`No frame selected for panel ${label}`}>
      <span className={styles.emptyIcon} aria-hidden="true">⬚</span>
      <p className={styles.emptyText}>Select frame {label}</p>
    </div>
  );
}

// -------------------------------------------------------------------------- //

interface FrameImageProps {
  id: number;
  kind: FrameImageKind;
}

function FrameImage({ id, kind }: FrameImageProps) {
  const [failed, setFailed] = useState(false);

  // Reset the error state when the frame or kind changes.
  const key = `${id}-${kind}`;

  return (
    <div className={styles.imageWrapper} key={key}>
      {failed ? (
        <div className={styles.imageError}>
          <span>Image could not be loaded</span>
          <button type="button" className={styles.retryBtn} onClick={() => setFailed(false)}>
            Retry
          </button>
        </div>
      ) : (
        <img
          className={styles.image}
          src={api.frameImageUrl(id, kind, {
            normalize: NORMALIZED_KINDS.has(kind),
          })}
          alt={`${kind} image for frame ${id}`}
          decoding="async"
          onError={() => setFailed(true)}
        />
      )}
      {NORMALIZED_KINDS.has(kind) && (
        <p className={styles.normalizedNote}>
          Stretched to this frame&rsquo;s observed range — no metric scale.
        </p>
      )}
    </div>
  );
}

// -------------------------------------------------------------------------- //

/**
 * Placeholder for the cloud view.
 *
 * The current API exposes no endpoint for per-frame point clouds. Showing the
 * scan cloud would be misleading (it is not this frame), and reconstructing a
 * cloud client-side from the depth image is not reliable: the browser canvas
 * API gives only 8-bit depth precision and the normalized depth carries no
 * metric scale.
 *
 * The backend work required: `GET /api/v1/frames/{id}/depth-cloud` —
 * backproject the stored depth image using the stored sensor intrinsics and
 * return a RUXP binary stream (same format as `/clouds/{name}/points?format=binary`).
 * Track in issue #446.
 */
function CloudStub() {
  return (
    <div className={styles.cloudStub}>
      <span className={styles.cloudIcon} aria-hidden="true">⬡</span>
      <p className={styles.cloudTitle}>Per-frame point cloud</p>
      <p className={styles.cloudDetail}>
        Requires a backend endpoint that is not yet implemented.
      </p>
      <p className={styles.cloudContract}>
        Expected: <code>GET /api/v1/frames/&#123;id&#125;/depth-cloud</code>
        <br />
        Format: RUXP binary stream (same as <code>/clouds/&#123;name&#125;/points?format=binary</code>)
      </p>
    </div>
  );
}
