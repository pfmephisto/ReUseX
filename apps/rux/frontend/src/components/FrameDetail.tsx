// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import { api } from '../api/client';
import type { FrameImageKind, FrameInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { formatFixed, formatText } from '../data/format';
import {
  formatFrameTimestamp,
  frameImageSlots,
  poseRows,
  poseTranslation,
} from '../data/framesModel';
import { ErrorBanner } from './ErrorBanner';
import { Spinner } from './Spinner';
import styles from './FrameDetail.module.css';

/**
 * Everything known about one sensor frame.
 *
 * The four images are shown at full size and each is labelled with what it
 * actually is. That labelling is not decoration: three of the four are
 * requested with `normalize=true`, which the contract is explicit about being a
 * *rendering* of a stored measurement and not the measurement — a normalised
 * depth image has no metric scale at all, because the mapping is stretched to
 * whatever range that particular frame happened to observe.
 *
 * The server reports that range in `X-Image-Range-Min`/`-Max`, and this pane
 * deliberately does not use it. A plain `<img>` cannot read response headers,
 * and building the UI around them would mean fetching every image twice — once
 * for the headers and once for the browser — to caption a picture that is not
 * measurable anyway. The contract calls them a debugging convenience; they are
 * treated as one.
 */
export interface FrameDetailProps {
  id: number;
  onClose: () => void;
}

export function FrameDetail({ id, onClose }: FrameDetailProps) {
  const frame = useAsync((signal) => api.frame(id, signal), [id]);

  return (
    <aside className={styles.pane} aria-label={`Sensor frame ${id}`}>
      <header className={styles.head}>
        <h2 className={styles.title}>
          Frame <span className="mono">{id}</span>
        </h2>
        <button type="button" className={styles.close} onClick={onClose}>
          Close
        </button>
      </header>

      {frame.error ? (
        <ErrorBanner
          error={frame.error}
          onRetry={frame.reload}
          context={`sensor frame ${id}`}
        />
      ) : !frame.data ? (
        <Spinner label="Reading the frame…" />
      ) : (
        <FrameBody frame={frame.data} />
      )}
    </aside>
  );
}

function FrameBody({ frame }: { frame: FrameInfo }) {
  const timestamp = formatFrameTimestamp(frame.timestamp);
  const rows = poseRows(frame.pose);
  const position = poseTranslation(frame.pose);
  const slots = frameImageSlots(frame);

  return (
    <>
      <section className={styles.section}>
        <h3 className={styles.heading}>Capture</h3>
        <dl className={styles.facts}>
          {/* Absent rather than 1969: `timestamp: -1` is the contract's
              "unknown", and rendering it as a date invents a fact. */}
          <Fact label="Timestamp" value={timestamp ?? 'not recorded'} />
          {position && (
            <Fact
              label="Position"
              value={`${formatFixed(position[0])}, ${formatFixed(position[1])}, ${formatFixed(position[2])}`}
              mono
            />
          )}
          <Fact label="Depth" value={frame.has_depth ? 'stored' : 'none'} />
          <Fact label="Confidence" value={frame.has_confidence ? 'stored' : 'none'} />
          <Fact
            label="Segmentation"
            value={frame.has_segmentation ? 'stored' : 'none'}
          />
        </dl>
      </section>

      {frame.intrinsics && (
        <section className={styles.section}>
          <h3 className={styles.heading}>Intrinsics</h3>
          <dl className={styles.facts}>
            <Fact label="fx / fy" value={`${formatFixed(frame.intrinsics.fx, 2)} / ${formatFixed(frame.intrinsics.fy, 2)}`} mono />
            <Fact label="cx / cy" value={`${formatFixed(frame.intrinsics.cx, 2)} / ${formatFixed(frame.intrinsics.cy, 2)}`} mono />
            <Fact
              label="Resolution"
              value={`${frame.intrinsics.width} × ${frame.intrinsics.height}`}
              mono
            />
          </dl>
        </section>
      )}

      <section className={styles.section}>
        <h3 className={styles.heading}>Pose</h3>
        {rows === null ? (
          <p className={styles.note}>
            {formatText(undefined)} — the server did not send a 4×4 pose for this frame.
          </p>
        ) : (
          <table className={`${styles.matrix} mono`}>
            <tbody>
              {rows.map((row, index) => (
                // Row order is fixed and rows carry no id; the index is the
                // only stable key a matrix has.
                // eslint-disable-next-line react/no-array-index-key
                <tr key={index}>
                  {row.map((value, column) => (
                    // eslint-disable-next-line react/no-array-index-key
                    <td key={column}>{formatFixed(value)}</td>
                  ))}
                </tr>
              ))}
            </tbody>
          </table>
        )}
        <p className={styles.note}>World pose, row-major. Translation is the last column.</p>
      </section>

      <section className={styles.section}>
        <h3 className={styles.heading}>Images</h3>
        <div className={styles.images}>
          {slots.map((slot) => (
            <FrameImage
              key={slot.kind}
              id={frame.id}
              kind={slot.kind}
              label={slot.label}
              available={slot.available}
              normalized={slot.normalized}
            />
          ))}
        </div>
      </section>
    </>
  );
}

function Fact({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className={styles.fact}>
      <dt className={styles.factLabel}>{label}</dt>
      <dd className={`${styles.factValue} ${mono ? 'mono' : ''}`}>{value}</dd>
    </div>
  );
}

interface FrameImageProps {
  id: number;
  kind: FrameImageKind;
  label: string;
  available: boolean;
  normalized: boolean;
}

function FrameImage({ id, kind, label, available, normalized }: FrameImageProps) {
  const [failed, setFailed] = useState(false);

  return (
    <figure className={styles.figure}>
      <figcaption className={styles.figcaption}>
        <span>{label}</span>
        {normalized && available && (
          <span className={styles.badge} title="Stretched to the range this frame observed. Not a measurement.">
            normalised
          </span>
        )}
      </figcaption>
      <div className={styles.frame}>
        {!available ? (
          <span className={styles.missing}>not stored for this frame</span>
        ) : failed ? (
          <span className={styles.missing}>could not be decoded</span>
        ) : (
          <img
            className={styles.full}
            // `normalize` is what makes depth and confidence visible at all: a
            // 16-bit millimetre PNG of a 3 m room peaks at 3000 of 65535 and
            // paints black. Colour is already displayable and is left alone.
            src={api.frameImageUrl(id, kind, { normalize: normalized })}
            alt={`${label} image of sensor frame ${id}`}
            decoding="async"
            onError={() => setFailed(true)}
          />
        )}
      </div>
      {normalized && available && (
        <p className={styles.note}>
          A rendering, not the stored values — no metric scale.
        </p>
      )}
    </figure>
  );
}
