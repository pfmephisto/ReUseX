// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's evidence panel: the Plan / 360° / Foto / Punktsky / Rum views
 * that back a survey selection (keys 1–5). Server-rendered images
 * (`GET /api/v1/renders`), the nearest 360° panorama as a pannable strip
 * centred on the part (`GET /instances/{cloud}/{id}/panoramas`, spec A5) and
 * the selected building part's best sensor frame.
 *
 * Used in two shapes, picked by `variant`:
 *  - `'panel'` — the right-column panel on the Kortlægning page: a tab strip
 *    and one image with a caption row below it.
 *  - `'stage'` — the edit dialog's evidence block: a 5-up thumbnail grid above
 *    a large stage image.
 *
 * `evidenceSources` is pure and exported so the dialog's thumbnail grid can
 * compute the same five sources without re-deriving the logic. It takes the
 * Foto tab's resolved best-frame id as an *optional third argument* (plus a
 * fourth `photoFailed` flag) rather than reaching into `api.instanceFrames`
 * itself — that lookup is async and callers already run their own `useAsync`
 * for it, keyed on the *resolved highlight part* (`part`, or for a
 * type-level selection the type's first linked part; see
 * `resolveHighlightPart` below). Pass:
 *  - `photoFrameId: undefined` — the lookup is in flight, hasn't started, or
 *    (inside `EvidencePanel`) its last-known result is for a *different*
 *    highlight — `useAsync` keeps stale `data`/`error` around across a deps
 *    change until the new request settles, so a consumer must gate on a key
 *    match, not just on `loading`, or it shows the previous part's photo (or
 *    its error) captioned as the new part's for a render or two.
 *  - `photoFrameId: null` — the lookup finished (for the *current* highlight)
 *    and found no frame.
 *  - `photoFrameId: <number>` — the resolved frame id.
 *  - `photoFailed: true` — the lookup itself failed (for the current
 *    highlight) rather than returning zero frames; shown as a distinct
 *    message from "no frame found".
 * The 360° tab takes the resolved panorama the same way, as an optional fifth
 * argument (`resolvePano` in `kortlaegning/pano.ts`); omitted = still loading.
 */

import { useEffect, useState } from 'react';
import { Link } from 'react-router-dom';

import { api } from '../../api/client';
import type { InstancePanorama, SurveyPart, SurveyType } from '../../api/types';
import { viewportPanoramaHref } from '../../app/links';
import { useAsync } from '../../app/useAsync';
import type { EvidenceTab } from '../../kortlaegning/keys';
import { partLabel } from '../../kortlaegning/model';
import { PANO_STRIP_MAX_SIZE, PANO_TEXT, type PanoLookup, panoCaption, resolvePano } from '../../kortlaegning/pano';
import { type FrameLookup, hasInstanceLink, instanceKey, resolvePhotoState } from '../../kortlaegning/photo';
import { PHOTO_EMPTY_TEXT } from '../../kortlaegning/vocab';
import { EmptyState } from '../EmptyState';
import styles from './EvidencePanel.module.css';
import { PanoStrip } from './PanoStrip';

export interface EvidencePanelProps {
  type: SurveyType | null;
  part: SurveyPart | null;
  tab: EvidenceTab;
  onTab: (t: EvidenceTab) => void;
  /** 'panel' (right column, 5 tabs) or 'stage' (dialog: thumbnails + large stage). */
  variant: 'panel' | 'stage';
}

export interface EvidenceSource {
  tab: EvidenceTab;
  label: string;
  caption: string;
  url: string | null;
  empty: string;
  /** 360° only: the panorama, and where the part is in it (u, v in 0..1). */
  pano?: { id: number; u: number; v: number; marker: boolean };
  /** 360° only: "Åbn i viewport" (`/viewport?pano=<id>`). */
  href?: string;
}

const RENDER_SIZE = { width: 640, height: 480 };

const GENERIC_IMAGE_ERROR =
  'Billedet kunne ikke tegnes — serveren mangler måske 3D-visning, eller projektet mangler data til denne visning.';

const EVIDENCE_LABEL: Record<EvidenceTab, string> = {
  plan: 'Plan',
  pano: '360°',
  foto: 'Foto',
  punktsky: 'Punktsky',
  rum: 'Rum',
};

/**
 * The part whose instance drives the Plan/Punktsky highlight and the Foto
 * lookup: the selected part itself, or — for a type-level selection — the
 * type's first part that is linked to an instance.
 */
export function resolveHighlightPart(type: SurveyType | null, part: SurveyPart | null): SurveyPart | null {
  if (part) return part;
  return type?.parts.find((p) => hasInstanceLink(p)) ?? null;
}

/**
 * The five evidence sources for a selection, in `EVIDENCE_TABS` order. See
 * the module doc for `photoFrameId` / `photoFailed` / `pano`.
 */
export function evidenceSources(
  type: SurveyType | null,
  part: SurveyPart | null,
  photoFrameId?: number | null,
  photoFailed?: boolean,
  pano?: { pano: InstancePanorama | null; empty: string },
): EvidenceSource[] {
  const highlight = resolveHighlightPart(type, part);
  const linked = hasInstanceLink(highlight);
  const highlightQuery = linked
    ? { highlight_instance: highlight.instance_id, highlight_cloud: highlight.cloud }
    : {};

  let fotoUrl: string | null = null;
  let fotoCaption = 'Bedste foto';
  let fotoEmpty: string = PHOTO_EMPTY_TEXT.unlinked;
  if (linked) {
    if (photoFailed) {
      fotoEmpty = PHOTO_EMPTY_TEXT.failed;
    } else if (photoFrameId === undefined) {
      fotoEmpty = PHOTO_EMPTY_TEXT.loading;
    } else if (photoFrameId === null) {
      fotoEmpty = PHOTO_EMPTY_TEXT.none;
    } else {
      fotoUrl = api.frameImageUrl(photoFrameId, 'color', { maxSize: 960 });
      fotoCaption = `Bedste foto · ramme ${photoFrameId}`;
      fotoEmpty = GENERIC_IMAGE_ERROR;
    }
  }

  const panoSource: EvidenceSource = {
    tab: 'pano',
    label: EVIDENCE_LABEL.pano,
    caption: panoCaption(highlight?.room_name),
    url: null,
    empty: linked ? (pano?.empty ?? PANO_TEXT.loading) : PANO_TEXT.unlinked,
  };
  if (linked && pano?.pano) {
    const p = pano.pano;
    panoSource.url = api.panoramaImageUrl(p.panorama_id, { maxSize: PANO_STRIP_MAX_SIZE });
    panoSource.caption = panoCaption(highlight.room_name, p.heading);
    panoSource.empty = GENERIC_IMAGE_ERROR;
    panoSource.pano = { id: p.panorama_id, u: p.u, v: p.v, marker: p.heading === 'resected' };
    panoSource.href = viewportPanoramaHref(p.panorama_id);
  }

  return [
    {
      tab: 'plan',
      label: EVIDENCE_LABEL.plan,
      caption: 'Stueplan · snit i 1,2 m',
      url: api.renderUrl({ view: 'plan', layers: ['cloud'], ...RENDER_SIZE, ...highlightQuery }),
      empty: GENERIC_IMAGE_ERROR,
    },
    panoSource,
    {
      tab: 'foto',
      label: EVIDENCE_LABEL.foto,
      caption: fotoCaption,
      url: fotoUrl,
      empty: fotoEmpty,
    },
    {
      tab: 'punktsky',
      label: EVIDENCE_LABEL.punktsky,
      caption: 'Punktsky · bygningsdel markeret',
      url: api.renderUrl({
        view: 'orbit',
        orbit_index: 1,
        layers: ['cloud'],
        ...RENDER_SIZE,
        ...highlightQuery,
      }),
      empty: GENERIC_IMAGE_ERROR,
    },
    {
      tab: 'rum',
      label: EVIDENCE_LABEL.rum,
      caption: 'Rumvis model · segmenterede rum',
      url: api.renderUrl({ view: 'orbit', orbit_index: 1, layers: ['rooms'], ...RENDER_SIZE }),
      empty: GENERIC_IMAGE_ERROR,
    },
  ];
}

/**
 * One evidence image: tries `source.url`, falls back to `source.empty` when
 * there is no URL at all, and to `source.empty` + the generic line when the
 * `<img>` itself fails to load. Resets its error flag whenever the URL
 * changes, so a broken render for one part doesn't carry over and hide the
 * next part's (perfectly fine) render.
 */
function EvidenceImage({
  source,
  wellClassName,
  compact,
}: {
  source: EvidenceSource;
  wellClassName: string;
  /** Thumbnail-sized well: too small for the full EmptyState block. */
  compact?: boolean;
}) {
  const [errored, setErrored] = useState(false);

  useEffect(() => {
    setErrored(false);
  }, [source.url]);

  if (!source.url || errored) {
    const message =
      source.url && source.empty !== GENERIC_IMAGE_ERROR
        ? `${source.empty} ${GENERIC_IMAGE_ERROR}`
        : source.empty;
    if (compact) {
      return (
        <div className={wellClassName} title={message}>
          <span className={styles.thumbEmpty}>Intet billede</span>
        </div>
      );
    }
    return (
      <div className={wellClassName}>
        <EmptyState title={message} bare />
      </div>
    );
  }

  if (source.pano && !compact) {
    return (
      <div className={wellClassName}>
        <PanoStrip
          url={source.url}
          u={source.pano.u}
          v={source.pano.v}
          marker={source.pano.marker}
          label={`360°-optagelse ${source.pano.id}`}
          onError={() => setErrored(true)}
        />
      </div>
    );
  }

  return (
    <div className={wellClassName}>
      <img
        src={source.url}
        // A thumbnail's caption strip already names the source ("1 · Plan"
        // right below it) — a non-empty alt would have a screen reader say
        // the label twice. The large stage/panel image has no adjacent label
        // repeating it, so it keeps the real alt text.
        alt={compact ? '' : source.label}
        className={styles.image}
        onError={() => setErrored(true)}
      />
    </div>
  );
}

/** Moved to `kortlaegning/photo` (pure); re-exported so existing imports keep working. */
export { type FrameLookup, hasInstanceLink, instanceKey, resolvePhotoState } from '../../kortlaegning/photo';

export function EvidencePanel({ type, part, tab, onTab, variant }: EvidencePanelProps) {
  const highlight = resolveHighlightPart(type, part);
  const linked = hasInstanceLink(highlight);
  const currentKey = linked ? instanceKey(highlight.cloud, highlight.instance_id) : null;

  const frames = useAsync<FrameLookup>(
    async (signal) => {
      if (!linked) return { key: '', frames: [], failed: false };
      const key = instanceKey(highlight.cloud, highlight.instance_id);
      try {
        const result = await api.instanceFrames(highlight.cloud, highlight.instance_id, signal);
        return { key, frames: result, failed: false };
      } catch (cause) {
        // A superseded request's abort still propagates here; let useAsync's
        // own `controller.signal.aborted` guard swallow it as usual rather
        // than reporting a stale key as a genuine failure.
        if (signal.aborted) throw cause;
        return { key, frames: [], failed: true };
      }
    },
    [highlight?.cloud, highlight?.instance_id],
  );

  const panoramas = useAsync<PanoLookup>(
    async (signal) => {
      if (!linked) return { key: '', panoramas: [], failed: false };
      const key = instanceKey(highlight.cloud, highlight.instance_id);
      try {
        const result = await api.instancePanoramas(highlight.cloud, highlight.instance_id, signal);
        return { key, panoramas: result.panoramas, failed: false };
      } catch (cause) {
        if (signal.aborted) throw cause;
        return { key, panoramas: [], failed: true };
      }
    },
    [highlight?.cloud, highlight?.instance_id],
  );

  const { photoFrameId, photoFailed } = resolvePhotoState(currentKey, frames.data);
  const sources = evidenceSources(type, part, photoFrameId, photoFailed, resolvePano(currentKey, panoramas.data));

  if (!type) {
    return (
      <div className={variant === 'panel' ? styles.panel : styles.stageWrap}>
        <EmptyState title="Vælg en række for evidens." />
      </div>
    );
  }

  const active = sources.find((s) => s.tab === tab) ?? sources[0];
  const selectionLabel = part ? partLabel(part) : type.name;

  if (variant === 'panel') {
    return (
      <section className={styles.panel}>
        <div className={styles.tabs} role="tablist" aria-label="Evidens">
          {sources.map((s) => (
            <button
              key={s.tab}
              type="button"
              role="tab"
              aria-selected={s.tab === tab}
              className={styles.tab}
              onClick={() => onTab(s.tab)}
            >
              {s.label}
            </button>
          ))}
        </div>
        <EvidenceImage source={active} wellClassName={styles.well} />
        <div className={styles.captionRow}>
          <span>{active.caption}</span>
          <span className={styles.selectionLabel}>{selectionLabel}</span>
        </div>
        {active.href && (
          <Link className={styles.openLink} to={active.href}>
            Åbn i viewport →
          </Link>
        )}
      </section>
    );
  }

  return (
    <section className={styles.stageWrap}>
      <div className={styles.thumbGrid}>
        {sources.map((s, index) => (
          <button
            key={s.tab}
            type="button"
            className={styles.thumbButton}
            data-active={s.tab === tab || undefined}
            aria-current={s.tab === tab || undefined}
            onClick={() => onTab(s.tab)}
          >
            <EvidenceImage source={s} wellClassName={styles.thumbWell} compact />
            <span className={styles.thumbCaption}>
              {index + 1} · {s.label}
            </span>
          </button>
        ))}
      </div>
      <div className={styles.stage}>
        <EvidenceImage source={active} wellClassName={styles.stageWell} />
        <span className={styles.tagline}>{active.caption}</span>
        {active.href && (
          <Link className={styles.stageLink} to={active.href}>
            Åbn i viewport →
          </Link>
        )}
      </div>
    </section>
  );
}
