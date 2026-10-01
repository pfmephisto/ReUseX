// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's evidence panel: the Plan / Foto / Punktsky / Rum-model views
 * that back a survey selection. Server-rendered images (`GET /api/v1/renders`)
 * plus the selected building part's best sensor frame.
 *
 * Used in two shapes, picked by `variant`:
 *  - `'panel'` — the right-column panel on the Kortlægning page: a tab strip
 *    and one image with a caption row below it.
 *  - `'stage'` — the edit dialog's evidence block: a 4-up thumbnail grid above
 *    a large stage image.
 *
 * `evidenceSources` is pure and exported so the dialog's thumbnail grid can
 * compute the same four sources without re-deriving the logic. It takes the
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
 */

import { useEffect, useState } from 'react';

import { api } from '../../api/client';
import type { SurveyPart, SurveyType, VisibleFrame } from '../../api/types';
import { useAsync } from '../../app/useAsync';
import type { EvidenceTab } from '../../kortlaegning/keys';
import { EmptyState } from '../EmptyState';
import styles from './EvidencePanel.module.css';

export interface EvidencePanelProps {
  type: SurveyType | null;
  part: SurveyPart | null;
  tab: EvidenceTab;
  onTab: (t: EvidenceTab) => void;
  /** 'panel' (right column, 4 tabs) or 'stage' (dialog: thumbnails + large stage). */
  variant: 'panel' | 'stage';
}

export interface EvidenceSource {
  tab: EvidenceTab;
  label: string;
  caption: string;
  url: string | null;
  empty: string;
}

const RENDER_SIZE = { width: 640, height: 480 };

const GENERIC_IMAGE_ERROR =
  'Billedet kunne ikke tegnes — serveren mangler måske 3D-visning, eller projektet mangler data til denne visning.';

const EVIDENCE_LABEL: Record<EvidenceTab, string> = {
  plan: 'Plan',
  foto: 'Foto',
  punktsky: 'Punktsky',
  rum: 'Rum-model',
};

/**
 * A part counts as linked when it names a cloud and an instance id — and that
 * id is `>= 1`: label `0` means unlabeled (STANDARDS §3), so an `instance_id`
 * of 0 is not a real instance and both `/renders` and the frames lookup would
 * 400 on it.
 */
export function hasInstanceLink(part: SurveyPart | null): part is SurveyPart & { cloud: string; instance_id: number } {
  return !!part && !!part.cloud && part.instance_id !== null && part.instance_id >= 1;
}

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
 * The four evidence sources for a selection, in `EVIDENCE_TABS` order. See
 * the module doc for `photoFrameId` / `photoFailed`.
 */
export function evidenceSources(
  type: SurveyType | null,
  part: SurveyPart | null,
  photoFrameId?: number | null,
  photoFailed?: boolean,
): EvidenceSource[] {
  const highlight = resolveHighlightPart(type, part);
  const linked = hasInstanceLink(highlight);
  const highlightQuery = linked
    ? { highlight_instance: highlight.instance_id, highlight_cloud: highlight.cloud }
    : {};

  let fotoUrl: string | null = null;
  let fotoCaption = 'Bedste foto';
  let fotoEmpty = 'Ingen foto — bygningsdelen er ikke koblet til en instans.';
  if (linked) {
    if (photoFailed) {
      fotoEmpty = 'Foto kunne ikke hentes.';
    } else if (photoFrameId === undefined) {
      fotoEmpty = 'Indlæser foto…';
    } else if (photoFrameId === null) {
      fotoEmpty = 'Ingen foto — der blev ikke fundet en ramme for denne instans.';
    } else {
      fotoUrl = api.frameImageUrl(photoFrameId, 'color', { maxSize: 960 });
      fotoCaption = `Bedste foto · ramme ${photoFrameId}`;
      fotoEmpty = GENERIC_IMAGE_ERROR;
    }
  }

  return [
    {
      tab: 'plan',
      label: EVIDENCE_LABEL.plan,
      caption: 'Stueplan · snit i 1,2 m',
      url: api.renderUrl({ view: 'plan', layers: ['cloud'], ...RENDER_SIZE, ...highlightQuery }),
      empty: GENERIC_IMAGE_ERROR,
    },
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
        <EmptyState title={message} />
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

/** Identifies which highlight a frame lookup's result belongs to. */
export function instanceKey(cloud: string, instanceId: number): string {
  return `${cloud}/${instanceId}`;
}

/**
 * A frame lookup's result, tagged with the highlight it was fetched for.
 *
 * `useAsync` keeps the previous `data` (and does not clear a previous
 * `error`) across a deps change until the new request settles — and `loading`
 * only flips to `true` inside the effect, so there is one render, right after
 * the selected part changes, where `loading` is still `false` and `data`
 * still holds the *old* part's result. Tagging the result with its own key
 * and comparing against the key computed fresh on every render (from props,
 * not from hook state) is what catches that render, not just the async race.
 */
export interface FrameLookup {
  key: string;
  frames: VisibleFrame[];
  /** True when the request for `key` itself failed (not just "zero frames"). */
  failed: boolean;
}

/**
 * Turns a (possibly stale) `FrameLookup` into the Foto tab's `photoFrameId`
 * / `photoFailed` input for `evidenceSources`. Exported and pure — kept
 * separate from `EvidencePanel` itself — so the "a different key's data (or
 * error) is never shown as the current part's" rule is unit-testable without
 * a DOM.
 *
 * `data` not matching `currentKey` (including `data` not having arrived yet)
 * is treated exactly like "still loading": `undefined`/not-failed. There is
 * deliberately no way to distinguish "loading" from "stale" in the output —
 * both must render as "Indlæser foto…", never as the previous part's photo
 * or error.
 */
export function resolvePhotoState(
  currentKey: string | null,
  data: FrameLookup | undefined,
): { photoFrameId: number | null | undefined; photoFailed: boolean } {
  if (!data || data.key !== currentKey) return { photoFrameId: undefined, photoFailed: false };
  if (data.failed) return { photoFrameId: undefined, photoFailed: true };
  return { photoFrameId: data.frames[0]?.frame_id ?? null, photoFailed: false };
}

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

  const { photoFrameId, photoFailed } = resolvePhotoState(currentKey, frames.data);
  const sources = evidenceSources(type, part, photoFrameId, photoFailed);

  if (!type) {
    return (
      <div className={variant === 'panel' ? styles.panel : styles.stageWrap}>
        <EmptyState title="Vælg en række for evidens." />
      </div>
    );
  }

  const active = sources.find((s) => s.tab === tab) ?? sources[0];
  const selectionLabel = part ? `${part.code} · ${part.room_name}` : type.name;

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
      </div>
    </section>
  );
}
