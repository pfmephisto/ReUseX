// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Link, useLocation, useNavigate } from 'react-router-dom';

import { api } from '../api/client';
import type { Sample, SampleCreate, SurveyPartPatch, SurveyType } from '../api/types';
import {
  KORTLAEGNING_PATH,
  onsiteHref,
  parseOnsiteQuery,
  rawOnsiteDel,
  sampleHref,
  surveyTypeHref,
} from '../app/links';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { appWriteChain } from '../app/writeChain';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Pill } from '../components/Pill';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { hasInstanceLink, instanceKey } from '../components/kortlaegning/EvidencePanel';
import { CaptureSheet } from '../components/onsite/CaptureSheet';
import { CaptureStage } from '../components/onsite/CaptureStage';
import { PartPicker } from '../components/onsite/PartPicker';
import { replacePart } from '../kortlaegning/model';
import { gateChanges, statusPill } from '../miljoe/model';
import {
  chipDetail,
  currentStop,
  emptyWalkText,
  partAt,
  PHOTO_TEXT,
  photoView,
  pickerGroups,
  reticleBox,
  sampleReloadFailedToast,
  sampleToast,
  stopAfter,
  typeSamples,
  unknownNotice,
  walkOrder,
  type PhotoLookup,
} from '../onsite/model';
import styles from './OnsitePage.module.css';

/** The photo's longest side: the 390px phone stage at 2x. */
const PHOTO_MAX_SIZE = 800;

/**
 * Commit a field in the sheet before leaving the part. iOS Safari does not
 * blur the note on a tap elsewhere, and the sheet is re-keyed per part, so
 * without this a typed note would unmount uncommitted; the blur commits it
 * through the queue before the navigation. Focus anywhere else — the picker —
 * is left where it is.
 */
function blurSheet(device: HTMLElement | null) {
  const active = document.activeElement;
  if (device && active instanceof HTMLElement && device.contains(active)) active.blur();
}

/**
 * On-site — the phone capture sheet (R6, R7). A walk through the stored
 * bygningsdele: the part's best photo with the reticle on its instance, and a
 * sheet that writes ★, a note and a sample against the part. Quantities,
 * metadata and approval stay in Kortlægning.
 *
 * Writes run on the app-wide chain (R11), so the next screen's first read
 * sees them. Responses are folded in through a ref mirror, so a queued write
 * reads the state its predecessor's response produced.
 */
export function OnsitePage() {
  const location = useLocation();
  const navigate = useNavigate();
  const { data, error, loading, reload } = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.survey(s), api.samples(s), api.projectSummary(s)])),
    [],
  );
  const { refresh } = useSurveyCounts();
  const toast = useToast(2600);
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });

  const [types, setTypesState] = useState<SurveyType[]>([]);
  const typesRef = useRef<SurveyType[]>([]);
  const setTypes = useCallback((next: SurveyType[]) => {
    typesRef.current = next;
    setTypesState(next);
  }, []);
  const [samples, setSamples] = useState<Sample[]>([]);
  // `data` lands one render before the effect copies it into state; until
  // then the walk is empty, which must not flash the empty state or a notice.
  const [loadedOnce, setLoadedOnce] = useState(false);
  useEffect(() => {
    if (!data) return;
    setTypes(data[0].types);
    setSamples(data[1]);
    setLoadedOnce(true);
  }, [data, setTypes]);
  const deviceRef = useRef<HTMLElement>(null);
  // The part whose sheet takes focus when it mounts: set by Videre.
  const [focusSheetFor, setFocusSheetFor] = useState<string | null>(null);

  const order = useMemo(() => walkOrder(types), [types]);
  const asked = parseOnsiteQuery(location.search);
  const stop = currentStop(order, asked);
  const at = partAt(types, stop);
  const next = stop ? stopAfter(order, stop.code) : null;
  const part = at?.part ?? null;
  const key = hasInstanceLink(part) ? instanceKey(part.cloud, part.instance_id) : null;
  // Named with the raw value, so a malformed `?del=` is said, not silently dropped.
  const notice = unknownNotice(rawOnsiteDel(location.search), stop);

  const frames = useAsync<PhotoLookup>(
    async (signal) => {
      if (!hasInstanceLink(part)) return { key: '', frames: [], failed: false };
      const k = instanceKey(part.cloud, part.instance_id);
      try {
        return {
          key: k,
          frames: await api.instanceFrames(part.cloud, part.instance_id, signal),
          failed: false,
        };
      } catch (cause) {
        if (signal.aborted) throw cause; // a superseded lookup; useAsync drops it
        return { key: k, frames: [], failed: true };
      }
    },
    [key],
  );
  const photo = photoView(key, frames.data);

  // Walking replaces the entry: Back leaves On-site instead of retracing parts.
  const go = (code: string) => {
    blurSheet(deviceRef.current);
    navigate(onsiteHref(code), { replace: true });
  };

  const patchPart = (code: string, patch: SurveyPartPatch) =>
    mutate(async () => {
      const updated = await api.patchSurveyPart(code, patch);
      setTypes(replacePart(typesRef.current, updated));
    });

  const register = (body: SampleCreate, done: () => void) =>
    mutate(async () => {
      const before = typesRef.current;
      const created = await api.createSample(body);
      // The sample exists from here on: close the form even if the re-read fails.
      done();
      const partCode = created.part_code ?? body.part_code ?? '';
      try {
        const [survey, list] = await Promise.all([api.survey(), api.samples()]);
        setTypes(survey.types);
        setSamples(list);
        toast.show(sampleToast(created.code, partCode, gateChanges(before, survey.types)));
      } catch {
        setSamples((prev) => [...prev, created]);
        toast.show(sampleReloadFailedToast(created.code, partCode));
      }
    });

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="bygningsdelene" />
      </div>
    );
  }
  if ((loading && !data) || (data && !loadedOnce)) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser bygningsdele…" />
      </div>
    );
  }
  if (!data) return null;

  if (!at || !stop) {
    const empty = emptyWalkText(types);
    return (
      <div className={styles.page}>
        <h1 className={styles.title}>On-site</h1>
        {notice && <p className={styles.notice}>{notice}</p>}
        <EmptyState
          title={empty.title}
          detail={empty.detail}
          action={
            <Link className={styles.crossLink} to={KORTLAEGNING_PATH}>
              Åbn Kortlægning
            </Link>
          }
        />
      </div>
    );
  }

  const { type } = at;
  const frameSize = data[2].sensor_frames;
  const aspect = frameSize.width && frameSize.height ? { width: frameSize.width, height: frameSize.height } : null;
  const photoUrl =
    photo.kind === 'photo'
      ? api.frameImageUrl(photo.frame.frame_id, 'color', {
          maxSize: PHOTO_MAX_SIZE,
        })
      : null;
  const onType = typeSamples(type, at.part, samples);

  return (
    <div className={styles.page}>
      <h1 className={styles.title}>On-site</h1>
      {notice && <p className={styles.notice}>{notice}</p>}

      {/* The sheet first, as on the phone; the jump list below it, or beside it on a wide screen. */}
      <div className={styles.layout}>
        <div className={styles.main}>
          <section ref={deviceRef} className={styles.device} aria-label={`Bygningsdel ${at.part.code}`}>
            <CaptureStage
              photoUrl={photoUrl}
              placeholder={photo.kind === 'photo' ? '' : PHOTO_TEXT[photo.kind]}
              reticle={photo.kind === 'photo' ? reticleBox(photo.frame, frameSize) : null}
              title={type.name}
              detail={chipDetail(type, at.part)}
              code={at.part.code}
              aspect={aspect}
            />
            <CaptureSheet
              key={at.part.code}
              part={at.part}
              busy={busy}
              next={next}
              onStar={() => patchPart(at.part.code, { starred: !at.part.starred })}
              onNote={(note) => patchPart(at.part.code, { note })}
              onRegister={register}
              focusOnMount={focusSheetFor === at.part.code}
              onNext={() => {
                if (!next) return;
                setFocusSheetFor(next.code);
                go(next.code);
              }}
            />
          </section>

          <section className={styles.samples} aria-labelledby="onsite-proever">
            <h2 id="onsite-proever" className={styles.samplesHeading}>
              Prøver på typen
            </h2>
            {onType.length === 0 ? (
              <p className={styles.muted}>Ingen prøver på {type.name} endnu.</p>
            ) : (
              <ul className={styles.sampleList}>
                {onType.map(({ sample, here }) => {
                  const pill = statusPill(sample);
                  return (
                    <li key={sample.id} className={styles.sampleRow}>
                      <Link className={styles.crossLink} to={sampleHref(sample.id)}>
                        {sample.code} · {sample.title}
                      </Link>
                      {here && <span className={styles.muted}>· her</span>}
                      <Pill tone={pill.tone}>{pill.label}</Pill>
                    </li>
                  );
                })}
              </ul>
            )}
          </section>

          <p className={styles.footnote}>
            Mængder, metadata og godkendelse venter til gennemsynet i Kortlægning.{' '}
            <Link className={styles.crossLink} to={surveyTypeHref(type.id)}>
              Åbn {type.name} i Kortlægning
            </Link>
          </p>
        </div>
        <aside className={styles.side}>
          <PartPicker groups={pickerGroups(order, types)} value={stop.code} onChange={go} />
        </aside>
      </div>
      <Toast message={toast.message} />
    </div>
  );
}
