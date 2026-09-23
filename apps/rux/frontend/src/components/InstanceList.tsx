// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';
import { useNavigate } from 'react-router-dom';

import { api } from '../api/client';
import type { CloudInfo, InstanceInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { describeWriteFailure, type WriteFailure } from '../data/writeState';
import { EmptyState } from './EmptyState';
import { ErrorBanner } from './ErrorBanner';
import { Spinner } from './Spinner';
import { WriteBanner } from './WriteBanner';
import styles from './InstanceList.module.css';

/**
 * Instance rows for a cloud, each with a "Create material" button.
 *
 * The flow when the button is pressed:
 *   1. `POST /materials` — mint a blank passport, get a guid.
 *   2. `PUT /instances/{cloud}/{id}/material` — link the passport.
 *   3. `GET /instances/{cloud}/{id}/frames` — pick the top-ranked frame
 *      (best source image for the instance centroid).
 *   4. `GET /frames/{frame_id}/image` → `PUT /materials/{guid}/thumbnail`
 *      — prefill the thumbnail.  Best-effort: skipped silently when no
 *      frames exist (instance has no visible sensor frames).
 *   5. Navigate to `/materials`.
 *
 * Failures in steps 1–2 surface through `WriteBanner`; steps 3–4 are
 * best-effort and never block navigation.
 */
export function InstanceList({ cloud }: { cloud: string }) {
  const navigate = useNavigate();
  const rows = useAsync((signal) => api.instances(cloud, signal), [cloud]);
  const [failure, setFailure] = useState<WriteFailure | null>(null);
  const [creating, setCreating] = useState<number | null>(null);

  const handleCreateMaterial = useCallback(
    async (instance: InstanceInfo) => {
      setCreating(instance.instance_id);
      setFailure(null);
      try {
        // 1. Mint a blank material passport.
        const passport = await api.createMaterial();
        const guid = passport.guid;

        // 2. Link the instance to the new passport.
        await api.linkInstanceMaterial(cloud, instance.instance_id, guid);

        // 3–4. Prefill thumbnail from the top-ranked frame (best-effort).
        try {
          const frames = await api.instanceFrames(cloud, instance.instance_id);
          if (frames.length > 0) {
            const frameId = frames[0].frame_id;
            const imageUrl = api.frameImageUrl(frameId, 'color');
            const imageResp = await fetch(imageUrl);
            if (imageResp.ok) {
              const blob = await imageResp.blob();
              const file = new File([blob], `frame-${frameId}.jpg`, {
                type: blob.type || 'image/jpeg',
              });
              await api.uploadThumbnail(guid, file);
            }
          }
        } catch {
          // thumbnail prefill is best-effort — never block navigation
        }

        // 5. Navigate to the materials page.
        navigate('/materials');
      } catch (err) {
        setFailure(describeWriteFailure(err as Error, 'the material passport'));
      } finally {
        setCreating(null);
      }
    },
    [cloud, navigate],
  );

  if (rows.error) {
    return <ErrorBanner error={rows.error} onRetry={rows.reload} context="the instance list" />;
  }
  if (!rows.data) return <Spinner label="Loading instances…" />;
  if (rows.data.length === 0) {
    return (
      <EmptyState
        title="No instances"
        detail="`rux create instances` produces the instance rows shown here."
      />
    );
  }

  return (
    <div className={styles.root}>
      {failure && (
        <WriteBanner
          failure={failure}
          onDismiss={() => setFailure(null)}
          onRetry={failure.retryable ? () => setFailure(null) : undefined}
        />
      )}
      <table className={styles.table}>
        <thead>
          <tr>
            <th className={styles.th}>ID</th>
            <th className={styles.th}>GUID</th>
            <th className={styles.th}>Class</th>
            <th className={styles.th}>Points</th>
            <th className={styles.th}>Material</th>
            <th className={styles.th}></th>
          </tr>
        </thead>
        <tbody>
          {rows.data.map((inst) => (
            <tr key={inst.instance_id} className={styles.row}>
              <td className={styles.td}>{inst.instance_id}</td>
              <td className={styles.td}>{inst.guid}</td>
              <td className={styles.td}>{inst.semantic_class}</td>
              <td className={styles.td}>{inst.point_count.toLocaleString()}</td>
              <td className={styles.td}>
                {inst.material_guid ? (
                  <span className={styles.linked} title={inst.material_guid}>
                    linked
                  </span>
                ) : (
                  <span className={styles.unlinked}>—</span>
                )}
              </td>
              <td className={styles.td}>
                {!inst.material_guid && (
                  <button
                    type="button"
                    className={styles.createBtn}
                    disabled={creating === inst.instance_id}
                    onClick={() => handleCreateMaterial(inst)}
                  >
                    {creating === inst.instance_id ? 'Creating…' : 'Create material'}
                  </button>
                )}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

/**
 * Cloud selector + instance list for the Instances page.
 *
 * Reads all Label clouds, filters to those classified as semantic (i.e. not
 * `planes`/`rooms`), and lets the user pick which one to browse.
 */
export function InstancePanel() {
  const clouds = useAsync((signal) => api.clouds(signal), []);
  const [selected, setSelected] = useState<string | null>(null);

  const instanceClouds = (clouds.data ?? []).filter(
    (c: CloudInfo) => c.type === 'Label' && !['planes', 'rooms'].includes(c.name),
  );

  const active = selected ?? instanceClouds[0]?.name ?? null;

  if (clouds.error) {
    return (
      <ErrorBanner error={clouds.error} onRetry={clouds.reload} context="the cloud list" />
    );
  }
  if (!clouds.data) return <Spinner label="Reading clouds…" />;

  if (instanceClouds.length === 0) {
    return (
      <EmptyState
        title="No instance clouds"
        detail="`rux create instances` produces the instance clouds shown here."
      />
    );
  }

  return (
    <div className={styles.panel}>
      {instanceClouds.length > 1 && (
        <div className={styles.toolbar}>
          <label className={styles.cloudLabel} htmlFor="instance-cloud">
            Cloud
          </label>
          <select
            id="instance-cloud"
            className={styles.cloudSelect}
            value={active ?? ''}
            onChange={(e) => setSelected(e.target.value || null)}
          >
            {instanceClouds.map((c) => (
              <option key={c.name} value={c.name}>
                {c.name}
              </option>
            ))}
          </select>
        </div>
      )}
      {active && <InstanceList key={active} cloud={active} />}
    </div>
  );
}
