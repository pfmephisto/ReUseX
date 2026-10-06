// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, VisibleFrame } from '../api/types';
import { photoBatchKey, photoCountText, photoStripModel, rowThumbFrame } from '../kortlaegning/photo';

function part(code: string, instance_id: number | null, cloud: string | null = 'instances'): SurveyPart {
  return {
    code,
    type_id: 1,
    cloud,
    instance_id,
    room_id: null,
    room_name: '',
    quantity: 1,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: null,
    orphaned: false,
  };
}

function frames(n: number): VisibleFrame[] {
  return Array.from({ length: n }, (_, i) => ({ frame_id: i + 1, centrality: 0, score: 1, depth: 1, u: 0, v: 0 }));
}

describe('photoCountText', () => {
  it('says "n fotos", singular for one, nothing before the batch resolves', () => {
    expect(photoCountText({ count: 6, best_frame_id: 3 })).toBe('6 fotos');
    expect(photoCountText({ count: 1, best_frame_id: 3 })).toBe('1 foto');
    expect(photoCountText({ count: 0, best_frame_id: null })).toBe('0 fotos');
    expect(photoCountText(undefined)).toBe('');
  });
});

describe('rowThumbFrame', () => {
  const photos = {
    'RX-001': { count: 0, best_frame_id: null },
    'RX-002': { count: 4, best_frame_id: 42 },
    'RX-003': { count: 2, best_frame_id: 7 },
  };

  it('a part row uses its own best frame', () => {
    expect(rowThumbFrame([{ code: 'RX-003' }], photos)).toBe(7);
    expect(rowThumbFrame([{ code: 'RX-001' }], photos)).toBeNull();
  });

  it('a type row uses its first part that has a photo', () => {
    expect(rowThumbFrame([{ code: 'RX-001' }, { code: 'RX-002' }, { code: 'RX-003' }], photos)).toBe(42);
    expect(rowThumbFrame([{ code: 'RX-009' }], photos)).toBeNull();
  });

  it('is a placeholder until the batch resolves', () => {
    expect(rowThumbFrame([{ code: 'RX-002' }], null)).toBeNull();
  });
});

describe('photoBatchKey', () => {
  it('changes with the linked parts, not with unlinked ones or order', () => {
    const a = photoBatchKey([{ parts: [part('RX-002', 2), part('RX-001', 1), part('RX-003', null, null)] }]);
    const b = photoBatchKey([{ parts: [part('RX-001', 1)] }, { parts: [part('RX-002', 2)] }]);
    expect(a).toBe(b);
    expect(a).toBe('RX-001=instances/1,RX-002=instances/2');
    expect(photoBatchKey([{ parts: [part('RX-001', 1), part('RX-004', 4)] }])).not.toBe(a);
  });

  it('is empty with no linked part (nothing to fetch)', () => {
    expect(photoBatchKey([{ parts: [part('RX-001', null, null), part('RX-002', 0)] }])).toBe('');
    expect(photoBatchKey([])).toBe('');
  });
});

describe('photoStripModel', () => {
  it('names why there is no strip', () => {
    expect(photoStripModel(null, undefined).message).toBe('Ingen fotos — bygningsdelen er ikke koblet til en instans.');
    expect(photoStripModel('instances/1', undefined).message).toBe('Indlæser fotos…');
    expect(photoStripModel('instances/1', { key: 'instances/1', frames: [], failed: true }).message).toBe(
      'Fotos kunne ikke hentes.',
    );
    const none = photoStripModel('instances/1', { key: 'instances/1', frames: [], failed: false });
    expect(none).toEqual({ strip: null, count: 0, message: 'Ingen fotos fundet for denne instans.' });
  });

  it('treats another key\'s frames as loading', () => {
    const stale = photoStripModel('instances/2', { key: 'instances/1', frames: frames(3), failed: false });
    expect(stale).toEqual({ strip: null, count: null, message: 'Indlæser fotos…' });
  });

  it('builds the strip with the count and +n overflow', () => {
    const m = photoStripModel('instances/1', { key: 'instances/1', frames: frames(8), failed: false });
    expect(m.count).toBe(8);
    expect(m.message).toBeNull();
    expect(m.strip?.visible.map((f) => f.frame_id)).toEqual([1, 2, 3, 4, 5]);
    expect(m.strip?.overflow).toBe(3);
  });
});
