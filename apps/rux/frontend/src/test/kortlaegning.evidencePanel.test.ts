// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import {
  evidenceSources,
  resolvePhotoState,
  type EvidenceSource,
  type FrameLookup,
} from '../components/kortlaegning/EvidencePanel';
import { EVIDENCE_TABS, type EvidenceTab } from '../kortlaegning/keys';
import { PANO_TEXT } from '../kortlaegning/pano';

function tabOf(sources: EvidenceSource[], tab: EvidenceTab): EvidenceSource {
  const found = sources.find((s) => s.tab === tab);
  if (!found) throw new Error(`no ${tab} source`);
  return found;
}

function frame(overrides: Partial<VisibleFrame> = {}): VisibleFrame {
  return { frame_id: 1, centrality: 0, score: 1, depth: 1, u: 0, v: 0, ...overrides };
}

function part(overrides: Partial<SurveyPart> = {}): SurveyPart {
  return {
    code: 'RX-001',
    type_id: 1,
    cloud: 'instances',
    instance_id: 7,
    room_id: 1,
    room_name: 'Office Zone',
    quantity: 1,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: 'guid-1',
    orphaned: false,
    ...overrides,
  };
}

function type(overrides: Partial<SurveyType> = {}, parts: SurveyPart[] = [part()]): SurveyType {
  return {
    id: 1,
    name: 'Betonsøjler, bærende',
    eak_code: '17.01.01',
    eak_name: 'Beton',
    bim7aa_code: '221 Bærende konstr.',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.82,
    mass_t: 58,
    note: '',
    starred: false,
    semantic_class: 1,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 24,
    parts,
    created_at: '',
    updated_at: '',
    ...overrides,
  };
}

describe('kortlægning evidenceSources', () => {
  it('returns the five sources in EVIDENCE_TABS order', () => {
    const sources = evidenceSources(null, null);
    expect(sources.map((s) => s.tab)).toEqual(EVIDENCE_TABS);
    expect(sources.map((s) => s.label)).toEqual(['Plan', '360°', 'Foto', 'Punktsky', 'Rum']);
  });

  it('carries fixed captions for Plan, Punktsky and Rum', () => {
    const sources = evidenceSources(type(), part());
    const [plan, punktsky, rum] = [tabOf(sources, 'plan'), tabOf(sources, 'punktsky'), tabOf(sources, 'rum')];
    expect(plan.caption).toBe('Stueplan · snit i 1,2 m');
    expect(punktsky.caption).toBe('Punktsky · bygningsdel markeret');
    expect(rum.caption).toBe('Rumvis model · segmenterede rum');
  });

  it('builds the Rum render with no highlight — it is never part-specific', () => {
    const rum = tabOf(evidenceSources(type(), part()), 'rum');
    expect(rum.url).not.toBeNull();
    expect(rum.url).not.toContain('highlight_instance');
    expect(rum.url).toContain('view=orbit');
    expect(rum.url).toContain('layers=rooms');
  });

  it('highlights the selected part on Plan and Punktsky', () => {
    const p = part({ cloud: 'instances', instance_id: 7 });
    const sources = evidenceSources(type({}, [p]), p);
    const [plan, punktsky] = [tabOf(sources, 'plan'), tabOf(sources, 'punktsky')];
    expect(plan.url).toContain('highlight_instance=7');
    expect(plan.url).toContain('highlight_cloud=instances');
    expect(punktsky.url).toContain('highlight_instance=7');
    expect(punktsky.url).toContain('highlight_cloud=instances');
  });

  it('falls back to the type\'s first linked part when no part is selected', () => {
    const unlinked = part({ code: 'RX-000', cloud: null, instance_id: null });
    const linked = part({ code: 'RX-002', cloud: 'instances', instance_id: 9 });
    const t = type({}, [unlinked, linked]);
    const [plan] = evidenceSources(t, null);
    expect(plan.url).toContain('highlight_instance=9');
  });

  it('does not highlight when the type has no linked part', () => {
    const unlinked = part({ cloud: null, instance_id: null });
    const t = type({}, [unlinked]);
    const [plan] = evidenceSources(t, null);
    expect(plan.url).not.toContain('highlight_instance');
  });

  describe('Foto', () => {
    it('reports "not linked" when the part has no instance', () => {
      const p = part({ cloud: null, instance_id: null });
      const foto = tabOf(evidenceSources(type({}, [p]), p), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.caption).toBe('Bedste foto');
      expect(foto.empty).toBe('Intet foto — bygningsdelen er ikke koblet til en instans.');
    });

    it('reports loading while the frame lookup is in flight (photoFrameId undefined)', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, undefined), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Indlæser foto…');
    });

    it('reports no frame found once resolved to null', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, null), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Intet foto — der blev ikke fundet en ramme for denne instans.');
    });

    it('builds the frame image URL and caption once resolved', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, 42), 'foto');
      expect(foto.url).toContain('/frames/42/image');
      expect(foto.url).toContain('kind=color');
      expect(foto.url).toContain('max_size=960');
      expect(foto.caption).toBe('Bedste foto · ramme 42');
    });

    it('treats instance_id 0 as unlinked — labels start at 1 (STANDARDS §3)', () => {
      const p = part({ instance_id: 0 });
      const foto = tabOf(evidenceSources(type({}, [p]), p, 1), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Intet foto — bygningsdelen er ikke koblet til en instans.');
    });

    it('reports a distinct message when the lookup itself failed', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, undefined, true), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Foto kunne ikke hentes.');
    });
  });

  describe('360°', () => {
    const pano = { panorama_id: 5, node_id: 12, distance: 2.5, u: 0.3, v: 0.55, heading: 'resected' as const };

    it('shows the lookup state while there is no panorama', () => {
      const p = part();
      const loading = tabOf(evidenceSources(type({}, [p]), p), 'pano');
      expect(loading.url).toBeNull();
      expect(loading.empty).toBe(PANO_TEXT.loading);
      const none = tabOf(evidenceSources(type({}, [p]), p, null, false, { pano: null, empty: PANO_TEXT.none }), 'pano');
      expect(none.empty).toBe('Ingen 360°-optagelse nær denne ressource');
      expect(none.href).toBeUndefined();
    });

    it('points at the nearest equirect, centred on the part, with a viewport link', () => {
      const p = part({ room_name: 'Mødelokale' });
      const src = tabOf(evidenceSources(type({}, [p]), p, null, false, { pano, empty: '' }), 'pano');
      expect(src.label).toBe('360°');
      expect(src.url).toContain('/panoramas/5/image');
      expect(src.url).toContain('max_size=2048');
      expect(src.pano).toEqual({ id: 5, u: 0.3, v: 0.55, marker: true });
      expect(src.caption).toBe('Mødelokale · 360°');
      expect(src.href).toBe('/viewport?pano=5');
    });
  });

  it('a part with instance_id 0 is never highlighted on Plan/Punktsky either', () => {
    const p = part({ instance_id: 0 });
    const [plan] = evidenceSources(type({}, [p]), p);
    expect(plan.url).not.toContain('highlight_instance');
  });
});

describe('kortlægning resolvePhotoState', () => {
  it('reports loading when no data has arrived yet', () => {
    expect(resolvePhotoState('instances/7', undefined)).toEqual({
      photoFrameId: undefined,
      photoFailed: false,
    });
  });

  it('does not use data tagged with a different key — the stale-part bug', () => {
    // Part A's lookup already resolved; the selection has since moved to
    // part B (a different key) whose own lookup hasn't settled yet. Must
    // report "loading", never A's frame id captioned as B's evidence.
    const staleFromA: FrameLookup = {
      key: 'instances/7',
      frames: [frame({ frame_id: 99 })],
      failed: false,
    };
    const result = resolvePhotoState('instances/9', staleFromA);
    expect(result).toEqual({ photoFrameId: undefined, photoFailed: false });
    expect(result.photoFrameId).not.toBe(99);
  });

  it('does not surface a stale error from a superseded key either', () => {
    const staleErrorFromA: FrameLookup = { key: 'instances/7', frames: [], failed: true };
    const result = resolvePhotoState('instances/9', staleErrorFromA);
    expect(result).toEqual({ photoFrameId: undefined, photoFailed: false });
  });

  it('reports failure only once the error is for the current key', () => {
    const failedForCurrent: FrameLookup = { key: 'instances/9', frames: [], failed: true };
    expect(resolvePhotoState('instances/9', failedForCurrent)).toEqual({
      photoFrameId: undefined,
      photoFailed: true,
    });
  });

  it('resolves the best frame id once fresh data for the current key arrives', () => {
    const fresh: FrameLookup = {
      key: 'instances/9',
      frames: [frame({ frame_id: 42 }), frame({ frame_id: 43 })],
      failed: false,
    };
    expect(resolvePhotoState('instances/9', fresh)).toEqual({
      photoFrameId: 42,
      photoFailed: false,
    });
  });

  it('resolves to null (not undefined) when the current key genuinely has no frames', () => {
    const empty: FrameLookup = { key: 'instances/9', frames: [], failed: false };
    expect(resolvePhotoState('instances/9', empty)).toEqual({
      photoFrameId: null,
      photoFailed: false,
    });
  });
});
