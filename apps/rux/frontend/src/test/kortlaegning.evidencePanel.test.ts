// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType } from '../api/types';
import { evidenceSources } from '../components/kortlaegning/EvidencePanel';
import { EVIDENCE_TABS } from '../kortlaegning/keys';

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
  it('returns the four sources in EVIDENCE_TABS order', () => {
    const sources = evidenceSources(null, null);
    expect(sources.map((s) => s.tab)).toEqual(EVIDENCE_TABS);
    expect(sources.map((s) => s.label)).toEqual(['Plan', 'Foto', 'Punktsky', 'Rum-model']);
  });

  it('carries fixed captions for Plan, Punktsky and Rum-model', () => {
    const [plan, , punktsky, rum] = evidenceSources(type(), part());
    expect(plan.caption).toBe('Stueplan · snit i 1,2 m');
    expect(punktsky.caption).toBe('Punktsky · bygningsdel markeret');
    expect(rum.caption).toBe('Rumvis model · segmenterede rum');
  });

  it('builds the Rum-model render with no highlight — it is never part-specific', () => {
    const [, , , rum] = evidenceSources(type(), part());
    expect(rum.url).not.toBeNull();
    expect(rum.url).not.toContain('highlight_instance');
    expect(rum.url).toContain('view=orbit');
    expect(rum.url).toContain('layers=rooms');
  });

  it('highlights the selected part on Plan and Punktsky', () => {
    const p = part({ cloud: 'instances', instance_id: 7 });
    const [plan, , punktsky] = evidenceSources(type({}, [p]), p);
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
      const [, foto] = evidenceSources(type({}, [p]), p);
      expect(foto.url).toBeNull();
      expect(foto.caption).toBe('Bedste foto');
      expect(foto.empty).toBe('Ingen foto — bygningsdelen er ikke koblet til en instans.');
    });

    it('reports loading while the frame lookup is in flight (photoFrameId undefined)', () => {
      const p = part();
      const [, foto] = evidenceSources(type({}, [p]), p, undefined);
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Indlæser foto…');
    });

    it('reports no frame found once resolved to null', () => {
      const p = part();
      const [, foto] = evidenceSources(type({}, [p]), p, null);
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Ingen foto — der blev ikke fundet en ramme for denne instans.');
    });

    it('builds the frame image URL and caption once resolved', () => {
      const p = part();
      const [, foto] = evidenceSources(type({}, [p]), p, 42);
      expect(foto.url).toContain('/frames/42/image');
      expect(foto.url).toContain('kind=color');
      expect(foto.url).toContain('max_size=960');
      expect(foto.caption).toBe('Bedste foto · ramme 42');
    });

    it('treats instance_id 0 as linked (not falsy-checked)', () => {
      const p = part({ instance_id: 0 });
      const [, foto] = evidenceSources(type({}, [p]), p, 1);
      expect(foto.url).not.toBeNull();
    });
  });
});
