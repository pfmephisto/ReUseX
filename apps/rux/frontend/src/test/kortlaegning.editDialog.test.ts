// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import {
  partChips,
  partCountText,
  PHOTO_STRIP_MAX,
  photoStrip,
  primaryLabel,
  quantityLabel,
  titlePrefix,
  wrapFocusIndex,
} from '../components/kortlaegning/EditDialog';

function part(overrides: Partial<SurveyPart> = {}): SurveyPart {
  return {
    code: 'RX-001',
    type_id: 1,
    cloud: 'instances',
    instance_id: 7,
    room_id: 1,
    room_name: 'Production Hall',
    quantity: 18,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: 'guid-1',
    orphaned: false,
    ...overrides,
  };
}

function type(overrides: Partial<SurveyType> = {}): SurveyType {
  return {
    id: 1,
    name: 'Betonsøjler, bærende',
    eak_code: '17.01.01',
    eak_name: 'Beton',
    bim7aa_code: '221 Bærende konstr.',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.88,
    mass_t: 58,
    note: '',
    starred: false,
    semantic_class: 1,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 24,
    parts: [part(), part({ code: 'RX-002', room_name: 'Entrance', quantity: 6 })],
    created_at: '',
    updated_at: '',
    ...overrides,
  };
}

function frames(n: number): VisibleFrame[] {
  return Array.from({ length: n }, (_, i) => ({
    frame_id: i + 1,
    centrality: 0,
    score: 1,
    depth: 1,
    u: 0,
    v: 0,
  }));
}

describe('titlePrefix', () => {
  it('prefixes the part code when a part is selected', () => {
    expect(titlePrefix(part())).toBe('RX-001 · ');
  });

  it('is empty for a type-level selection', () => {
    expect(titlePrefix(null)).toBe('');
  });
});

describe('partChips', () => {
  it('starts with an "Alle" chip carrying the total and unit', () => {
    expect(partChips(type())[0]).toEqual({ code: null, label: 'Alle · 24 stk' });
  });

  it('lists each part as code · room · quantity', () => {
    expect(partChips(type()).slice(1)).toEqual([
      { code: 'RX-001', label: 'RX-001 · Production Hall · 18' },
      { code: 'RX-002', label: 'RX-002 · Entrance · 6' },
    ]);
  });

  it('formats quantities the Danish way', () => {
    const t = type({ quantity: 1240.5, unit: 'm²', parts: [part({ quantity: 1240.5 })] });
    expect(partChips(t).map((c) => c.label)).toEqual([
      'Alle · 1.240,5 m²',
      'RX-001 · Production Hall · 1.240,5',
    ]);
  });

  it('has only the "Alle" chip for a type without parts', () => {
    expect(partChips(type({ parts: [] }))).toHaveLength(1);
  });
});

describe('quantityLabel', () => {
  it('names a part quantity "denne del"', () => {
    expect(quantityLabel(part())).toBe('Mængde (denne del)');
  });

  it('names the type quantity as an aggregate spread over the parts', () => {
    expect(quantityLabel(null)).toBe('Mængde (aggregeret — fordeles på delene)');
  });
});

describe('partCountText', () => {
  it('uses the singular for one part', () => {
    expect(partCountText(1)).toBe('1 del');
  });

  it('uses the plural otherwise', () => {
    expect(partCountText(2)).toBe('2 dele');
    expect(partCountText(0)).toBe('0 dele');
  });
});

describe('primaryLabel', () => {
  it('asks to approve a queued type', () => {
    expect(primaryLabel(type())).toBe('Godkend & næste ✓');
  });

  it('only moves on for an approved type', () => {
    expect(primaryLabel(type({ review_status: 'approved' }))).toBe('Godkendt ✓ — næste');
  });

  it('offers approval again for a rejected type', () => {
    expect(primaryLabel(type({ review_status: 'rejected' }))).toBe('Godkend & næste ✓');
  });
});

describe('photoStrip', () => {
  it('shows at most five thumbnails', () => {
    expect(PHOTO_STRIP_MAX).toBe(5);
  });

  it('shows all frames and no overflow at or below the limit', () => {
    expect(photoStrip(frames(3))).toEqual({ visible: frames(3), overflow: 0 });
    expect(photoStrip(frames(5))).toEqual({ visible: frames(5), overflow: 0 });
  });

  it('collapses the rest into an overflow count, keeping the order', () => {
    const strip = photoStrip(frames(8));
    expect(strip.visible.map((f) => f.frame_id)).toEqual([1, 2, 3, 4, 5]);
    expect(strip.overflow).toBe(3);
  });

  it('handles no frames', () => {
    expect(photoStrip([])).toEqual({ visible: [], overflow: 0 });
  });
});

describe('wrapFocusIndex', () => {
  it('wraps Tab from the last element to the first', () => {
    expect(wrapFocusIndex(4, 3, false)).toBe(0);
  });

  it('wraps Shift+Tab from the first element to the last', () => {
    expect(wrapFocusIndex(4, 0, true)).toBe(3);
  });

  it('leaves Tab inside the list to the browser', () => {
    expect(wrapFocusIndex(4, 1, false)).toBeNull();
    expect(wrapFocusIndex(4, 2, true)).toBeNull();
  });

  it('pulls focus from outside the list back in', () => {
    expect(wrapFocusIndex(4, -1, false)).toBe(0);
    expect(wrapFocusIndex(4, -1, true)).toBe(3);
  });

  it('wraps onto itself with a single focusable', () => {
    expect(wrapFocusIndex(1, 0, false)).toBe(0);
    expect(wrapFocusIndex(1, 0, true)).toBe(0);
  });

  it('does nothing with no focusables', () => {
    expect(wrapFocusIndex(0, -1, false)).toBeNull();
  });
});
