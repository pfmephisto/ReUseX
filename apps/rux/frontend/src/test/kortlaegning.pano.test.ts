// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { InstancePanorama } from '../api/types';
import {
  panoBackgroundX,
  panoCaption,
  panoImageWidth,
  panoKeyStep,
  panoMarkerX,
  pickPano,
  PANO_TEXT,
  resolvePano,
  wrapCentred,
} from '../kortlaegning/pano';

function pano(overrides: Partial<InstancePanorama> = {}): InstancePanorama {
  return { panorama_id: 1, node_id: 10, distance: 1, u: 0.5, v: 0.5, heading: 'resected', ...overrides };
}

describe('pickPano', () => {
  it('prefers the nearest resected panorama over a nearer levelled one', () => {
    const list = [pano({ panorama_id: 4, heading: 'levelled' }), pano({ panorama_id: 5 }), pano({ panorama_id: 7 })];
    expect(pickPano(list)?.panorama_id).toBe(5);
  });

  it('falls back to the nearest levelled one, and to null on an empty list', () => {
    expect(pickPano([pano({ panorama_id: 4, heading: 'levelled' })])?.panorama_id).toBe(4);
    expect(pickPano([])).toBeNull();
  });
});

describe('resolvePano', () => {
  it('says why there is no panorama', () => {
    expect(resolvePano(null, undefined)).toEqual({ pano: null, empty: PANO_TEXT.unlinked });
    expect(resolvePano('instances/3', undefined)).toEqual({ pano: null, empty: PANO_TEXT.loading });
    expect(resolvePano('instances/3', { key: 'instances/3', panoramas: [], failed: true }).empty).toBe(PANO_TEXT.failed);
    expect(resolvePano('instances/3', { key: 'instances/3', panoramas: [], failed: false }).empty).toBe(
      'Ingen 360°-optagelse nær denne ressource',
    );
  });

  it('never shows another highlight\'s panorama', () => {
    const stale = { key: 'instances/2', panoramas: [pano()], failed: false };
    expect(resolvePano('instances/3', stale)).toEqual({ pano: null, empty: PANO_TEXT.loading });
  });

  it('resolves the picked panorama for the current key', () => {
    const fresh = { key: 'instances/3', panoramas: [pano({ panorama_id: 9 })], failed: false };
    expect(resolvePano('instances/3', fresh)).toEqual({ pano: fresh.panoramas[0], empty: '' });
  });
});

describe('panoCaption', () => {
  it('is "<rum> · 360°", and says when the heading is unknown', () => {
    expect(panoCaption('Mødelokale')).toBe('Mødelokale · 360°');
    expect(panoCaption('  ')).toBe('360°');
    expect(panoCaption(null)).toBe('360°');
    expect(panoCaption('Gang', 'levelled')).toBe('Gang · 360° · retning ukendt');
  });
});

describe('pannable strip arithmetic', () => {
  it('draws a 2:1 equirect', () => {
    expect(panoImageWidth(300)).toBe(600);
  });

  it('centres column u in the strip before any pan', () => {
    // 400 px strip, 600 px image, u = 0.25 → image column 150 at x = 200.
    const x = panoBackgroundX(0.25, 400, 600, 0);
    expect(x).toBe(50);
    expect(x + 0.25 * 600).toBe(200);
  });

  it('wraps the background offset into one image period', () => {
    // u = 0.9 → raw 200 - 540 = -340 → wrapped 260 (same picture, repeat-x).
    expect(panoBackgroundX(0.9, 400, 600, 0)).toBe(260);
    expect(panoBackgroundX(0.25, 400, 600, 600)).toBe(50);
    expect(panoBackgroundX(0.5, 400, 0, 0)).toBe(0);
  });

  it('moves the marker with the pan and keeps it on the nearest copy', () => {
    expect(panoMarkerX(400, 600, 0)).toBe(200);
    expect(panoMarkerX(400, 600, 50)).toBe(250);
    // Panned a full turn: back where it started.
    expect(panoMarkerX(400, 600, 600)).toBe(200);
    // Panned 400 right is the same view as 200 left.
    expect(panoMarkerX(400, 600, 400)).toBe(0);
  });

  it('wrapCentred maps into [-period/2, period/2)', () => {
    expect(wrapCentred(0, 10)).toBe(0);
    expect(wrapCentred(5, 10)).toBe(-5);
    expect(wrapCentred(-6, 10)).toBe(4);
    expect(wrapCentred(3, 0)).toBe(0);
  });

  it('steps an eighth of the strip per arrow key, at least 1 px', () => {
    expect(panoKeyStep(400)).toBe(50);
    expect(panoKeyStep(0)).toBe(1);
  });
});
