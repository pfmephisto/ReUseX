// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { newSampleHref, parseMiljoeQuery, parseTypeQuery, sampleHref, surveyTypeHref } from '../app/links';
import * as links from '../app/links';

describe('cross-screen links', () => {
  it('builds the hrefs both screens link with', () => {
    expect(sampleHref(1)).toBe('/miljoe?sample=1');
    expect(newSampleHref(6)).toBe('/miljoe?ny=6');
    expect(surveyTypeHref(11)).toBe('/kortlaegning?type=11');
  });

  it('parses the Miljø query, ignoring anything that is not a positive id', () => {
    expect(parseMiljoeQuery('?sample=3')).toEqual({ sampleId: 3, newForType: null });
    expect(parseMiljoeQuery('?ny=6')).toEqual({ sampleId: null, newForType: 6 });
    expect(parseMiljoeQuery('')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=abc&ny=-2')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=0')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=1.5')).toEqual({ sampleId: null, newForType: null });
  });

  it('parses the Kortlægning type query', () => {
    expect(parseTypeQuery('?type=6')).toBe(6);
    expect(parseTypeQuery('?type=')).toBeNull();
    expect(parseTypeQuery('?other=1')).toBeNull();
    expect(parseTypeQuery('?type=-1')).toBeNull();
    expect(parseTypeQuery('?type=1.5')).toBeNull();
    expect(parseTypeQuery('?type=9007199254740993')).toBeNull();
  });

  it('takes the first value of a duplicate-key type query, like URLSearchParams does', () => {
    expect(parseTypeQuery('?type=6&type=7')).toBe(6);
  });
});

describe('retired On-site links', () => {
  it('links no longer export the On-site helpers', () => {
    for (const name of ['ONSITE_PATH', 'onsiteHref', 'parseOnsiteQuery', 'rawOnsiteDel']) {
      expect(name in links, name).toBe(false);
    }
  });
});

describe('Segmentering links', () => {
  it('builds /segmentering hrefs with an optional frame and seed pixel', () => {
    expect(links.SEGMENTERING_PATH).toBe('/segmentering');
    expect(links.segmentHref()).toBe('/segmentering');
    expect(links.segmentHref(12)).toBe('/segmentering?frame=12');
    expect(links.segmentHref(12, { u: 101.6, v: 40.2 })).toBe('/segmentering?frame=12&u=102&v=40');
  });

  it('parses frame, u and v; u/v only as a non-negative pair', () => {
    expect(links.parseSegmentQuery('?frame=12&u=102&v=40')).toEqual({ frameId: 12, seed: { u: 102, v: 40 } });
    expect(links.parseSegmentQuery('?frame=12')).toEqual({ frameId: 12, seed: null });
    expect(links.parseSegmentQuery('?frame=12&u=5')).toEqual({ frameId: 12, seed: null });
    expect(links.parseSegmentQuery('?frame=12&u=-1&v=3')).toEqual({ frameId: 12, seed: null });
    expect(links.parseSegmentQuery('?frame=0&u=1&v=1')).toEqual({ frameId: null, seed: null });
    expect(links.parseSegmentQuery('?frame=x')).toEqual({ frameId: null, seed: null });
    expect(links.parseSegmentQuery('?frame=3&u=1.5&v=2.25')).toEqual({ frameId: 3, seed: { u: 1.5, v: 2.25 } });
  });
});
