// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  newSampleHref,
  ONSITE_PATH,
  onsiteHref,
  parseMiljoeQuery,
  parseOnsiteQuery,
  parseTypeQuery,
  sampleHref,
  surveyTypeHref,
} from '../app/links';

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

describe('on-site links', () => {
  it('builds and reads ?del=<part code>', () => {
    expect(ONSITE_PATH).toBe('/on-site');
    expect(onsiteHref('RX-008')).toBe('/on-site?del=RX-008');
    expect(parseOnsiteQuery('?del=RX-008')).toBe('RX-008');
    expect(parseOnsiteQuery('?del=RX-008&x=1')).toBe('RX-008');
  });

  it('ignores anything that is not a part code', () => {
    expect(parseOnsiteQuery('')).toBeNull();
    expect(parseOnsiteQuery('?del=')).toBeNull();
    expect(parseOnsiteQuery('?del=rx-008')).toBeNull();
    expect(parseOnsiteQuery('?del=RX-8a')).toBeNull();
    expect(parseOnsiteQuery('?del=%3Cscript%3E')).toBeNull();
  });
});
