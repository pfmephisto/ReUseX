// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  ABSENT,
  formatArea,
  formatConfidence,
  formatCount,
  formatFixed,
  formatGuidShort,
  formatParent,
  formatText,
} from '../data/format';
import { parseDataTab } from '../data/tabs';
import { COMPONENTS } from './fixtures';

describe('formatArea', () => {
  it('renders square metres to two decimals', () => {
    expect(formatArea(1.4400000000000002)).toBe('1.44 m²');
    expect(formatArea(21.840000000000003)).toBe('21.84 m²');
  });

  it('shows a real zero rather than hiding it', () => {
    expect(formatArea(0)).toBe('0.00 m²');
  });

  it('renders an absent or non-finite area as the absent glyph', () => {
    // `area` is optional in the contract — a boundary of fewer than three
    // vertices has none — so this is the normal path, not an error path.
    expect(formatArea(undefined)).toBe(ABSENT);
    expect(formatArea(Number.NaN)).toBe(ABSENT);
    expect(formatArea(Number.POSITIVE_INFINITY)).toBe(ABSENT);
  });
});

describe('formatConfidence', () => {
  it('renders a detection confidence as a percentage', () => {
    expect(formatConfidence(0.94)).toBe('94%');
  });

  it('names the manual sentinel instead of printing it', () => {
    // -1 means hand-authored. Rendering it as -100% would sort manual
    // components below the worst automatic ones.
    expect(formatConfidence(-1)).toBe('manual');
  });

  it('renders an absent confidence as the absent glyph', () => {
    expect(formatConfidence(undefined)).toBe(ABSENT);
  });
});

describe('formatParent', () => {
  it('renders a real parent component id', () => {
    expect(formatParent(7)).toBe('7');
  });

  it('hides the no-parent sentinel, which is emphatically not a room', () => {
    expect(formatParent(-1)).toBe(ABSENT);
    expect(formatParent(undefined)).toBe(ABSENT);
  });
});

describe('formatGuidShort', () => {
  it('shows the leading segment of a UUID-shaped guid', () => {
    expect(formatGuidShort('9f8e7d6c-1111-2222-3333-444455556666')).toBe('9f8e7d6c');
  });

  it('falls back to eight characters when the leading segment is shorter', () => {
    // 'inst-a1b2c3d4' splits to 'inst', which is too short to identify a row.
    expect(formatGuidShort('inst-a1b2c3d4')).toBe('inst-a1b');
  });

  it('distinguishes an absent link from a truncated one', () => {
    expect(formatGuidShort(undefined)).toBe(ABSENT);
    expect(formatGuidShort('   ')).toBe(ABSENT);
  });
});

describe('formatText and formatCount', () => {
  it('treats an empty or whitespace string as absent', () => {
    expect(formatText('')).toBe(ABSENT);
    expect(formatText('  ')).toBe(ABSENT);
    expect(formatText(' notes ')).toBe('notes');
  });

  it('shows a zero count, which is an answer, not an absence', () => {
    expect(formatCount(0)).toBe('0');
    expect(formatCount(undefined)).toBe(ABSENT);
  });

  it('pads fixed-decimal numbers so a matrix column lines up', () => {
    expect(formatFixed(1)).toBe('1.000');
    expect(formatFixed(-0.0236, 3)).toBe('-0.024');
    expect(formatFixed(undefined)).toBe(ABSENT);
  });
});

describe('formatting the recorded components', () => {
  it('never renders undefined or NaN for an optional field', () => {
    // Both `area` and `source_instance_guid` are optional in the contract, and
    // the recorded set contains rows that omit the provenance link.
    for (const component of COMPONENTS) {
      for (const cell of [
        formatArea(component.area),
        formatConfidence(component.confidence),
        formatGuidShort(component.source_instance_guid),
        formatParent(component.parent_id),
      ]) {
        expect(cell).not.toMatch(/undefined|NaN/);
        expect(cell.length).toBeGreaterThan(0);
      }
    }
  });

  it('renders the manual door with no confidence and no provenance', () => {
    const manual = COMPONENTS.find((c) => c.name === 'Door-02');
    expect(manual).toBeDefined();
    expect(formatConfidence(manual!.confidence)).toBe('manual');
    expect(formatGuidShort(manual!.source_instance_guid)).toBe(ABSENT);
  });
});

describe('parseDataTab', () => {
  it('accepts the three documented tabs', () => {
    expect(parseDataTab('components')).toBe('components');
    expect(parseDataTab('materials')).toBe('materials');
    expect(parseDataTab('labels')).toBe('labels');
  });

  it('falls back to components for junk in the URL', () => {
    for (const raw of ['', null, undefined, 'passports']) {
      expect(parseDataTab(raw)).toBe('components');
    }
  });
});
