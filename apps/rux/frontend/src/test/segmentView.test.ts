// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { FrameInfo, FrameSegmentPrompt, FrameSegmentResult, SurveyType } from '../api/types';
import {
  buildRequestPrompts,
  clampNeighborCount,
  NEIGHBOR_MAX,
  cornersToBox,
  defaultTypeChoice,
  effectiveTypeChoice,
  filmstripWindow,
  geometryHint,
  geometryNotice,
  maskRgba,
  newTypeLabel,
  offscreenRunError,
  pointBox,
  resourceBlockReason,
  resourceRequest,
  resultClasses,
  segmentKeyAction,
  seedToImage,
  selectableTypes,
  stepFrame,
  type SegPrompt,
} from '../data/segmentView';

describe('filmstrip', () => {
  const ids = [10, 11, 12, 13, 14, 15, 16];

  it('centres a fixed-width window on the current frame, clamped at the ends', () => {
    expect(filmstripWindow(ids, 13, 1)).toEqual([12, 13, 14]);
    expect(filmstripWindow(ids, 10, 2)).toEqual([10, 11, 12, 13, 14]);
    expect(filmstripWindow(ids, 16, 2)).toEqual([12, 13, 14, 15, 16]);
    expect(filmstripWindow(ids, 99, 1)).toEqual([10, 11, 12]);
    expect(filmstripWindow([1, 2], 2, 5)).toEqual([1, 2]);
    expect(filmstripWindow([], null, 3)).toEqual([]);
  });

  it('steps through frames, clamped, starting from the first when lost', () => {
    expect(stepFrame(ids, 12, 1)).toBe(13);
    expect(stepFrame(ids, 10, -1)).toBe(10);
    expect(stepFrame(ids, 16, 5)).toBe(16);
    expect(stepFrame(ids, 99, 1)).toBe(10);
    expect(stepFrame([], 1, 1)).toBeNull();
  });
});

describe('prompt geometry', () => {
  it('turns a click into a clamped point box and two corners into a normalised box', () => {
    expect(pointBox(100, 50, 640, 480)).toEqual([92, 42, 108, 58]);
    expect(pointBox(2, 479, 640, 480)).toEqual([0, 471, 10, 479]);
    expect(cornersToBox({ x: 50.4, y: 9 }, { x: 10, y: 30.6 }, 640, 480)).toEqual([10, 9, 50, 31]);
  });

  it('maps a seed pixel from the intrinsics grid to the colour image', () => {
    expect(seedToImage({ u: 96, v: 72 }, { width: 192, height: 144 }, { width: 1920, height: 1440 })).toEqual({
      x: 960,
      y: 720,
    });
    expect(seedToImage({ u: 5, v: 5 }, undefined, { width: 10, height: 10 })).toEqual({ x: 5, y: 5 });
    expect(seedToImage({ u: 10, v: 5 }, undefined, { width: 10, height: 10 })).toBeNull();
  });
});

describe('buildRequestPrompts', () => {
  it('sends geometry-only prompts with empty text, drops empty text-only ones, and records the index order', () => {
    const prompts: SegPrompt[] = [
      { id: 'a', text: '  door ', box: null, point: false },
      { id: 'b', text: '', box: null, point: false },
      { id: 'c', text: '', box: [1, 2, 3, 4], point: false },
    ];
    const { prompts: wire, sent } = buildRequestPrompts(prompts);
    expect(wire).toEqual([{ text: 'door' }, { text: '', boxes: [['pos', [1, 2, 3, 4]]] }]);
    expect(sent).toEqual([
      { promptId: 'a', text: 'door' },
      { promptId: 'c', text: '' },
    ]);
  });

  it('sends a clicked point as exactly the clicked pixel, not as a box', () => {
    const prompts: SegPrompt[] = [{ id: 'p', text: 'chair', box: [92, 42, 108, 58], point: true, at: [100.5, 50.25] }];
    expect(buildRequestPrompts(prompts).prompts).toEqual([{ text: 'chair', points: [[100.5, 50.25]] }]);
  });

  it('keeps a click at the image edge where it was, though its marker is clamped', () => {
    const at: [number, number] = [2, 479];
    const prompts: SegPrompt[] = [{ id: 'p', text: '', box: pointBox(2, 479, 640, 480), point: true, at }];
    expect(buildRequestPrompts(prompts).prompts).toEqual([{ text: '', points: [[2, 479]] }]);
  });
});

describe('resultClasses', () => {
  it('lists each sent prompt by index with its pixel count; box-only prompts get a neutral name', () => {
    const result = { frame_id: 1, labeled_pixels: 30, saved: true, labels: { '0': 'door', '1': 'visual' } };
    const sent = [
      { promptId: 'a', text: 'door' },
      { promptId: 'c', text: '' },
    ];
    expect(resultClasses(result, sent, new Map([[0, 20], [1, 10]]))).toEqual([
      { index: 0, promptId: 'a', name: 'door', className: 'door', pixels: 20 },
      { index: 1, promptId: 'c', name: 'Markering 2', className: '', pixels: 10 },
    ]);
    expect(resultClasses(result, sent, null)[0].pixels).toBeNull();
    expect(resultClasses(result, sent, new Map())[1].pixels).toBe(0);
  });

  it('lists the model default classes from labels when no prompt was sent', () => {
    const result = { frame_id: 1, labeled_pixels: 3, saved: true, labels: { '2': 'wall', '0': 'floor' } };
    expect(resultClasses(result, [], null).map((c) => [c.index, c.name])).toEqual([
      [0, 'floor'],
      [2, 'wall'],
    ]);
  });
});

describe('resourceBlockReason', () => {
  const frame = { id: 1, pose: [], has_segmentation: true } as FrameInfo;
  it('blocks without pose or depth and explains why', () => {
    expect(resourceBlockReason(undefined)).toMatch(/indlæses/);
    expect(resourceBlockReason({ ...frame, has_pose: false, has_depth: true })).toMatch(/kameraposition/);
    expect(resourceBlockReason({ ...frame, has_pose: true, has_depth: false })).toMatch(/dybdebillede/);
    expect(resourceBlockReason({ ...frame, has_pose: true, has_depth: true })).toBeNull();
  });
});

describe('maskRgba', () => {
  it('colours prompts by --label-N, leaves unlabeled transparent and emphasises the selection', () => {
    const image = { width: 3, height: 1, data: new Uint16Array([0, 1, 2]) };
    const colors: [number, number, number][] = [
      [1, 0, 0],
      [0, 0, 1],
    ];
    const alpha = { normal: 140, strong: 200, faint: 40 };
    expect([...maskRgba(image, colors, null, alpha)]).toEqual([0, 0, 0, 0, 255, 0, 0, 140, 0, 0, 255, 140]);
    const sel = maskRgba(image, colors, 1, alpha);
    expect(sel[7]).toBe(40);
    expect(sel[11]).toBe(200);
  });
});

describe('resource dialog', () => {
  const type = (id: number, name: string, semantic_class: number, review: SurveyType['review_status'] = 'queue') =>
    ({ id, name, semantic_class, review_status: review }) as SurveyType;
  const types = [type(1, 'Vindue', 4), type(2, 'Dør', 7), type(3, 'Spam', 9, 'rejected')];

  it('offers non-rejected types alphabetically', () => {
    expect(selectableTypes(types).map((t) => t.name)).toEqual(['Dør', 'Vindue']);
  });

  it('preselects the type the server would use, else a new one', () => {
    const labels = { '0': 'ceiling', '4': 'window' };
    expect(defaultTypeChoice(types, 'window', labels)).toBe(1); // by semantic class
    expect(defaultTypeChoice(types, 'Dør', labels)).toBe(2); // by name
    expect(defaultTypeChoice(types, 'ceiling', labels)).toBe('new'); // id 0 is not a class
    expect(defaultTypeChoice(types, 'radiator', null)).toBe('new');
    expect(defaultTypeChoice(types, '  ', labels)).toBe('new');
    expect(newTypeLabel(' radiator ')).toBe('Ny type: radiator');
  });

  it('never preselects a rejected type: it creates a new one, as the server does', () => {
    const labels = { '9': 'spam' };
    // By semantic class (9) and by name ("Spam"), only the rejected type 3 matches.
    expect(defaultTypeChoice(types, 'spam', labels)).toBe('new');
    expect(defaultTypeChoice(types, 'Spam', null)).toBe('new');
    // A rejected match by class falls through to a live type by name.
    const both = [...types, type(4, 'spam', 2)];
    expect(defaultTypeChoice(both, 'spam', labels)).toBe(4);
  });

  it('shows the type it submits: the choice is always a rendered option', () => {
    const options = selectableTypes(types);
    // Untouched: follows the automatic choice.
    expect(effectiveTypeChoice(null, 2, options)).toBe(2);
    expect(effectiveTypeChoice(null, 'new', options)).toBe('new');
    // A pick wins while it is on offer.
    expect(effectiveTypeChoice(1, 'new', options)).toBe(1);
    expect(effectiveTypeChoice('new', 'new', options)).toBe('new');
    // "Ny type" is only on offer when automatic; otherwise the automatic type.
    expect(effectiveTypeChoice('new', 2, options)).toBe(2);
    // A pick (or an automatic id) that is not rendered — rejected, deleted —
    // never reaches the request.
    expect(effectiveTypeChoice(3, 'new', options)).toBe('new');
    expect(effectiveTypeChoice(3, 2, options)).toBe(2);
    expect(effectiveTypeChoice(null, 3, options)).toBe('new');
  });

  it('builds the request, omitting type_id for a new type', () => {
    expect(resourceRequest(1, ' door ', 'new')).toEqual({ mask_label: 1, class_name: 'door' });
    expect(resourceRequest(0, 'door', 2)).toEqual({ mask_label: 0, class_name: 'door', type_id: 2 });
  });

  it('echoes the run mask revision so a stale mask is refused', () => {
    expect(resourceRequest(0, 'door', 'new', 'abc')).toEqual({ mask_label: 0, class_name: 'door', mask_revision: 'abc' });
    expect(resourceRequest(0, 'door', 'new', null)).toEqual({ mask_label: 0, class_name: 'door' });
  });
});

describe('geometryNotice', () => {
  const result = (geometry: boolean) =>
    ({ frame_id: 1, labeled_pixels: 0, saved: true, labels: {}, geometry_prompts_used: geometry, mask_revision: 'r' }) as FrameSegmentResult;
  const box = { prompts: [{ text: '', boxes: [['pos', [0, 0, 4, 4]]] }] } as { prompts: FrameSegmentPrompt[] };
  const point = { prompts: [{ text: 'dør', points: [[2, 2]] }] } as { prompts: FrameSegmentPrompt[] };
  const text = { prompts: [{ text: 'dør' }] } as { prompts: FrameSegmentPrompt[] };

  it('is quiet unless geometry was sent and did not reach the model', () => {
    expect(geometryNotice(result(true), box.prompts)).toBeNull();
    expect(geometryNotice(result(false), text.prompts)).toBeNull();
    expect(geometryNotice(result(false), [])).toBeNull();
    expect(geometryNotice(result(false), box.prompts)).toMatch(/nåede ikke modellen/);
    expect(geometryNotice(result(false), point.prompts)).toMatch(/nåede ikke modellen/);
  });

  it('treats an older server without the field as "used" (no false alarm)', () => {
    const old = { frame_id: 1, labeled_pixels: 0, saved: true, labels: {} } as FrameSegmentResult;
    expect(geometryNotice(old, box.prompts)).toBeNull();
  });
});

describe('segmentKeyAction', () => {
  it('steps frames and toggles the mask only outside fields; Ctrl/⌘+Enter runs anywhere', () => {
    expect(segmentKeyAction({ key: 'ArrowLeft', inField: false })).toBe('prev');
    expect(segmentKeyAction({ key: 'ArrowRight', inField: false })).toBe('next');
    expect(segmentKeyAction({ key: 'm', inField: false })).toBe('mask');
    expect(segmentKeyAction({ key: 'ArrowRight', inField: true })).toBeNull();
    expect(segmentKeyAction({ key: 'm', inField: false, ctrlKey: true })).toBeNull();
    expect(segmentKeyAction({ key: 'Enter', inField: true, ctrlKey: true })).toBe('run');
    expect(segmentKeyAction({ key: 'Enter', inField: false })).toBeNull();
  });
});

describe('geometryHint', () => {
  const cls = (promptId: string, className: string, pixels: number) => ({
    index: 0,
    promptId,
    name: className || 'Markering 1',
    className,
    pixels,
  });
  it('only speaks up when a drawn point or box found nothing', () => {
    const point: SegPrompt = { id: 'p', text: '', box: [0, 0, 16, 16], point: true, at: [8, 8] };
    const box: SegPrompt = { id: 'b', text: '', box: [0, 0, 50, 50], point: false };
    const text: SegPrompt = { id: 't', text: 'wall', box: null, point: false };
    // A point that found its object is fine now that points reach SAM3.
    expect(geometryHint([cls('p', '', 9888)], [point])).toBeNull();
    expect(geometryHint([cls('p', '', 0)], [point])).toMatch(/punkt/i);
    expect(geometryHint([cls('b', '', 0)], [box])).toMatch(/boks/i);
    expect(geometryHint([cls('b', '', 10)], [box])).toBeNull();
    expect(geometryHint([cls('t', 'wall', 0)], [text])).toBeNull();
  });

  it('suggests a class name only when the prompt has none', () => {
    const unnamed: SegPrompt = { id: 'b', text: '', box: [0, 0, 50, 50], point: false };
    const named: SegPrompt = { id: 'n', text: 'dør', box: [0, 0, 50, 50], point: false };
    const namedPoint: SegPrompt = { id: 'q', text: 'dør', box: [0, 0, 16, 16], point: true, at: [8, 8] };
    expect(geometryHint([cls('b', '', 0)], [unnamed])).toMatch(/klassenavn/);
    expect(geometryHint([cls('n', 'dør', 0)], [named])).not.toMatch(/klassenavn/);
    expect(geometryHint([cls('n', 'dør', 0)], [named])).toMatch(/dør/);
    expect(geometryHint([cls('q', 'dør', 0)], [namedPoint])).not.toMatch(/klassenavn/);
  });
});

describe('clampNeighborCount', () => {
  it('clamps the Før/Efter count to 0..max and drops junk', () => {
    expect(clampNeighborCount('7')).toBe(7);
    expect(clampNeighborCount('100000')).toBe(NEIGHBOR_MAX);
    expect(clampNeighborCount('-3')).toBe(0);
    expect(clampNeighborCount('')).toBe(0);
    expect(clampNeighborCount('abc')).toBe(0);
    expect(clampNeighborCount('2.9')).toBe(2);
  });
});

describe('offscreenRunError', () => {
  it('names the frame the failed run was for', () => {
    expect(offscreenRunError(1500, 'SAM3 svarede ikke')).toBe('Segmentering af billede 1500 fejlede: SAM3 svarede ikke');
  });
});
