// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Unit tests for the pure label-queue data module (#447).
 *
 * `loadStoredState` / `saveState` touch `localStorage`, which is absent in
 * the Node test environment — those helpers are thin wrappers and are not
 * tested here.
 */

import { beforeEach, describe, expect, it } from 'vitest';
import {
  _resetIds,
  clearFinished,
  makeLabel,
  makeQueueItems,
  neighborFrameIds,
  removeItem,
  removeLabel,
  setItemStatus,
  updateLabel,
  type LabelEntry,
  type QueueItem,
} from '../data/labelQueue';

beforeEach(() => {
  _resetIds();
});

// --------------------------------------------------------- label library --

describe('makeLabel', () => {
  it('trims whitespace from text', () => {
    expect(makeLabel('  wall  ').text).toBe('wall');
  });

  it('assigns a unique id', () => {
    const a = makeLabel('wall');
    const b = makeLabel('floor');
    expect(a.id).not.toBe(b.id);
  });

  it('stores confidence when provided', () => {
    expect(makeLabel('door', 0.7).confidence).toBe(0.7);
  });

  it('omits confidence when not provided', () => {
    expect(makeLabel('ceiling').confidence).toBeUndefined();
  });
});

describe('updateLabel', () => {
  it('updates text and confidence for the matching id', () => {
    const a = makeLabel('wall');
    const b = makeLabel('floor');
    const result = updateLabel([a, b], a.id, 'ceiling', 0.8);
    expect(result[0].text).toBe('ceiling');
    expect(result[0].confidence).toBe(0.8);
    expect(result[1]).toBe(b);
  });

  it('trims whitespace from new text', () => {
    const a = makeLabel('wall');
    const result = updateLabel([a], a.id, '  door  ');
    expect(result[0].text).toBe('door');
  });

  it('is a no-op for an unknown id', () => {
    const a = makeLabel('wall');
    const result = updateLabel([a], 'unknown', 'new');
    expect(result[0]).toBe(a);
  });

  it('can clear confidence by passing undefined', () => {
    const a = makeLabel('wall', 0.9);
    const result = updateLabel([a], a.id, 'wall', undefined);
    expect(result[0].confidence).toBeUndefined();
  });
});

describe('removeLabel', () => {
  it('removes the matching entry', () => {
    const a = makeLabel('wall');
    const b = makeLabel('floor');
    expect(removeLabel([a, b], a.id)).toEqual([b]);
  });

  it('is a no-op for an unknown id', () => {
    const a = makeLabel('wall');
    expect(removeLabel([a], 'ghost')).toEqual([a]);
  });

  it('returns an empty list when the only entry is removed', () => {
    const a = makeLabel('wall');
    expect(removeLabel([a], a.id)).toEqual([]);
  });
});

// -------------------------------------------------------------- queue ops --

function makeItems(): QueueItem[] {
  return makeQueueItems([10, 20, 30], [{ text: 'wall' }], '/models/sam3', 0.5);
}

describe('makeQueueItems', () => {
  it('creates one item per frame id', () => {
    const items = makeQueueItems([1, 2, 3], [{ text: 'wall' }], '/m', 0.5);
    expect(items).toHaveLength(3);
    expect(items.map((i) => i.frameId)).toEqual([1, 2, 3]);
  });

  it('sets status to pending', () => {
    const items = makeQueueItems([1], [], '/m', 0.5);
    expect(items[0].status).toBe('pending');
  });

  it('assigns unique ids', () => {
    const items = makeQueueItems([1, 2, 3], [], '/m', 0.5);
    const ids = new Set(items.map((i) => i.id));
    expect(ids.size).toBe(3);
  });

  it('copies prompts into each item', () => {
    const prompts = [{ text: 'wall' }, { text: 'floor' }];
    const items = makeQueueItems([1, 2], prompts, '/m', 0.5);
    expect(items[0].prompts).toEqual(prompts);
    expect(items[1].prompts).toEqual(prompts);
    // Shallow copy — mutations to original do not affect items
    prompts.push({ text: 'ceiling' });
    expect(items[0].prompts).toHaveLength(2);
  });

  it('stores modelPath and confidence', () => {
    const items = makeQueueItems([5], [], '/engines/sam3', 0.75);
    expect(items[0].modelPath).toBe('/engines/sam3');
    expect(items[0].confidence).toBe(0.75);
  });

  it('returns an empty list for an empty frame array', () => {
    expect(makeQueueItems([], [{ text: 'x' }], '/m', 0.5)).toEqual([]);
  });
});

describe('setItemStatus', () => {
  it('updates status for the matching id', () => {
    const items = makeItems();
    const updated = setItemStatus(items, items[0].id, 'running');
    expect(updated[0].status).toBe('running');
    expect(updated[1].status).toBe('pending');
  });

  it('attaches extra fields when provided', () => {
    const items = makeItems();
    const updated = setItemStatus(items, items[0].id, 'done', { labeledPixels: 42000 });
    expect(updated[0].labeledPixels).toBe(42000);
  });

  it('attaches error when status is failed', () => {
    const items = makeItems();
    const updated = setItemStatus(items, items[0].id, 'failed', { error: 'no segmenter' });
    expect(updated[0].error).toBe('no segmenter');
  });

  it('clears previous error and labeledPixels on status update', () => {
    const items = makeItems();
    const withError = setItemStatus(items, items[0].id, 'failed', { error: 'oops' });
    const cleared = setItemStatus(withError, items[0].id, 'pending');
    expect(cleared[0].error).toBeUndefined();
    expect(cleared[0].labeledPixels).toBeUndefined();
  });

  it('is a no-op for an unknown id', () => {
    const items = makeItems();
    const updated = setItemStatus(items, 'ghost', 'running');
    expect(updated).toEqual(items);
  });
});

describe('removeItem', () => {
  it('removes the item with the matching id', () => {
    const items = makeItems();
    const result = removeItem(items, items[1].id);
    expect(result).toHaveLength(2);
    expect(result.some((i) => i.id === items[1].id)).toBe(false);
  });

  it('is a no-op for an unknown id', () => {
    const items = makeItems();
    expect(removeItem(items, 'ghost')).toEqual(items);
  });
});

describe('clearFinished', () => {
  it('removes done items', () => {
    const items = makeItems();
    const mixed = [
      { ...items[0], status: 'done' as const },
      { ...items[1], status: 'pending' as const },
      { ...items[2], status: 'failed' as const },
    ];
    const result = clearFinished(mixed);
    expect(result).toHaveLength(2);
    expect(result.every((i) => i.status !== 'done')).toBe(true);
  });

  it('keeps running items', () => {
    const items = makeItems();
    const running = [{ ...items[0], status: 'running' as const }];
    expect(clearFinished(running)).toHaveLength(1);
  });

  it('returns an empty list when all items are done', () => {
    const done = makeItems().map((i) => ({ ...i, status: 'done' as const }));
    expect(clearFinished(done)).toEqual([]);
  });
});

// --------------------------------------------------------- frame windowing --

describe('neighborFrameIds', () => {
  const ids = [10, 20, 30, 40, 50];

  it('returns only the centre when before=0 and after=0', () => {
    expect(neighborFrameIds(ids, 30, 0, 0)).toEqual([30]);
  });

  it('returns correct window for a middle frame', () => {
    expect(neighborFrameIds(ids, 30, 1, 2)).toEqual([20, 30, 40, 50]);
  });

  it('clamps to the start of the array', () => {
    expect(neighborFrameIds(ids, 10, 5, 1)).toEqual([10, 20]);
  });

  it('clamps to the end of the array', () => {
    expect(neighborFrameIds(ids, 50, 2, 10)).toEqual([30, 40, 50]);
  });

  it('returns [centerId] when centerId is not in allIds', () => {
    expect(neighborFrameIds(ids, 99, 2, 2)).toEqual([99]);
  });

  it('returns [centerId] for an empty allIds', () => {
    expect(neighborFrameIds([], 5, 1, 1)).toEqual([5]);
  });

  it('ignores negative before/after values (treats as 0)', () => {
    expect(neighborFrameIds(ids, 30, -3, -2)).toEqual([30]);
  });

  it('returns the full array when window exceeds length on both sides', () => {
    expect(neighborFrameIds(ids, 30, 100, 100)).toEqual(ids);
  });

  it('includes only the centre and one neighbour when at the start', () => {
    expect(neighborFrameIds(ids, 10, 0, 1)).toEqual([10, 20]);
  });

  it('result always contains the centerId', () => {
    const result = neighborFrameIds(ids, 40, 2, 1);
    expect(result).toContain(40);
  });
});

// --------------------------------------------------------- label round-trip --

describe('label entry round-trip', () => {
  it('preserves all fields through add → update → remove', () => {
    const initial: LabelEntry[] = [];
    const a = makeLabel('wall', 0.6);
    const withA = [...initial, a];
    expect(withA).toHaveLength(1);

    const updated = updateLabel(withA, a.id, 'floor', 0.8);
    expect(updated[0].text).toBe('floor');
    expect(updated[0].confidence).toBe(0.8);
    expect(updated[0].id).toBe(a.id);

    const removed = removeLabel(updated, a.id);
    expect(removed).toHaveLength(0);
  });
});
