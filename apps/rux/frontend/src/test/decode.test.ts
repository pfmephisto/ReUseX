// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Decoding a points page into typed arrays, over both transports.
 *
 * Two silent corruptions live here, and both are asserted against explicitly:
 * reading columns positionally without consulting `fields` (which turns a Label
 * cloud's label column into an x coordinate), and treating 0..255 sRGB bytes as
 * if they were 0..1 linear floats (which washes the whole cloud out).
 *
 * The RUXP half (#283) is held to the same two, plus one the JSON path cannot
 * have: its positions and labels must arrive as *views* onto the response
 * buffer rather than copies, since avoiding that copy is the reason the binary
 * transport exists at all.
 */

import { describe, expect, it } from 'vitest';
import type { CloudPointsPage } from '../api/types';
import { parseRuxp } from '../viewport/binaryPoints';
import {
  decodeBinaryColors,
  decodeBinaryLabels,
  decodeBinaryPositions,
  decodeColors,
  decodeLabels,
  decodePositions,
  pageCount,
  pageTotal,
  toPageBuffers,
} from '../viewport/decode';
import { CLOUD_POINTS_PAGE, LABEL_POINTS_PAGE } from './fixtures';
import { XYZRGB_THREE_POINTS, buildRuxpPage } from './ruxpPage';

/** A one-off page, for the cases no recording covers. */
function page(fields: string[], points: number[][]): CloudPointsPage {
  return {
    name: 'synthetic',
    type: 'PointXYZRGB',
    offset: 0,
    count: points.length,
    total: points.length,
    fields,
    points,
  };
}

describe('decodePositions', () => {
  it('decodes the recorded PointXYZRGB page', () => {
    const positions = decodePositions(CLOUD_POINTS_PAGE);
    expect(positions).toBeInstanceOf(Float32Array);
    expect(positions).not.toBeNull();
    expect(positions!.length).toBe(CLOUD_POINTS_PAGE.count * 3);

    // Float32 cannot hold the recorded doubles exactly; that rounding is the
    // price of the GPU-ready layout and is well below scan noise.
    const [x, y, z] = CLOUD_POINTS_PAGE.points[0];
    expect(positions![0]).toBeCloseTo(x, 4);
    expect(positions![1]).toBeCloseTo(y, 4);
    expect(positions![2]).toBeCloseTo(z, 4);

    const last = CLOUD_POINTS_PAGE.points[3];
    expect(positions![9]).toBeCloseTo(last[0], 4);
    expect(positions![10]).toBeCloseTo(last[1], 4);
    expect(positions![11]).toBeCloseTo(last[2], 4);
  });

  it('returns null for a Label page — it has no geometry', () => {
    // The guard that stops column 0 (a label id) being read as an x coordinate,
    // which would scatter the whole cloud along the x axis.
    expect(decodePositions(LABEL_POINTS_PAGE)).toBeNull();
  });

  it('returns null for a Normal page', () => {
    expect(decodePositions(page(['nx', 'ny', 'nz'], [[0, 0, 1]]))).toBeNull();
  });

  it('reads positions by field name, not by column order', () => {
    // A server free to reorder `fields` must not silently swap the axes.
    const reordered = decodePositions(page(['z', 'y', 'x'], [[3, 2, 1]]));
    expect(Array.from(reordered!)).toEqual([1, 2, 3]);
  });

  it('decodes a PointXYZ page with no colour columns', () => {
    const positions = decodePositions(page(['x', 'y', 'z'], [[1, 2, 3]]));
    expect(Array.from(positions!)).toEqual([1, 2, 3]);
  });

  it('returns an empty array for an empty page', () => {
    expect(decodePositions(page(['x', 'y', 'z'], []))!.length).toBe(0);
  });
});

describe('decodeColors', () => {
  it('applies the sRGB transfer rather than dividing by 255', () => {
    const colors = decodeColors(CLOUD_POINTS_PAGE);
    expect(colors).not.toBeNull();
    expect(colors!.length).toBe(CLOUD_POINTS_PAGE.count * 3);

    // The recorded first point is r=126. three.js treats a colour attribute as
    // linear and applies the inverse transfer on output, so handing it
    // 126/255 = 0.4941 double-encodes and the cloud comes out washed out and
    // flat. The correct value is sRGB->linear of 126/255.
    expect(colors![0]).toBeCloseTo(0.2086369, 6);
    expect(colors![0]).not.toBeCloseTo(0.4941176, 3);

    expect(CLOUD_POINTS_PAGE.points[0][3]).toBe(126);
    expect(CLOUD_POINTS_PAGE.points[0][5]).toBe(118);
    // b = 118 is a different byte, so it must decode differently from r = 126.
    expect(colors![2]).toBeLessThan(colors![0]);
  });

  it('maps the endpoints exactly', () => {
    const colors = decodeColors(
      page(
        ['x', 'y', 'z', 'r', 'g', 'b'],
        [
          [0, 0, 0, 0, 255, 0],
          [0, 0, 0, 255, 0, 255],
        ],
      ),
    );
    expect(colors![0]).toBe(0);
    expect(colors![1]).toBe(1);
    expect(colors![2]).toBe(0);
    expect(colors![3]).toBe(1);
    expect(colors![4]).toBe(0);
    expect(colors![5]).toBe(1);
  });

  it('clamps out-of-range and non-finite components instead of producing NaN', () => {
    // One NaN in a colour attribute makes three.js drop the whole draw call,
    // not one point — so a broken byte must not escape this function.
    const colors = decodeColors(
      page(
        ['x', 'y', 'z', 'r', 'g', 'b'],
        [
          [0, 0, 0, -5, 300, Number.NaN],
          [0, 0, 0, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY, 127.6],
        ],
      ),
    );
    expect(Array.from(colors!).every((value) => Number.isFinite(value))).toBe(true);
    expect(colors![0]).toBe(0); // -5 clamps low
    expect(colors![1]).toBe(1); // 300 clamps high
    expect(colors![2]).toBe(0); // NaN falls back to 0
    expect(colors![3]).toBe(1); // +Infinity clamps high
    expect(colors![4]).toBe(0); // -Infinity clamps low
    expect(colors![5]).toBeGreaterThan(0); // 127.6 rounds to a real byte
    expect(colors![5]).toBeLessThan(1);
  });

  it('returns null for a page without colour', () => {
    expect(decodeColors(LABEL_POINTS_PAGE)).toBeNull();
    expect(decodeColors(page(['x', 'y', 'z'], [[1, 2, 3]]))).toBeNull();
    expect(decodeColors(page(['nx', 'ny', 'nz'], [[0, 0, 1]]))).toBeNull();
  });
});

describe('decodeLabels', () => {
  it('decodes the recorded Label page, preserving 0 as unlabeled', () => {
    const labels = decodeLabels(LABEL_POINTS_PAGE);
    expect(labels).toBeInstanceOf(Uint32Array);
    // No `label - 1` shift at decode time: 0 stays 0 (unlabeled, STANDARDS §3)
    // and 2 stays 2. The palette lookup is where the 1-based offset is applied.
    expect(Array.from(labels!)).toEqual([2, 0, 0, 0]);
  });

  it('keeps large label ids intact', () => {
    const labels = decodeLabels(page(['label'], [[1], [7], [65535], [1000000]]));
    expect(Array.from(labels!)).toEqual([1, 7, 65535, 1000000]);
  });

  it('folds a nonsensical label into unlabeled rather than wrapping it', () => {
    const labels = decodeLabels(page(['label'], [[-3], [Number.NaN], [2.7]]));
    expect(Array.from(labels!)).toEqual([0, 0, 2]);
  });

  it('returns null for a page with no label field', () => {
    expect(decodeLabels(CLOUD_POINTS_PAGE)).toBeNull();
    expect(decodeLabels(page(['x', 'y', 'z'], [[1, 2, 3]]))).toBeNull();
  });

  it('finds the label column wherever it sits', () => {
    const labels = decodeLabels(page(['x', 'y', 'z', 'label'], [[0, 0, 0, 4]]));
    expect(Array.from(labels!)).toEqual([4]);
  });
});

// ------------------------------------------------------------ RUXP (#283) ----

const XYZRGB_BUFFER = buildRuxpPage(XYZRGB_THREE_POINTS);

function labelPage(values: number[]) {
  return parseRuxp(
    buildRuxpPage({
      count: values.length,
      total: values.length,
      fields: [{ name: 'label', type: 3, components: 1, values }],
    }),
  );
}

describe('decodeBinaryPositions', () => {
  it('returns the xyz section itself, not a copy of it', () => {
    const page = parseRuxp(XYZRGB_BUFFER);
    const positions = decodeBinaryPositions(page)!;
    expect(Array.from(positions)).toEqual([1, 2, 3, -4.5, 0, 0.25, 1024, -0.5, 65536]);
    // The whole point of the binary transport: three.js gets the bytes the
    // socket delivered, with nothing in between.
    expect(positions.buffer).toBe(XYZRGB_BUFFER);
    expect(positions).toBe(page.view('xyz'));
  });

  it('reads a Normal page as no geometry rather than as positions', () => {
    // `normal` is f32x3 exactly like `xyz`, so a decoder that went by shape
    // instead of by name would render a unit sphere at the origin and call it
    // a scan.
    const page = parseRuxp(
      buildRuxpPage({
        count: 1,
        fields: [{ name: 'normal', type: 1, components: 3, values: [0, 0, 1] }],
      }),
    );
    expect(decodeBinaryPositions(page)).toBeNull();
  });

  it('returns null for a Label page', () => {
    expect(decodeBinaryPositions(labelPage([1, 2]))).toBeNull();
  });

  it('throws rather than misreads an xyz field of the wrong shape', () => {
    // Not null: null means "this cloud type has no positions", which the caller
    // treats as normal. A 2-component `xyz` is a server bug and must be loud.
    const page = parseRuxp(
      buildRuxpPage({
        count: 1,
        fields: [{ name: 'xyz', type: 1, components: 2, values: [1, 2] }],
      }),
    );
    expect(() => decodeBinaryPositions(page)).toThrow(/expected f32x3/);
  });
});

describe('decodeBinaryColors', () => {
  it('applies the sRGB transfer, not a divide by 255', () => {
    const colors = decodeBinaryColors(parseRuxp(XYZRGB_BUFFER))!;
    expect(colors).toBeInstanceOf(Float32Array);
    expect(colors.length).toBe(9);
    expect(colors[0]).toBe(0); // byte 0
    expect(colors[2]).toBe(1); // byte 255
    expect(colors[3]).toBeCloseTo(0.2086369, 6); // byte 126
    expect(colors[3]).not.toBeCloseTo(126 / 255, 3);
  });

  it('produces bit-identical values to the JSON path for the same bytes', () => {
    // The two transports share one 256-entry table by construction. If someone
    // ever duplicates it, the copies drift and this catches it — a cloud would
    // change colour depending on which server answered.
    const viaJson = decodeColors(
      page(
        ['x', 'y', 'z', 'r', 'g', 'b'],
        [
          [0, 0, 0, 0, 128, 255],
          [0, 0, 0, 126, 130, 118],
          [0, 0, 0, 1, 2, 3],
        ],
      ),
    )!;
    const viaBinary = decodeBinaryColors(parseRuxp(XYZRGB_BUFFER))!;
    expect(Array.from(viaBinary)).toEqual(Array.from(viaJson));
  });

  it('returns null for a page with no colour', () => {
    expect(decodeBinaryColors(labelPage([1]))).toBeNull();
    expect(
      decodeBinaryColors(
        parseRuxp(
          buildRuxpPage({ count: 1, fields: [{ name: 'xyz', type: 1, components: 3 }] }),
        ),
      ),
    ).toBeNull();
  });
});

describe('decodeBinaryLabels', () => {
  it('returns the label section itself, preserving 0 as unlabeled', () => {
    const page = labelPage([2, 0, 0, 7]);
    const labels = decodeBinaryLabels(page)!;
    expect(labels).toBeInstanceOf(Uint32Array);
    expect(Array.from(labels)).toEqual([2, 0, 0, 7]);
    expect(labels).toBe(page.view('label'));
  });

  it('returns null for a geometry page', () => {
    expect(decodeBinaryLabels(parseRuxp(XYZRGB_BUFFER))).toBeNull();
  });
});

describe('toPageBuffers', () => {
  it('folds a binary geometry page and a binary label page into one upload', () => {
    const buffers = toPageBuffers(
      { format: 'binary', page: parseRuxp(XYZRGB_BUFFER) },
      { format: 'binary', page: labelPage([4, 0, 9]) },
    )!;
    expect(Array.from(buffers.positions)).toEqual([1, 2, 3, -4.5, 0, 0.25, 1024, -0.5, 65536]);
    expect(buffers.rgb!.length).toBe(9);
    expect(Array.from(buffers.labels!)).toEqual([4, 0, 9]);
  });

  it('zips across transports — the two clouds need not agree on one', () => {
    // The format latch flips mid-stream if a server starts answering 501, so
    // geometry and labels can legitimately arrive over different transports.
    // They are joined positionally either way (docs/CONTRACTS.md).
    const buffers = toPageBuffers(
      { format: 'binary', page: parseRuxp(XYZRGB_BUFFER) },
      { format: 'json', page: LABEL_POINTS_PAGE },
    )!;
    expect(Array.from(buffers.labels!)).toEqual([2, 0, 0, 0]);
  });

  it('still decodes the pure JSON case', () => {
    const buffers = toPageBuffers({ format: 'json', page: CLOUD_POINTS_PAGE }, null)!;
    expect(buffers.positions.length).toBe(CLOUD_POINTS_PAGE.count * 3);
    expect(buffers.rgb!.length).toBe(CLOUD_POINTS_PAGE.count * 3);
    expect(buffers.labels).toBeNull();
  });

  it('returns null for a page with no renderable geometry', () => {
    const normals = parseRuxp(
      buildRuxpPage({
        count: 1,
        fields: [{ name: 'normal', type: 1, components: 3, values: [0, 0, 1] }],
      }),
    );
    expect(toPageBuffers({ format: 'binary', page: normals }, null)).toBeNull();
  });

  it('leaves rgb null for a PointXYZ page rather than inventing white', () => {
    const xyz = parseRuxp(
      buildRuxpPage({
        count: 1,
        fields: [{ name: 'xyz', type: 1, components: 3, values: [1, 2, 3] }],
      }),
    );
    const buffers = toPageBuffers({ format: 'binary', page: xyz }, null)!;
    expect(buffers.rgb).toBeNull();
  });
});

describe('pageCount / pageTotal', () => {
  it('read the same two numbers out of either transport', () => {
    const binary = { format: 'binary', page: parseRuxp(XYZRGB_BUFFER) } as const;
    expect(pageCount(binary)).toBe(3);
    expect(pageTotal(binary)).toBe(9);

    const json = { format: 'json', page: CLOUD_POINTS_PAGE } as const;
    expect(pageCount(json)).toBe(CLOUD_POINTS_PAGE.count);
    expect(pageTotal(json)).toBe(CLOUD_POINTS_PAGE.total);
  });
});
