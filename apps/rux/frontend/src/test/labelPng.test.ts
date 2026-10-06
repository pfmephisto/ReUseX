// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { deflateSync } from 'node:zlib';
import { describe, expect, it } from 'vitest';

import { decodeLabelPng, inflateWithStream, promptPixelCounts, unfilter } from '../data/labelPng';

function chunk(type: string, data: Uint8Array): Uint8Array {
  const out = new Uint8Array(12 + data.length);
  const view = new DataView(out.buffer);
  view.setUint32(0, data.length);
  out.set([...type].map((c) => c.charCodeAt(0)), 4);
  out.set(data, 8);
  // CRC is not checked by the decoder; leave it zero.
  return out;
}

/** A greyscale PNG with every row unfiltered (filter 0). */
function png(width: number, height: number, bitDepth: 8 | 16, values: number[]): Uint8Array {
  const bpp = bitDepth / 8;
  const raw = new Uint8Array(height * (1 + width * bpp));
  for (let y = 0; y < height; y += 1) {
    const row = y * (1 + width * bpp);
    raw[row] = 0;
    for (let x = 0; x < width; x += 1) {
      const v = values[y * width + x];
      if (bpp === 2) {
        raw[row + 1 + 2 * x] = v >> 8;
        raw[row + 2 + 2 * x] = v & 0xff;
      } else raw[row + 1 + x] = v;
    }
  }
  const ihdr = new Uint8Array(13);
  const v = new DataView(ihdr.buffer);
  v.setUint32(0, width);
  v.setUint32(4, height);
  ihdr[8] = bitDepth;
  const parts = [
    new Uint8Array([137, 80, 78, 71, 13, 10, 26, 10]),
    chunk('IHDR', ihdr),
    chunk('IDAT', new Uint8Array(deflateSync(raw))),
    chunk('IEND', new Uint8Array()),
  ];
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
}

describe('decodeLabelPng', () => {
  it('reads 16-bit storage values exactly, including values a browser would flatten to 0', async () => {
    const values = [0, 1, 2, 0, 1, 300];
    const image = await decodeLabelPng(png(3, 2, 16, values));
    expect(image.width).toBe(3);
    expect(image.height).toBe(2);
    expect([...image.data]).toEqual(values);
  });

  it('reads 8-bit greyscale too', async () => {
    const image = await decodeLabelPng(png(2, 1, 8, [0, 7]));
    expect([...image.data]).toEqual([0, 7]);
  });

  it('refuses what is not a greyscale PNG', async () => {
    await expect(decodeLabelPng(new Uint8Array([1, 2, 3]))).rejects.toThrow('not a PNG');
  });

  it('inflates with the platform DecompressionStream', async () => {
    const out = await inflateWithStream(new Uint8Array(deflateSync(new Uint8Array([5, 6, 7]))));
    expect([...out]).toEqual([5, 6, 7]);
  });
});

describe('unfilter', () => {
  it('undoes Sub, Up, Average and Paeth rows', () => {
    // 2x4 image, 1 byte per pixel. Row 0 plain: 10 20; then each filter type.
    const raw = new Uint8Array([
      0, 10, 20, // none
      1, 30, 5, // sub: 30, 35
      2, 1, 1, // up: 31, 36
      3, 4, 4, // avg: 4 + (0+31)/2 = 19, 4 + (19+36)/2 = 31
      4, 1, 1, // paeth: a=0,b=19,c=0 -> 19+1=20; a=20,b=31,c=19 -> p=32 -> b=31 -> 32
    ]);
    expect([...unfilter(raw, 2, 5, 1)]).toEqual([10, 20, 30, 35, 31, 36, 19, 31, 20, 32]);
  });
});

describe('promptPixelCounts', () => {
  it('counts pixels per prompt index, skipping unlabeled', () => {
    const counts = promptPixelCounts({ width: 5, height: 1, data: new Uint16Array([0, 1, 1, 3, 0]) });
    expect([...counts.entries()].sort()).toEqual([
      [0, 2],
      [2, 1],
    ]);
  });
});
