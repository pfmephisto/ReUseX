// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Decode a frame's stored label image (`GET /frames/{id}/image?kind=segmentation`).
 *
 * The server sends the storage encoding as a 16-bit greyscale PNG (0 =
 * unlabeled, prompt k = k + 1). A browser cannot read that through `<img>` or
 * a canvas — both keep only the high byte, and every label below 256 becomes
 * 0 — so this decodes the PNG by hand: chunk walk, zlib inflate (injected, the
 * browser's `DecompressionStream` by default), and the five PNG row filters.
 * Only greyscale, non-interlaced, 8- or 16-bit images are accepted; that is
 * all OpenCV writes for a single-channel label map.
 */

export interface LabelImage {
  width: number;
  height: number;
  /** Row-major storage values: 0 = unlabeled, prompt k = k + 1. */
  data: Uint16Array;
}

export type Inflate = (zlib: Uint8Array) => Promise<Uint8Array>;

const SIGNATURE = [137, 80, 78, 71, 13, 10, 26, 10];

export async function inflateWithStream(zlib: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([zlib as BlobPart]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

interface Header {
  width: number;
  height: number;
  bitDepth: number;
  idat: Uint8Array;
}

function readHeader(bytes: Uint8Array): Header {
  if (bytes.length < 8 || SIGNATURE.some((b, i) => bytes[i] !== b)) throw new Error('not a PNG');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let offset = 8;
  let width = 0;
  let height = 0;
  let bitDepth = 0;
  const parts: Uint8Array[] = [];
  while (offset + 8 <= bytes.length) {
    const length = view.getUint32(offset);
    const type = String.fromCharCode(...bytes.subarray(offset + 4, offset + 8));
    const data = bytes.subarray(offset + 8, offset + 8 + length);
    if (type === 'IHDR') {
      width = view.getUint32(offset + 8);
      height = view.getUint32(offset + 12);
      bitDepth = data[8];
      const colourType = data[9];
      const interlace = data[12];
      if (colourType !== 0) throw new Error(`unsupported PNG colour type ${colourType}`);
      if (interlace !== 0) throw new Error('interlaced PNG not supported');
      if (bitDepth !== 8 && bitDepth !== 16) throw new Error(`unsupported PNG bit depth ${bitDepth}`);
    } else if (type === 'IDAT') {
      parts.push(data);
    } else if (type === 'IEND') {
      break;
    }
    offset += 12 + length;
  }
  if (width === 0 || height === 0) throw new Error('PNG has no IHDR');
  const total = parts.reduce((n, p) => n + p.length, 0);
  const idat = new Uint8Array(total);
  let at = 0;
  for (const p of parts) {
    idat.set(p, at);
    at += p.length;
  }
  return { width, height, bitDepth, idat };
}

function paeth(a: number, b: number, c: number): number {
  const p = a + b - c;
  const pa = Math.abs(p - a);
  const pb = Math.abs(p - b);
  const pc = Math.abs(p - c);
  if (pa <= pb && pa <= pc) return a;
  return pb <= pc ? b : c;
}

/** Undo the per-row PNG filters in place of a fresh buffer; `bpp` is bytes per pixel. */
export function unfilter(raw: Uint8Array, width: number, height: number, bpp: number): Uint8Array {
  const stride = width * bpp;
  if (raw.length < height * (stride + 1)) throw new Error('PNG data is truncated');
  const out = new Uint8Array(height * stride);
  for (let y = 0; y < height; y += 1) {
    const filter = raw[y * (stride + 1)];
    const src = y * (stride + 1) + 1;
    const row = y * stride;
    const prev = row - stride;
    for (let x = 0; x < stride; x += 1) {
      const v = raw[src + x];
      const a = x >= bpp ? out[row + x - bpp] : 0;
      const b = y > 0 ? out[prev + x] : 0;
      const c = x >= bpp && y > 0 ? out[prev + x - bpp] : 0;
      let r: number;
      switch (filter) {
        case 0:
          r = v;
          break;
        case 1:
          r = v + a;
          break;
        case 2:
          r = v + b;
          break;
        case 3:
          r = v + ((a + b) >> 1);
          break;
        case 4:
          r = v + paeth(a, b, c);
          break;
        default:
          throw new Error(`bad PNG filter ${filter}`);
      }
      out[row + x] = r & 0xff;
    }
  }
  return out;
}

export async function decodeLabelPng(bytes: Uint8Array, inflate: Inflate = inflateWithStream): Promise<LabelImage> {
  const { width, height, bitDepth, idat } = readHeader(bytes);
  const bpp = bitDepth / 8;
  const pixels = unfilter(await inflate(idat), width, height, bpp);
  const data = new Uint16Array(width * height);
  if (bpp === 2) {
    for (let i = 0; i < data.length; i += 1) data[i] = (pixels[2 * i] << 8) | pixels[2 * i + 1];
  } else {
    data.set(pixels.subarray(0, data.length));
  }
  return { width, height, data };
}

/** Pixel count per prompt index (storage value − 1); unlabeled pixels are not counted. */
export function promptPixelCounts(image: LabelImage): Map<number, number> {
  const counts = new Map<number, number>();
  for (const v of image.data) {
    if (v === 0) continue;
    counts.set(v - 1, (counts.get(v - 1) ?? 0) + 1);
  }
  return counts;
}
