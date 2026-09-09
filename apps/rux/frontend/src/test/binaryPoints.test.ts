// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The RUXP v1 reader (`docs/gui/binary-points.md`, #283).
 *
 * Two properties are being pinned, and they pull in opposite directions.
 *
 * A well-formed page must reach the caller as a **zero-copy** view — that is the
 * entire reason the format exists, so the tests assert on
 * `view.buffer === buffer`, not merely on the numbers coming out right.
 *
 * A malformed page must reach the caller as a **throw**, never as a view onto
 * the wrong bytes. Each rejection below is one way a server, a proxy or a
 * version mismatch can hand this code a buffer whose header disagrees with its
 * payload, and every one of them is silently renderable garbage if unchecked.
 *
 * Fixtures are built byte by byte in `ruxpPage.ts`; nothing here constructs its
 * input with the parser it is testing.
 */

import { describe, expect, it } from 'vitest';
import { isRuxp, parseRuxp, ruxpField } from '../viewport/binaryPoints';
import {
  FIELD_DESCRIPTOR_BYTES,
  FIXED_HEADER_BYTES,
  XYZRGB_THREE_POINTS,
  buildRuxpPage,
  resized,
} from './ruxpPage';

const HEADER_TWO_FIELDS = FIXED_HEADER_BYTES + 2 * FIELD_DESCRIPTOR_BYTES; // 72
const HEADER_ONE_FIELD = FIXED_HEADER_BYTES + FIELD_DESCRIPTOR_BYTES; // 56

describe('isRuxp', () => {
  it('accepts a page and rejects anything else, without reading past 4 bytes', () => {
    expect(isRuxp(buildRuxpPage(XYZRGB_THREE_POINTS))).toBe(true);
    // The two things a `format=binary` request can actually come back as when
    // the server has never heard of RUXP: a JSON body, or nothing at all.
    expect(isRuxp(new TextEncoder().encode('{"name":"cloud"}').buffer as ArrayBuffer)).toBe(false);
    expect(isRuxp(new ArrayBuffer(0))).toBe(false);
    expect(isRuxp(new ArrayBuffer(3))).toBe(false);
  });
});

describe('parseRuxp — PointXYZRGB', () => {
  const buffer = buildRuxpPage(XYZRGB_THREE_POINTS);
  const page = parseRuxp(buffer);

  it('decodes the header', () => {
    expect(page.version).toBe(1);
    expect(page.count).toBe(3);
    expect(page.offset).toBe(100);
    expect(page.total).toBe(9);
  });

  it('decodes the field table in header order', () => {
    expect(page.fields).toEqual([
      { name: 'xyz', type: 'f32', components: 3, byteOffset: HEADER_TWO_FIELDS },
      { name: 'rgb', type: 'u8', components: 3, byteOffset: HEADER_TWO_FIELDS + 3 * 3 * 4 },
    ]);
    expect(ruxpField(page, 'xyz')?.type).toBe('f32');
    expect(ruxpField(page, 'curvature')).toBeNull();
  });

  it('returns exact positions', () => {
    const xyz = page.view('xyz');
    expect(xyz).toBeInstanceOf(Float32Array);
    expect(Array.from(xyz!)).toEqual([1, 2, 3, -4.5, 0, 0.25, 1024, -0.5, 65536]);
  });

  it('returns the rgb bytes verbatim, in r,g,b order', () => {
    // The server already swizzled out of PCL's BGRA word, so what arrives is
    // r,g,b — 0..255 sRGB samples, untouched. The sRGB->linear transfer is
    // `decode.ts`'s job, deliberately not this module's.
    const rgb = page.view('rgb');
    expect(rgb).toBeInstanceOf(Uint8Array);
    expect(Array.from(rgb!)).toEqual([0, 128, 255, 126, 130, 118, 1, 2, 3]);
  });

  it('hands back views onto the received buffer, not copies', () => {
    // Its own buffer: this test writes through it, and the assertions above
    // must not depend on the order vitest happens to run them in.
    const own = buildRuxpPage(XYZRGB_THREE_POINTS);
    const parsed = parseRuxp(own);
    const xyz = parsed.view('xyz')!;
    const rgb = parsed.view('rgb')!;
    expect(xyz.buffer).toBe(own);
    expect(rgb.buffer).toBe(own);
    expect(xyz.byteOffset).toBe(HEADER_TWO_FIELDS);
    expect(rgb.byteOffset).toBe(HEADER_TWO_FIELDS + 36);
    // A copy would pass every assertion above while doubling what a stream
    // costs in memory, so aliasing is asserted rather than assumed.
    new Uint8Array(own)[HEADER_TWO_FIELDS + 36] = 200;
    expect(rgb[0]).toBe(200);
  });

  it('caches the view, so a per-frame lookup allocates nothing', () => {
    const first = page.view('xyz');
    expect(page.view('xyz')).toBe(first);
  });

  it('returns null for a field the page does not carry', () => {
    expect(page.view('label')).toBeNull();
    expect(page.view('normal')).toBeNull();
  });
});

describe('parseRuxp — the other cloud types', () => {
  it('reads a PointXYZ page', () => {
    const page = parseRuxp(
      buildRuxpPage({
        count: 2,
        total: 2,
        fields: [{ name: 'xyz', type: 1, components: 3, values: [1, 2, 3, 4, 5, 6] }],
      }),
    );
    expect(page.fields).toHaveLength(1);
    expect(Array.from(page.view('xyz')!)).toEqual([1, 2, 3, 4, 5, 6]);
    expect(page.view('rgb')).toBeNull();
  });

  it('reads a Normal page — and calls the field `normal`, not `xyz`', () => {
    const page = parseRuxp(
      buildRuxpPage({
        count: 2,
        total: 2,
        fields: [{ name: 'normal', type: 1, components: 3, values: [0, 0, 1, 0, -1, 0] }],
      }),
    );
    // The distinction matters: a normals page named `xyz` would be rendered as
    // a unit-sphere-shaped point cloud sitting at the origin.
    expect(page.view('xyz')).toBeNull();
    expect(Array.from(page.view('normal')!)).toEqual([0, 0, 1, 0, -1, 0]);
  });

  it('reads a Label page as u32, keeping ids above 2^31 unsigned', () => {
    const page = parseRuxp(
      buildRuxpPage({
        count: 4,
        total: 4,
        fields: [{ name: 'label', type: 3, components: 1, values: [0, 2, 65535, 4294967295] }],
      }),
    );
    const labels = page.view('label');
    expect(labels).toBeInstanceOf(Uint32Array);
    // 0 stays 0 (unlabeled, STANDARDS §3) and the top of the range does not
    // come back as -1.
    expect(Array.from(labels!)).toEqual([0, 2, 65535, 4294967295]);
  });
});

describe('parseRuxp — the empty page', () => {
  // A page past the end of a cloud is a 200 with count 0, not a 404: it is how
  // a client discovers a cloud is shorter than it thought without a second
  // route. The parser must accept it, not treat it as truncation.
  const buffer = buildRuxpPage({
    count: 0,
    offset: 500,
    total: 500,
    fields: [
      { name: 'xyz', type: 1, components: 3 },
      { name: 'rgb', type: 2, components: 3 },
    ],
  });

  it('is exactly header_size bytes', () => {
    expect(buffer.byteLength).toBe(HEADER_TWO_FIELDS);
  });

  it('parses, with zero-length views and the cloud total intact', () => {
    const page = parseRuxp(buffer);
    expect(page.count).toBe(0);
    expect(page.offset).toBe(500);
    expect(page.total).toBe(500);
    expect(page.view('xyz')!.length).toBe(0);
    expect(page.view('rgb')!.length).toBe(0);
    expect(page.fields).toHaveLength(2);
  });
});

describe('parseRuxp — rejections', () => {
  it('rejects a body that is not RUXP at all', () => {
    const json = new TextEncoder().encode('{"name":"cloud","count":0}').buffer as ArrayBuffer;
    expect(() => parseRuxp(json)).toThrow(/bad magic/);
    expect(() => parseRuxp(new ArrayBuffer(0))).toThrow(/bad magic/);
    // One byte off is the interesting near-miss.
    expect(() => parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, magic: 'RUXQ' }))).toThrow(
      /bad magic/,
    );
  });

  it('rejects a version it does not know', () => {
    // A v2 exists precisely because it changed something this reader would
    // misread. Guessing is worse than refusing.
    expect(() => parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, version: 2 }))).toThrow(
      /unsupported version 2/,
    );
    expect(() => parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, version: 0 }))).toThrow(
      /unsupported version 0/,
    );
  });

  it('rejects unknown flag bits', () => {
    // v1 defines none, so any bit set means the sender expects a decode step
    // this reader has never heard of.
    expect(() => parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, flags: 1 }))).toThrow(
      /unknown flags/,
    );
    expect(() => parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, flags: 0x8000_0000 }))).toThrow(
      /unknown flags/,
    );
  });

  it('rejects a header_size that disagrees with field_count', () => {
    expect(() =>
      parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, headerSize: HEADER_ONE_FIELD })),
    ).toThrow(/header_size 56 disagrees with field_count 2/);
    expect(() =>
      parseRuxp(buildRuxpPage({ ...XYZRGB_THREE_POINTS, fieldCount: 5 })),
    ).toThrow(/field_count 5/);
  });

  it('rejects a buffer too short to hold the header it claims', () => {
    // Magic and nothing else — what a truncated response looks like.
    const stub = new ArrayBuffer(4);
    new Uint8Array(stub).set([0x52, 0x55, 0x58, 0x50]);
    expect(() => parseRuxp(stub)).toThrow(/too short for the 40-byte fixed header/);

    const oneField = buildRuxpPage({
      count: 0,
      fields: [{ name: 'label', type: 3, components: 1 }],
    });
    expect(() => parseRuxp(resized(oneField, -8))).toThrow(/header_size 56 exceeds/);
  });

  it('rejects a body one byte short', () => {
    // The case the whole length check exists for: a view built over a short
    // buffer either throws a bare RangeError or reads whatever follows it.
    expect(() => parseRuxp(resized(buildRuxpPage(XYZRGB_THREE_POINTS), -1))).toThrow(
      /does not match the received/,
    );
  });

  it('rejects a body one byte long', () => {
    // Less obviously wrong and just as fatal: sender and reader disagree about
    // the layout, and the bytes that agree cannot be told from those that do not.
    expect(() => parseRuxp(resized(buildRuxpPage(XYZRGB_THREE_POINTS), 1))).toThrow(
      /does not match the received/,
    );
  });

  it('rejects an unknown field type code', () => {
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [{ name: 'xyz', type: 1, components: 3, typeByte: 7, values: [1, 2, 3] }],
        }),
      ),
    ).toThrow(/unknown type code 7/);
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [{ name: 'xyz', type: 1, components: 3, typeByte: 0, values: [1, 2, 3] }],
        }),
      ),
    ).toThrow(/unknown type code 0/);
  });

  it('rejects a byte_offset that is not 4-byte aligned for an f32 section', () => {
    // `new Float32Array(buffer, 73, 3)` is a bare RangeError naming neither the
    // field nor the format, so the parser checks it and says which field.
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [
            { name: 'xyz', type: 1, components: 3, byteOffsetByte: 73, values: [1, 2, 3] },
            { name: 'rgb', type: 2, components: 3, values: [1, 2, 3] },
          ],
        }),
      ),
    ).toThrow(/byte_offset 73 is not 4-byte aligned/);
  });

  it('rejects a byte_offset whose section runs off the end', () => {
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [{ name: 'xyz', type: 1, components: 3, byteOffsetByte: 60, values: [1, 2, 3] }],
        }),
      ),
    ).toThrow(/is outside the payload/);
  });

  it('rejects a byte_offset that points into the header', () => {
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [{ name: 'xyz', type: 1, components: 3, byteOffsetByte: 40, values: [1, 2, 3] }],
        }),
      ),
    ).toThrow(/is outside the payload/);
  });

  it('rejects a zero-component field', () => {
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [{ name: 'xyz', type: 1, components: 3, componentsByte: 0, values: [1, 2, 3] }],
        }),
      ),
    ).toThrow(/declares 0 components/);
  });

  it('rejects duplicate field names', () => {
    // `view('xyz')` would have to pick one, and either choice is wrong.
    expect(() =>
      parseRuxp(
        buildRuxpPage({
          count: 1,
          fields: [
            { name: 'xyz', type: 1, components: 3, values: [1, 2, 3] },
            { name: 'xyz', type: 1, components: 3, values: [4, 5, 6] },
          ],
        }),
      ),
    ).toThrow(/duplicate field/);
  });
});

describe('parseRuxp — the little-endian assumption', () => {
  /**
   * One page written out as literal bytes, with no builder in the way.
   *
   * This is the test that would catch a `littleEndian` argument dropped from a
   * `DataView` call — the builder shares the parser's assumption about which
   * end a number starts at, and a literal byte array does not. Numbers were
   * chosen so a big-endian or signed misread is a wildly different value, not
   * an off-by-one: `offset`'s low word has its top bit set, and `total` uses
   * both words.
   */
  const bytes = new Uint8Array([
    // magic "RUXP"
    0x52, 0x55, 0x58, 0x50,
    // version = 1
    0x01, 0x00,
    // header_size = 56
    0x38, 0x00,
    // flags = 0
    0x00, 0x00, 0x00, 0x00,
    // field_count = 1
    0x01, 0x00, 0x00, 0x00,
    // count = 1
    0x01, 0x00, 0x00, 0x00,
    // reserved
    0x00, 0x00, 0x00, 0x00,
    // offset = 0x0000000087654321
    0x21, 0x43, 0x65, 0x87, 0x00, 0x00, 0x00, 0x00,
    // total = 0x0000000504030201
    0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00,
    // field "xyz", f32 x 3, byte_offset 56
    0x78, 0x79, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
    // 1.0, -2.5, 0.5 as little-endian IEEE-754 binary32
    0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x20, 0xc0, 0x00, 0x00, 0x00, 0x3f,
  ]);

  it('reads header words little-endian and u64s unsigned', () => {
    expect(bytes.byteLength).toBe(68);
    const page = parseRuxp(bytes.buffer as ArrayBuffer);
    expect(page.version).toBe(1);
    expect(page.count).toBe(1);
    // 0x87654321. A signed read gives -2023406815; a big-endian one 0x21436587.
    expect(page.offset).toBe(2271560481);
    // 5 * 2^32 + 0x04030201 — a value no single u32 can hold, so it also pins
    // that the high word is combined in rather than dropped.
    expect(page.total).toBe(21542142465);
    expect(page.fields[0]).toEqual({ name: 'xyz', type: 'f32', components: 3, byteOffset: 56 });
  });

  it('reads the payload little-endian (which is host order, not a choice)', () => {
    const page = parseRuxp(bytes.buffer as ArrayBuffer);
    expect(Array.from(page.view('xyz')!)).toEqual([1, -2.5, 0.5]);
  });

  it('refuses a u64 too large to be an exact Number', () => {
    // Silently losing precision on `total` would make the page plan drift, so
    // the guard is exact: the high word may not exceed 2^21 - 1.
    const tooBig = bytes.slice();
    tooBig[32 + 7] = 0xff; // total's top byte
    expect(() => parseRuxp(tooBig.buffer as ArrayBuffer)).toThrow(
      /total exceeds Number.MAX_SAFE_INTEGER/,
    );
  });
});
