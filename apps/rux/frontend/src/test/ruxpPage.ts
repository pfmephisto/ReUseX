// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Hand-built RUXP v1 pages, for testing the parser.
 *
 * Deliberately **not** in `fixtures.ts`. That module holds recordings of what a
 * real `rux gui` returned and must never be hand-edited; this one is the
 * opposite — every page here is synthesised byte by byte precisely so it can be
 * made *wrong*, which a recording by definition cannot be.
 *
 * Every byte is written through a `DataView` against
 * `docs/gui/binary-points.md` directly. Nothing here calls `parseRuxp`, or the
 * tests would be checking the parser against itself: a reader and a writer that
 * share one bug agree perfectly.
 *
 * The payload is written with `DataView`'s explicit `littleEndian: true` rather
 * than by filling a `Float32Array`, so these buffers are byte-identical on any
 * host — the same guarantee the server gives. (The *parser* still reads the
 * payload through typed-array views and so is correct only on a little-endian
 * host; that is the format's documented non-goal, and
 * `binaryPoints.test.ts` pins it explicitly.)
 */

/** `1` = f32, `2` = u8, `3` = u32 — the three codes v1 defines. */
export type RuxpTypeCode = 1 | 2 | 3;

const TYPE_SIZE: Record<RuxpTypeCode, number> = { 1: 4, 2: 1, 3: 4 };

export const FIXED_HEADER_BYTES = 40;
export const FIELD_DESCRIPTOR_BYTES = 16;

export interface FieldFixture {
  /** ASCII, at most 8 bytes. Written NUL-padded. */
  name: string;
  /** Governs both the written type byte and the section's width. */
  type: RuxpTypeCode;
  components: number;
  /** `count * components` values in point order; a short list leaves zeros. */
  values?: number[];

  // ---- corruption knobs: each changes one header byte and nothing else ----

  /** Type byte to write instead of `type`. The layout still follows `type`. */
  typeByte?: number;
  /** `byte_offset` to write instead of where the section really sits. */
  byteOffsetByte?: number;
  /** `components` byte to write instead of `components`. */
  componentsByte?: number;
}

export interface PageFixture {
  /** Points in this page. Drives every section's length. */
  count: number;
  fields: FieldFixture[];
  offset?: number;
  total?: number;

  // ---- corruption knobs ----

  /** Four bytes to write instead of `"RUXP"`. */
  magic?: string;
  version?: number;
  /** `header_size` to write instead of `40 + 16 * fields.length`. */
  headerSize?: number;
  flags?: number;
  /** `field_count` to write instead of `fields.length`. */
  fieldCount?: number;
}

/** Where each field's section really starts, and the true body length. */
function layout(spec: PageFixture): { sections: number[]; bodyLength: number } {
  const headerSize = FIXED_HEADER_BYTES + FIELD_DESCRIPTOR_BYTES * spec.fields.length;
  const sections: number[] = [];
  let cursor = headerSize;
  for (const field of spec.fields) {
    sections.push(cursor);
    cursor += spec.count * field.components * TYPE_SIZE[field.type];
  }
  return { sections, bodyLength: cursor };
}

/** Little-endian `u64`, as two `u32`s — `DataView` has no unsigned 64-bit setter. */
function setU64(view: DataView, at: number, value: number): void {
  view.setUint32(at, value >>> 0, true);
  view.setUint32(at + 4, Math.floor(value / 2 ** 32), true);
}

/**
 * A RUXP page, laid out exactly as the server lays one out unless a knob says
 * otherwise.
 *
 * The sections always sit where the format says they sit; the knobs only
 * falsify what the *header* claims about them. That is the interesting kind of
 * malformed page — one a buggy server would actually produce.
 */
export function buildRuxpPage(spec: PageFixture): ArrayBuffer {
  const { sections, bodyLength } = layout(spec);
  const headerSize = FIXED_HEADER_BYTES + FIELD_DESCRIPTOR_BYTES * spec.fields.length;

  const buffer = new ArrayBuffer(bodyLength);
  const view = new DataView(buffer);

  const magic = spec.magic ?? 'RUXP';
  for (let i = 0; i < 4; i += 1) view.setUint8(i, magic.charCodeAt(i) & 0xff);
  view.setUint16(4, spec.version ?? 1, true);
  view.setUint16(6, spec.headerSize ?? headerSize, true);
  view.setUint32(8, spec.flags ?? 0, true);
  view.setUint32(12, spec.fieldCount ?? spec.fields.length, true);
  view.setUint32(16, spec.count, true);
  view.setUint32(20, 0, true); // reserved
  setU64(view, 24, spec.offset ?? 0);
  setU64(view, 32, spec.total ?? spec.count);

  spec.fields.forEach((field, index) => {
    const at = FIXED_HEADER_BYTES + FIELD_DESCRIPTOR_BYTES * index;
    for (let i = 0; i < Math.min(field.name.length, 8); i += 1) {
      view.setUint8(at + i, field.name.charCodeAt(i) & 0xff);
    }
    view.setUint8(at + 8, field.typeByte ?? field.type);
    view.setUint8(at + 9, field.componentsByte ?? field.components);
    view.setUint16(at + 10, 0, true); // reserved
    view.setUint32(at + 12, field.byteOffsetByte ?? sections[index], true);
  });

  spec.fields.forEach((field, index) => {
    let at = sections[index];
    for (const value of field.values ?? []) {
      if (field.type === 1) {
        view.setFloat32(at, value, true);
        at += 4;
      } else if (field.type === 2) {
        view.setUint8(at, value);
        at += 1;
      } else {
        view.setUint32(at, value, true);
        at += 4;
      }
    }
  });

  return buffer;
}

/**
 * The same page, `delta` bytes longer or shorter.
 *
 * Truncating leaves the header claiming a length the buffer no longer has,
 * which is the "one byte short" case; growing is the "one byte long" case. Both
 * are protocol errors, and only one of them is the obvious one.
 */
export function resized(buffer: ArrayBuffer, delta: number): ArrayBuffer {
  const out = new ArrayBuffer(buffer.byteLength + delta);
  const keep = Math.min(buffer.byteLength, out.byteLength);
  new Uint8Array(out).set(new Uint8Array(buffer, 0, keep));
  return out;
}

/** A well-formed 3-point `PointXYZRGB` page, the shape the viewport sees most. */
export const XYZRGB_THREE_POINTS: PageFixture = {
  count: 3,
  offset: 100,
  total: 9,
  fields: [
    {
      name: 'xyz',
      type: 1,
      components: 3,
      // Exact in f32 (all are sums of powers of two), so the assertions can be
      // equalities rather than tolerances.
      values: [1, 2, 3, -4.5, 0, 0.25, 1024, -0.5, 65536],
    },
    { name: 'rgb', type: 2, components: 3, values: [0, 128, 255, 126, 130, 118, 1, 2, 3] },
  ],
};
