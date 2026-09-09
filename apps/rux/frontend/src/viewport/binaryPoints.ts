// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * RUXP v1 — the binary point transport.
 *
 * `docs/gui/binary-points.md` is normative; this module is a reader for exactly
 * that format and nothing else. The point of RUXP is that a page of points
 * reaches `THREE.BufferAttribute` without being touched per point in
 * JavaScript, so {@link RuxpPage.view} hands back a typed array constructed
 * *directly over the received `ArrayBuffer`* — never a copy, never a loop.
 *
 * ## Little-endian only
 *
 * The header is read through a `DataView` with an explicit `littleEndian: true`,
 * so it decodes correctly anywhere. **The payload does not.** A typed-array view
 * uses the host's byte order and offers no way to override it, so on a
 * big-endian host every `f32` coordinate and every `u32` label in a page would
 * decode as silent garbage — no throw, no clue, just a scrambled cloud.
 *
 * That is the format's deliberate non-goal, not an oversight here
 * (`docs/gui/binary-points.md` § Endianness): every platform `rux gui` and its
 * browser client run on is little-endian. If a big-endian client ever matters,
 * the fix is a `flags` bit advertising byte order plus a byte-swapping slow
 * path — and this parser already refuses any page with an unknown flag set, so
 * such a page cannot be misread by today's code.
 *
 * ## Everything malformed is a throw
 *
 * A truncated, over-long, misaligned or unknown-version page is rejected with a
 * message naming the number that disagreed. The alternative — constructing a
 * view anyway — is either a bare `RangeError` from the typed-array constructor
 * or, worse, a view onto the wrong bytes. A caller that wants to survive a bad
 * page catches this and falls back to `format=json`.
 */

/** Value type of one field's section. The three codes v1 defines. */
export type RuxpFieldType = 'f32' | 'u8' | 'u32';

/** A typed-array view over one field's section, in whatever type it declares. */
export type RuxpView = Float32Array | Uint8Array | Uint32Array;

/** One entry of the header's field table. */
export interface RuxpField {
  /** ASCII, NUL-padding stripped — `xyz`, `rgb`, `normal`, `label`. */
  name: string;
  type: RuxpFieldType;
  /** Values per point: 3 for `xyz`, 1 for `label`. */
  components: number;
  /** Absolute offset of this field's section from the start of the buffer. */
  byteOffset: number;
}

/** A parsed page. Views are lazy but cached, so repeated lookups are free. */
export interface RuxpPage {
  /** Format version. Always 1 — anything else was refused during parsing. */
  version: number;
  /** Points in *this* page. */
  count: number;
  /** Index of this page's first point within the cloud. */
  offset: number;
  /** Points in the whole cloud. */
  total: number;
  /** Field table, in header order. */
  fields: RuxpField[];
  /**
   * Zero-copy view onto a field's section, or `null` when the page has no such
   * field. The same view object is returned on every call for a given name.
   */
  view(name: string): RuxpView | null;
}

/** `"RUXP"` — the four bytes every page starts with. */
const MAGIC = [0x52, 0x55, 0x58, 0x50] as const;

/** Header bytes before the field table. `header_size` is this + 16 per field. */
const FIXED_HEADER_BYTES = 40;
const FIELD_DESCRIPTOR_BYTES = 16;

/** The only version this reader understands. A v2 page must be refused. */
const SUPPORTED_VERSION = 1;

/**
 * Flag bits this reader knows how to honour.
 *
 * v1 defines none, so any bit set is a page written by something that expects
 * the reader to do something it has never heard of — byte-swapping, say, or
 * decoding quantised positions. Refusing is the whole reason the field exists.
 */
const KNOWN_FLAGS = 0;

const TYPE_SIZES: Record<RuxpFieldType, number> = { f32: 4, u8: 1, u32: 4 };

/**
 * Cheap, allocation-free "is this a RUXP page at all?".
 *
 * Safe on a short or empty buffer, because the interesting caller is the one
 * asking whether a `format=binary` response body is really binary — an old
 * server, or a proxy, can answer 200 with something else entirely.
 */
export function isRuxp(buffer: ArrayBuffer): boolean {
  if (buffer.byteLength < MAGIC.length) return false;
  const bytes = new Uint8Array(buffer, 0, MAGIC.length);
  return MAGIC.every((byte, index) => bytes[index] === byte);
}

/**
 * Parse a RUXP page, or throw explaining which rule it broke.
 *
 * Nothing is read out of the payload here: the whole body is validated against
 * the header first, and only then are views handed out. That ordering is what
 * makes {@link RuxpPage.view} unconditionally safe for a caller to use without
 * re-checking bounds.
 */
export function parseRuxp(buffer: ArrayBuffer): RuxpPage {
  if (!isRuxp(buffer)) {
    throw new Error(
      `RUXP: bad magic — expected "RUXP", buffer is ${describeStart(buffer)}`,
    );
  }
  if (buffer.byteLength < FIXED_HEADER_BYTES) {
    throw new Error(
      `RUXP: buffer is ${buffer.byteLength} bytes, too short for the ` +
        `${FIXED_HEADER_BYTES}-byte fixed header`,
    );
  }

  const header = new DataView(buffer);

  const version = header.getUint16(4, true);
  if (version !== SUPPORTED_VERSION) {
    // Refused, not guessed at: a v2 exists precisely because it changed
    // something this code would otherwise misread (`docs/gui/binary-points.md`
    // § "Not in v1").
    throw new Error(`RUXP: unsupported version ${version}, this client reads v${SUPPORTED_VERSION}`);
  }

  const headerSize = header.getUint16(6, true);
  const flags = header.getUint32(8, true);
  const fieldCount = header.getUint32(12, true);
  const count = header.getUint32(16, true);
  const offset = readU64(header, 24, 'offset');
  const total = readU64(header, 32, 'total');

  if (flags !== KNOWN_FLAGS) {
    throw new Error(`RUXP: unknown flags 0x${flags.toString(16)} set; v1 defines none`);
  }

  const expectedHeaderSize = FIXED_HEADER_BYTES + FIELD_DESCRIPTOR_BYTES * fieldCount;
  if (headerSize !== expectedHeaderSize) {
    throw new Error(
      `RUXP: header_size ${headerSize} disagrees with field_count ${fieldCount} ` +
        `(expected ${expectedHeaderSize})`,
    );
  }
  if (headerSize > buffer.byteLength) {
    throw new Error(
      `RUXP: header_size ${headerSize} exceeds the ${buffer.byteLength}-byte buffer`,
    );
  }

  // Pass 1: read the field table and work out how long the body must be.
  //
  // Split from the bounds checking below so a truncated response is diagnosed
  // as a truncated response. Doing both in one loop reports whichever field
  // happens to sit last as "outside the payload", which sends the reader
  // hunting for a layout bug that is not there.
  const fields: RuxpField[] = [];
  const seen = new Set<string>();
  let payloadBytes = 0;

  for (let index = 0; index < fieldCount; index += 1) {
    const base = FIXED_HEADER_BYTES + index * FIELD_DESCRIPTOR_BYTES;
    const name = readName(buffer, base);
    const type = typeFromCode(header.getUint8(base + 8), name);
    const components = header.getUint8(base + 9);
    const byteOffset = header.getUint32(base + 12, true);

    if (components === 0) {
      // A zero-component field contributes no bytes and no meaning, so it
      // cannot be what the server intended to send — and it would hand the
      // caller an empty view for an attribute it believes it received.
      throw new Error(`RUXP: field '${name}' declares 0 components`);
    }
    if (seen.has(name)) {
      throw new Error(`RUXP: duplicate field '${name}' — view() would be ambiguous`);
    }
    seen.add(name);

    payloadBytes += count * components * TYPE_SIZES[type];
    fields.push({ name, type, components, byteOffset });
  }

  // Both directions are protocol errors. Short would mean reading off the end;
  // long means the sender and this reader disagree about the layout, and the
  // bytes that agree cannot be told from the bytes that do not.
  const bodyLength = headerSize + payloadBytes;
  if (bodyLength !== buffer.byteLength) {
    throw new Error(
      `RUXP: body_length ${bodyLength} (header ${headerSize} + payload ${payloadBytes}) ` +
        `does not match the received ${buffer.byteLength} bytes`,
    );
  }

  // Pass 2: every section must lie inside the payload and be legally aligned.
  // The body is known to be the right *size* by here, so anything caught below
  // is a header that misplaces a section within it.
  for (const field of fields) {
    const sectionBytes = count * field.components * TYPE_SIZES[field.type];
    const end = field.byteOffset + sectionBytes;
    if (field.byteOffset < headerSize || end > buffer.byteLength) {
      throw new Error(
        `RUXP: field '${field.name}' section [${field.byteOffset}, ${end}) ` +
          `is outside the payload [${headerSize}, ${buffer.byteLength})`,
      );
    }
    // Checked rather than left to the typed-array constructor, which throws a
    // bare `RangeError` naming neither the field nor the format. The layout
    // guarantees alignment (`header_size` is a multiple of 8 and every f32/u32
    // section is 4-byte sized), so tripping this means the page is malformed.
    const alignment = TYPE_SIZES[field.type];
    if (field.byteOffset % alignment !== 0) {
      throw new Error(
        `RUXP: field '${field.name}' byte_offset ${field.byteOffset} is not ` +
          `${alignment}-byte aligned`,
      );
    }
  }

  const views = new Map<string, RuxpView>();
  return {
    version,
    count,
    offset,
    total,
    fields,
    view(name: string): RuxpView | null {
      const cached = views.get(name);
      if (cached) return cached;
      const field = fields.find((entry) => entry.name === name);
      if (!field) return null;
      const view = makeView(buffer, field, count);
      views.set(name, view);
      return view;
    },
  };
}

/** The field named `name`, or `null`. Lets a caller check a type before viewing. */
export function ruxpField(page: RuxpPage, name: string): RuxpField | null {
  return page.fields.find((field) => field.name === name) ?? null;
}

/**
 * The zero-copy step, and the only reason this format is planar.
 *
 * Every bound and every alignment constraint was checked before this ran, so
 * the constructor cannot throw here.
 */
function makeView(buffer: ArrayBuffer, field: RuxpField, count: number): RuxpView {
  const length = count * field.components;
  switch (field.type) {
    case 'f32':
      return new Float32Array(buffer, field.byteOffset, length);
    case 'u8':
      return new Uint8Array(buffer, field.byteOffset, length);
    case 'u32':
      return new Uint32Array(buffer, field.byteOffset, length);
  }
}

function typeFromCode(code: number, fieldName: string): RuxpFieldType {
  switch (code) {
    case 1:
      return 'f32';
    case 2:
      return 'u8';
    case 3:
      return 'u32';
    default:
      throw new Error(`RUXP: field '${fieldName}' has unknown type code ${code}`);
  }
}

/** ASCII name out of an 8-byte, NUL-padded (never NUL-terminated at 8) slot. */
function readName(buffer: ArrayBuffer, byteOffset: number): string {
  const bytes = new Uint8Array(buffer, byteOffset, 8);
  let end = 0;
  while (end < bytes.length && bytes[end] !== 0) end += 1;
  return String.fromCharCode(...bytes.subarray(0, end));
}

/**
 * A `u64` header field, as a `number`.
 *
 * Read as two `u32`s rather than with `getBigUint64`. Every consumer of
 * `offset`/`total` does `number` arithmetic — page planning, a progress
 * fraction — so a `BigInt` would be converted away immediately, and the pair of
 * `u32`s makes the range guard exact instead of approximate: a value that does
 * not fit a double's 53-bit mantissa is precisely one whose high word exceeds
 * 2^21 - 1.
 */
function readU64(header: DataView, byteOffset: number, what: string): number {
  const low = header.getUint32(byteOffset, true);
  const high = header.getUint32(byteOffset + 4, true);
  if (high > 0x1f_ffff) {
    throw new Error(`RUXP: ${what} exceeds Number.MAX_SAFE_INTEGER (high word 0x${high.toString(16)})`);
  }
  return high * 2 ** 32 + low;
}

/** First bytes of a non-RUXP buffer, for an error message worth reading. */
function describeStart(buffer: ArrayBuffer): string {
  if (buffer.byteLength === 0) return 'empty';
  const bytes = new Uint8Array(buffer, 0, Math.min(4, buffer.byteLength));
  const hex = Array.from(bytes, (byte) => byte.toString(16).padStart(2, '0')).join(' ');
  return `[${hex}]`;
}
