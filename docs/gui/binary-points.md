<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# RUXP — binary point transport

`GET /api/v1/clouds/{name}/points?format=binary` answers with **RUXP v1**: a
self-describing binary page of point data, served as
`Content-Type: application/octet-stream`.

It exists so the viewport can turn a page of points into `THREE.BufferAttribute`s
without touching a single point in JavaScript, and so the server can serve that
page without materialising the cloud it came from.

Everything on this page is normative. The JSON path
(`format=json`, the default) is unchanged and stays supported for
debuggability — see `docs/gui/openapi.yaml`.

## Why this shape

Two choices were open in [#283](https://github.com/pfmephisto/ReUseX/issues/283),
and both are settled here.

**The metadata lives in the body, not in HTTP headers.** A RUXP page is a
complete artefact: `curl -o page.ruxp` gives you something that can be replayed,
diffed, or handed to a test fixture without a sidecar of header values, and no
intermediary can strip half of it. The same numbers *are* mirrored into
`X-Ruxp-*` response headers, but purely as a `curl`-level convenience — the body
header is the contract, and a client must not depend on the HTTP headers being
present.

**The payload is planar (structure-of-arrays), not interleaved.** Planar means
each attribute is one contiguous run, so the client constructs a typed-array
view straight onto it:

```js
const xyz = new Float32Array(buffer, xyzField.byteOffset, count * 3);
const rgb = new Uint8Array(buffer, rgbField.byteOffset, count * 3);
```

Zero copies, zero per-point work, and both go directly into
`new THREE.BufferAttribute(...)`. An interleaved 16-byte record would have let
the server `memcpy` its storage blob almost verbatim, but it pushes the cost
onto the client — `InterleavedBufferAttribute` plumbing, and two differently
typed views over one buffer with a shared stride. Per-point work in C++ on the
server is cheap; per-point work in JS on the client is the thing this format
exists to delete.

## Layout

All multi-byte integers and all floats are **little-endian**. See
[Endianness](#endianness).

### Header

The header is `header_size` bytes and is followed immediately by the payload.

| offset | size | type | field |
|--------|------|------|-------|
| 0 | 4 | `char[4]` | magic, `"RUXP"` (`0x52 0x55 0x58 0x50`) |
| 4 | 2 | `u16` | `version` — 1 |
| 6 | 2 | `u16` | `header_size` — `40 + 16 * field_count`; also the byte offset at which the payload starts |
| 8 | 4 | `u32` | `flags` — 0 in v1; a client must reject a page with unknown flags set |
| 12 | 4 | `u32` | `field_count` |
| 16 | 4 | `u32` | `count` — points **in this page** |
| 20 | 4 | `u32` | reserved, 0 |
| 24 | 8 | `u64` | `offset` — index of the first point of this page within the cloud |
| 32 | 8 | `u64` | `total` — points in the whole cloud |
| 40 | `16 * field_count` | field[] | field descriptors, see below |

`header_size` is a `u16`, so `field_count` can never exceed 4093. No cloud type
comes close (the largest table today is two fields); an encoder that would
overflow it must fail loudly rather than truncate.

`flags` is the **only** forward-compatibility trapdoor. The reserved `u32` at
+20 and the reserved `u16` at descriptor +10 must be written as 0 and **ignored**
by a reader — a reader that rejected them would break on a future version that
is otherwise readable. If a later version needs to signal something a v1 reader
must not ignore, it sets a `flags` bit (which v1 readers reject) or bumps
`version`.

### Field descriptor

Each descriptor is 16 bytes:

| offset | size | type | field |
|--------|------|------|-------|
| 0 | 8 | `char[8]` | `name`, ASCII, NUL-padded (never NUL-terminated when 8 chars long) |

| 8 | 1 | `u8` | `type` — `1` = `f32`, `2` = `u8`, `3` = `u32` |
| 9 | 1 | `u8` | `components` — values per point |
| 10 | 2 | `u16` | reserved, 0 |
| 12 | 4 | `u32` | `byte_offset` — **absolute** offset of this field's section from the start of the buffer |

`byte_offset` is absolute rather than payload-relative so the client needs no
arithmetic to build a view, and so a section can be relocated in a future
version without changing the reader.

A field's section is `count * components * sizeof(type)` bytes of tightly packed
values in point order. There is no padding between sections.

`name` is unique within a page — readers key sections by name, so a duplicate is
ambiguous and must be rejected. A reader stops at the first NUL; an encoder
writes zeros for the remaining bytes.

On a `count = 0` page every field's `byte_offset` equals `header_size`: the
sections are all empty, so they all start where the payload would have. A reader
must not require the offsets to be distinct or to be strictly inside the body.

### Fields by cloud type

At parity with the JSON `fields` array — the same information, regrouped:

| Cloud type | RUXP fields | JSON `fields` | bytes/point |
|---|---|---|---|
| `PointXYZRGB` | `xyz` f32×3, `rgb` u8×3 | `x,y,z,r,g,b` | 15 |
| `PointXYZ` | `xyz` f32×3 | `x,y,z` | 12 |
| `Normal` | `normal` f32×3 | `nx,ny,nz` | 12 |
| `Label` | `label` u32×1 | `label` | 4 |

`rgb` bytes are **sRGB** samples in `r, g, b` order, 0..255 — the same values the
JSON path emits as integers. Note that this is *not* the byte order of the
underlying `pcl::PointXYZRGB::rgba` word (which is BGRA on a little-endian
host); the server swizzles. A client that wants three.js colours must still
apply the sRGB→linear transfer, exactly as the JSON path does.

`Normal` clouds carry a `curvature` value in storage that neither the JSON nor
the RUXP path exposes; if it is ever needed it becomes an additional RUXP field
and an additional JSON column, which is why both sides are field-table driven.

### Total length

```
body_length = header_size + Σ over fields ( count * components * sizeof(type) )
```

A client **must** check the received `ArrayBuffer` against this and reject a
short buffer rather than construct a view that runs off the end. A buffer longer
than `body_length` is also a protocol error in v1.

### Alignment

`header_size` is `40 + 16 * field_count`, so it is always a multiple of 8, and
every `f32`/`u32` section in the table above starts on a 4-byte boundary. This
is what makes the zero-copy `new Float32Array(buffer, byteOffset, …)` legal —
that constructor throws a `RangeError` on a misaligned offset. Any future field
layout must preserve it.

### Endianness

**Little-endian only.** The header is read with `DataView` and an explicit
`littleEndian: true`, but the payload is read with typed-array views, which use
the *host's* byte order and offer no way to override it. On a big-endian host
every float and every `u32` label would silently decode as garbage.

This is a deliberate non-goal, not an oversight. Every platform `rux gui` and
its browser client run on is little-endian (x86-64, aarch64 in LE mode, wasm is
LE by definition). If a big-endian client ever matters, the honest fix is a
`flags` bit advertising byte order plus a byte-swapping slow path — not a
silent reinterpretation.

## Paging

`offset` and `limit` mean exactly what they mean on the JSON path, and the two
formats are interchangeable page for page:

- `offset` is clamped to `total`; `limit` is clamped to the server maximum.
- A page past the end is a **200** with `count = 0` and no payload bytes, not a
  404. `total` is still populated, so a client can discover the length of a
  cloud with `limit=1&offset=0`.
- Points come back in storage order, which is index-aligned across the sibling
  clouds of one scan, so the same window can be pulled from a geometry cloud and
  a `Label` cloud and zipped positionally (`docs/CONTRACTS.md`).

## Server-side memory

Serving a page reads **only the bytes that page needs**.

`point_cloud_data` stores each cloud as a fixed-stride record array split across
one or more ≤256 MB blobs, and `point_clouds.point_step` / `point_count` are in
the metadata row. A page therefore maps to an exact byte range
`[offset * step, (offset + count) * step)`. The server:

1. reads `chunk_index, length(data)` for the cloud — `length()` on a blob column
   does not load the blob — and prefix-sums it into a chunk map;
2. opens each overlapping chunk with SQLite incremental blob I/O
   (`sqlite3_blob_open`) and `sqlite3_blob_read`s just the overlapping
   sub-range, handling a record that straddles a chunk boundary;
3. transposes the resulting record array into planar sections.

Peak resident memory is `O(page)`, not `O(cloud)`, with no schema change and no
cache: a 100k-point page of a 10M-point `PointXYZRGB` cloud touches ~1.6 MB of
blob instead of the ~160 MB the blob occupies (16 bytes per stored record), let
alone the ~320 MB it would occupy once inflated into `pcl::PointXYZRGB`, which
is 32 bytes wide in memory because of SSE padding.

The JSON path uses the same partial read, so it gets the same bound.

### Measured

One 100 000-point page (`offset=0&limit=100000`) of a 4 000 000-point
`PointXYZRGB` cloud, served by `rux gui`. "Peak RSS" is the rise in the server
process's `VmHWM` from idle to after the request — i.e. what serving that one
page cost.

| Path | Page bytes | Bytes/point | Peak RSS |
|---|---:|---:|---:|
| JSON, `dump(2)` — before | 12 157 640 | 121.6 | +181.0 MB |
| JSON, `dump(-1)` — after | 6 957 575 | 69.6 | +30.3 MB |
| **RUXP binary — after** | **1 500 072** | **15.0** | **+3.2 MB** |

**8.1× less on the wire and 56× less memory** than before. The 181 MB is the
old whole-cloud materialisation: 64 MB of stored blob plus the same points
inflated into `pcl::PointXYZRGB`, which is 32 bytes wide because of SSE padding
— for a page that is 2.5 % of the cloud. The 3.2 MB is the page and nothing
else, and it does not grow with the cloud.

`1 500 072 = 72 + 100 000 × 15` — the header's promise, exactly.

## Client

`apps/rux/frontend/src/viewport/binaryPoints.ts` parses a page; the viewport
loader asks for `format=binary` first and falls back to `format=json` when the
server answers `501` (a `rux gui` that predates this format) or when the body
does not start with the `RUXP` magic. The fallback is per-cloud-stream and
sticky, so a stream against an old server pays one failed request, not one per
page.

**`501` is reserved for "this server does not implement this format", and for
nothing else.** Because the client's fallback is sticky, a `501` demotes the
whole stream to JSON — so using it to report a per-cloud condition would
silently cost every *other* cloud in that session its binary transport. A cloud
whose type has no RUXP layout is a `500`, not a `501`; a malformed `format`
value is a `400`.

## Not in v1

Deliberately out of scope, tracked in
[#320](https://github.com/pfmephisto/ReUseX/issues/320):

- **Voxel LOD** (`lod` / `max_points`) — "all of it, coarsely" instead of a
  prefix of it. This is a different question from transport and wants its own
  design.
- **Quantised positions** (e.g. `u16` per axis plus a per-page bbox) — roughly
  halves the position payload, but needs a decode step on the client, which
  works against the reason this format is planar. Worth measuring once LOD
  exists.
- **Draco / content negotiation.**

A v2 that adds any of these must bump `version`; a client that sees a version it
does not know must refuse the page rather than guess.
