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
| 8 | 4 | `u32` | `flags` — bit 0 = LOD (see below); a client must reject a page with any *other* bit set |
| 12 | 4 | `u32` | `field_count` |
| 16 | 4 | `u32` | `count` — points **in this page** |
| 20 | 4 | `u32` | reserved, 0 |
| 24 | 8 | `u64` | `offset` — index of the first point of this page within the cloud |
| 32 | 8 | `u64` | `total` — points in the whole cloud |
| 40 | `16 * field_count` | field[] | field descriptors, see below |

`header_size` is a `u16`, so `field_count` can never exceed 4093. No cloud type
comes close (the largest table today is two fields); an encoder that would
overflow it must fail loudly rather than truncate.

### Flags

| bit | name | meaning |
|---|---|---|
| 0 | `LOD` | This page is a level-of-detail view of the **whole** cloud, not a contiguous window of it. See [Level of detail](#level-of-detail). |

Every other bit is reserved and **must** be rejected by a reader.

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

## Level of detail

`max_points=N` answers a different question from `offset`/`limit`: **all of the
cloud, coarsely**, instead of a prefix of it. It returns up to `N` points drawn
by a voxel grid from the whole cloud, in one page, with the `LOD` flag set
(`format=json` says the same thing with `"lod": true` and a `"voxel_size"`).

Same cloud, same point count, same bytes — a 177 656-point prefix of a
1 212 572-point office scan on the left, `max_points=200000` on the right:

![A prefix shows part of the floor; the voxel LOD shows all of
it](images/lod-prefix-vs-voxel.png)

The left-hand page is not a coarse version of the building. It is a different,
smaller building.

### Rules

- **`max_points` is mutually exclusive with `offset` and `limit`.** Sending it
  with either is a `400`. The two answer different questions, and any rule that
  let them coexist would have to invent a meaning for an offset into a set the
  client cannot enumerate.
- `N` is clamped to the server maximum (1 000 000), like `limit`. `N < 1` is a
  `400`.
- `total` keeps its usual meaning: points in the **whole** cloud. `count` is
  what came back, `offset` is 0 and carries no meaning.
- When the cloud already fits `N` it is returned complete, in storage order,
  with the flag **clear**. That page is an ordinary page in every respect —
  claiming LOD would tell the client to distrust an offset that is fine.
- `count <= N` is guaranteed. `count` is typically 80–98 % of `N`; see
  [Why the count is not exactly `max_points`](#why-the-count-is-not-exactly-max_points).
- The selection is deterministic: the representative of a voxel is its
  lowest-indexed point, and the page is sorted by that index. The same request
  against the same cloud returns the same bytes.

### Sibling clouds

A LOD page **must not** be zipped positionally against a page that came from a
different selection. A `Label` cloud carries no positions of its own, so
`max_points` alone on one is a `400`; pass `lod_source=<geometry cloud>` and the
server voxelises *that* cloud and returns the sibling's records at the same
storage indices. The two pages then describe the same points and the usual
index-aligned zip (`docs/CONTRACTS.md`, STANDARDS §3.2) still holds.

The two clouds must have the same `total`, or the request is a `400` — that is
what "index-aligned" means, and a short page instead would silently paint the
wrong labels onto the wrong points.

### Why this is computed per request

Three places could hold the answer, and only one of them is available without a
storage change.

**Precomputed progressive order (Morton / octree), rejected for now — and the
right end state.** Storing each cloud in a space-filling-curve order makes every
LOD a *prefix*: `max_points` collapses into `limit`, paging keeps working
unchanged, and the server does no work at all. It is plainly better. It is also
not a serving change: reordering a geometry cloud invalidates every sibling
cloud of that scan — `normals`, `planes`, `rooms`, `instances`, `labels` are
index-aligned with it by contract — so they must all be permuted atomically, at
`create clouds` time, behind a schema migration, with a rule for clouds written
by later stages. That is a pipeline and storage project, not an endpoint
parameter, and it is filed as a follow-up on
[#320](https://github.com/pfmephisto/ReUseX/issues/320).

**A server-side cache, rejected.** `rux gui` opens a fresh `ProjectDB` per
request and holds no cross-request state; a cache means inventing an
invalidation story against a pipeline that rewrites clouds underneath it. The
work being cached is one sequential scan of a blob the pipeline already reads
routinely — not enough to pay for that.

**Per request, chosen.** No schema change, no migration, nothing to invalidate,
and it works on every `.rux` that exists today rather than on ones rebuilt after
this lands.

The wire contract is written so the first can supersede the third without
breaking a client: `max_points` describes *what the client wants*, never how it
is obtained, and a progressive-order server would answer the identical request
by serving a prefix. It could then even clear the `LOD` flag, since the page
really would be a storage-ordered prefix — a strictly weaker claim, which no
client can be broken by.

Adding the flag needed **no version bump**: the byte layout is unchanged, and a
reader that predates it can only ever meet it by sending `max_points`, which it
does not know about. Refusing unknown flags is what makes that safe.

### How the selection works

One streaming pass over the stored records, at `kLodStreamChunk` (262 144)
points at a time. Each point is hashed into a voxel; the first point to claim a
voxel keeps it. When the kept set would exceed the budget the grid is
**coarsened**: the edge doubles, and because

```
floor( floor(c / s) / 2^n )  ==  floor( c / (s * 2^n) )
```

holds exactly for any real `c` and positive `s`, every key maps onto its parent
by an arithmetic shift. The already-kept set is merged in place rather than
re-read, so the scan never rewinds. Peak memory is `O(max_points)` — the kept
records and their keys, ~40 bytes per kept point — not `O(cloud)`.

Coarsening never takes a bigger step than the overshoot calls for. Jumping
ahead converges in fewer merge passes and was tried; it overshoots the coarsest
level that still fits the budget, and one level is a ~4× step in count.

#### Why the count is not exactly `max_points`

A dyadic grid can only offer counts about 4× apart, and where those land
relative to the budget is luck. Measured on the same scan, one pass answered a
100 000-point budget with 97 807 points and a 200 000-point budget with 45 720 —
23 % of what was asked for, which is a visibly different picture of the room.

So the pass runs at most **twice**. The first measures; having kept `n` points
at edge `e`, the same surface model that drives coarsening (`n ∝ e^-2`) inverts
to give the edge that would have filled the budget. The second pass starts
*there*, which moves the whole lattice instead of stepping along the old one,
and aims at 80 % of the budget so an approximate model does not land over it and
coarsen straight back. The better of the two answers wins. It is skipped
entirely when the first pass already used three quarters of the budget.

Measured on the 1 212 572-point scan above (`PointXYZRGB`, `rux gui` on
loopback):

| `max_points` | points returned | of budget | page bytes | time |
|---:|---:|---:|---:|---:|
| 20 000 | 16 583 | 83 % | 248 817 | 0.08 s |
| 50 000 | 45 720 | 91 % | 685 872 | 0.10 s |
| 100 000 | 97 807 | 98 % | 1 467 177 | 0.15 s |
| 200 000 | 177 656 | 89 % | 2 664 912 | 0.32 s |
| 500 000 | 418 685 | 84 % | 6 280 347 | 0.81 s |

Against 18 189 516 bytes over 13 requests for the whole cloud. The 200 000-point
overview is **6.8× fewer bytes** and, more to the point, the whole scan is on
screen after the first response instead of the thirteenth.

Note what this does *not* buy on loopback: the whole-cloud download also
finishes in ~0.3 s here, because a LOD costs a sequential scan the server would
not otherwise do. The win is bytes on the wire, GPU memory on the client, and
what is visible while the rest arrives — not server time. A progressive ordering
is what would make it free.

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

`apps/rux/frontend/src/viewport/binaryPoints.ts` parses a page. The viewport
loader's **first** request is not the first page — it is `max_points`, so the
whole scan is on screen, correctly framed, before any of the paging requests
behind it return. The overview goes into a scene layer of its own and is dropped
once the full-resolution pages have covered the same ground; `useCloudStream`
handles the three answers a server can give (a LOD page, the whole cloud when it
fit the budget, or a plain prefix from a server that predates `max_points`).

The loader asks for `format=binary` first and falls back to `format=json` when the
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

- **Precomputed progressive ordering** (Morton / octree at `create clouds`
  time) — would make every LOD level a prefix and cost the server nothing. See
  [Why this is computed per request](#why-this-is-computed-per-request) for what
  blocks it and why the wire contract already allows it to land later.
- **Quantised positions** (e.g. `u16` per axis plus a per-page bbox) — roughly
  halves the position payload, but needs a decode step on the client, which
  works against the reason this format is planar. Now worth measuring: LOD
  exists, and a `PointXYZRGB` page is 80 % positions.
- **Draco / content negotiation.**

Voxel LOD itself has landed — see [Level of detail](#level-of-detail). It needed
no version bump, because it added a `flags` bit rather than changing the layout.

A v2 that changes a section layout must bump `version`; a client that sees a
version it does not know must refuse the page rather than guess.
