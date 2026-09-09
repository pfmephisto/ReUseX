// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Chunk planning for `GET /clouds/{name}/points`.
 *
 * Pulled out of the React hook and kept pure so the arithmetic — which is the
 * part that silently drops or double-loads points when it is wrong — can be
 * tested without a server, a canvas or a clock.
 */

/** `limit` maximum the contract declares. Asking for more is a 400. */
export const MAX_PAGE_SIZE = 1_000_000;

/**
 * Default page size.
 *
 * 100_000 points is small enough that the first page paints quickly and the
 * user sees the cloud building, large enough that a 20-million-point scan is
 * 200 requests rather than 20_000.
 *
 * The number was originally chosen against JSON, where a page is roughly 4 MB.
 * RUXP (#283) makes the same page 1.5 MB and removes the per-point parse, so
 * the request *cost* dropped a lot — but the reason for paging at all did not
 * change, because it was never only about bytes: 20 M points must not arrive as
 * one response the user waits out with nothing on screen. Raising this would
 * trade first-paint latency for fewer round trips, which is the wrong direction
 * for a viewport. The thing that actually removes the trade-off is LOD
 * (#320) — "all of it, coarsely" instead of a prefix of it — not a bigger page.
 */
export const DEFAULT_PAGE_SIZE = 100_000;

export interface PagePlan {
  /** 0-based page number, and the order pages must be applied in. */
  index: number;
  offset: number;
  limit: number;
}

/** Clamp a requested page size into what the contract will actually serve. */
export function clampPageSize(pageSize: number): number {
  if (!Number.isFinite(pageSize)) return DEFAULT_PAGE_SIZE;
  return Math.min(MAX_PAGE_SIZE, Math.max(1, Math.floor(pageSize)));
}

/**
 * Split a cloud of `total` points into pages.
 *
 * The last page is short rather than over-reading: the server clamps `limit`
 * against the cloud size anyway, but planning it honestly means `loadedPoints`
 * below never has to be reconciled against what came back.
 *
 * A `total` of 0 (or negative, which a broken server could report) yields no
 * pages at all — the caller must render an empty cloud, not issue a request for
 * points that do not exist.
 */
export function planPages(total: number, pageSize: number = DEFAULT_PAGE_SIZE): PagePlan[] {
  const limit = clampPageSize(pageSize);
  if (!Number.isFinite(total) || total <= 0) return [];

  const count = Math.ceil(total / limit);
  const plans: PagePlan[] = [];
  for (let index = 0; index < count; index += 1) {
    const offset = index * limit;
    plans.push({ index, offset, limit: Math.min(limit, total - offset) });
  }
  return plans;
}

/**
 * Fraction of a cloud loaded so far, in [0, 1].
 *
 * Returns `null` when the total is not yet known, so the caller renders an
 * indeterminate indicator instead of a bar sitting at zero — the same
 * distinction the job-progress contract makes for `total: 0`.
 */
export function loadFraction(loaded: number, total: number | undefined): number | null {
  if (total === undefined || !Number.isFinite(total) || total <= 0) return null;
  return Math.min(1, Math.max(0, loaded / total));
}

/**
 * Index of each named field in a page's `fields` array.
 *
 * The contract makes the field set depend on the cloud's *type*
 * (`PointXYZRGB` → `x,y,z,r,g,b`, `PointXYZ` → `x,y,z`, `Normal` → `nx,ny,nz`,
 * `Label` → `label`), and explicitly says rows match `fields` positionally. So
 * positions are resolved from the payload rather than assumed — a `PointXYZ`
 * cloud must not be read as if columns 3–5 held colour.
 */
export function fieldIndices(fields: string[]): Record<string, number> {
  const map: Record<string, number> = {};
  fields.forEach((name, index) => {
    map[name] = index;
  });
  return map;
}

/** True when a page carries positional geometry this viewport can render. */
export function hasPositions(fields: string[]): boolean {
  const index = fieldIndices(fields);
  return index.x !== undefined && index.y !== undefined && index.z !== undefined;
}

/** True when a page carries per-point colour. */
export function hasColors(fields: string[]): boolean {
  const index = fieldIndices(fields);
  return index.r !== undefined && index.g !== undefined && index.b !== undefined;
}
