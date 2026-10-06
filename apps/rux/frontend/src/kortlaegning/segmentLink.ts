// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * "Segmentér" from Kortlægning (button + S): open the Segmentering view on a
 * part's best source frame with its instance centroid seeded as a point
 * prompt. The frames come from the same `instanceFrames` lookup the Foto
 * evidence uses; `frames[0]` is the most central one and its u,v is where the
 * centroid projects.
 */

import type { SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import { segmentHref } from '../app/links';
import { hasInstanceLink } from './photo';

/** The part to segment: the selected one when scan-backed, else a type row's first scan-backed part. */
export function segmentTargetPart(
  type: SurveyType | null,
  part: SurveyPart | null,
): (SurveyPart & { cloud: string; instance_id: number }) | null {
  if (part) return hasInstanceLink(part) ? part : null;
  const first = type?.parts.find((p) => hasInstanceLink(p));
  return first && hasInstanceLink(first) ? first : null;
}

export function segmentHrefFromFrames(frames: readonly VisibleFrame[]): string | null {
  const best = frames[0];
  return best ? segmentHref(best.frame_id, { u: best.u, v: best.v }) : null;
}
