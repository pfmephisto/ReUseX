// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The point-click marker size for the Segmentering view's SAM3 prompts. The
 * prompt geometry and the request shape live in `data/segmentView.ts`
 * (`pointBox`, `buildRequestPrompts`).
 */

/**
 * Half-width, in image pixels, of the square marker drawn for a point click.
 * The click itself goes to the server as a point (`points`); the square is
 * only its marker.
 */
export const POINT_CLICK_RADIUS = 8;
