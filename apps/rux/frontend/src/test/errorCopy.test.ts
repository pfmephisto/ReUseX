// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import { explainLoadError, RETRY_LABEL } from '../app/errorCopy';

describe('load-error copy', () => {
  it('says a 503 is a busy project and retry is the answer', () => {
    const c = explainLoadError(new ApiRequestError(503, 'busy', '/survey'), 'kortlægningen');
    expect(c.heading).toBe('Kunne ikke hente kortlægningen');
    expect(c.message).toBe(
      'Projektdatabasen var optaget — et kørende trin skrev til projektet, da kortlægningen blev hentet. Intet er galt; prøv igen om et øjeblik.',
    );
    expect(c.retryIsTheAnswer).toBe(true);
  });

  it('does not offer retry as the answer for 501 and 404', () => {
    expect(explainLoadError(new ApiRequestError(501, 'x', '/x'), 'rapportversionerne')).toEqual({
      heading: 'Kunne ikke hente rapportversionerne',
      message: 'Denne serverversion understøtter ikke rapportversionerne endnu.',
      retryIsTheAnswer: false,
    });
    expect(explainLoadError(new ApiRequestError(404, 'x', '/x'), 'punktskylisten').message).toBe(
      'Findes ikke i projektet: punktskylisten er ikke i den åbne .rux-fil.',
    );
  });

  it('shows a network failure in Danish, never the browser text, with a generic subject by default', () => {
    const c = explainLoadError(new TypeError('Failed to fetch'));
    expect(c).toEqual({
      heading: 'Kunne ikke hente dataene',
      message: 'Kunne ikke forbinde til serveren.',
      retryIsTheAnswer: true,
    });
  });

  it('labels the retry button in Danish', () => {
    expect(RETRY_LABEL).toBe('Prøv igen');
  });
});
