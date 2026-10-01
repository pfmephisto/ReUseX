// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What a failed load means, in Danish — the copy behind `ErrorBanner`.
 *
 * The three statuses the contract singles out want different reactions, so
 * they are not collapsed into one "request failed": 503 is transient (a
 * running step held the project) and wants the button, 501 is a missing
 * feature and 404 says the data is not in this project — retrying helps with
 * neither. `subject` is a definite noun phrase so both sentences read:
 * "Kunne ikke hente projektoversigten".
 *
 * A network failure (fetch rejects with a `TypeError` — "Failed to fetch" in
 * Chrome, "NetworkError when attempting to fetch resource." in Firefox) is
 * never shown verbatim: that text comes straight from the browser and is
 * always English, so it is replaced with a Danish sentence instead.
 */

import { ApiRequestError } from '../api/client';

export interface LoadErrorCopy {
  heading: string;
  message: string;
  /** Whether retrying is the thing to do about it. */
  retryIsTheAnswer: boolean;
}

export const RETRY_LABEL = 'Prøv igen';

export function explainLoadError(error: Error, subject = 'dataene'): LoadErrorCopy {
  const heading = `Kunne ikke hente ${subject}`;
  if (error instanceof ApiRequestError) {
    if (error.isRetryable) {
      return {
        heading,
        message: `Projektdatabasen var optaget — et kørende trin skrev til projektet, da ${subject} blev hentet. Intet er galt; prøv igen om et øjeblik.`,
        retryIsTheAnswer: true,
      };
    }
    if (error.isNotImplemented) {
      return { heading, message: `Denne serverversion understøtter ikke ${subject} endnu.`, retryIsTheAnswer: false };
    }
    if (error.isNotFound) {
      return { heading, message: `Findes ikke i projektet: ${subject} er ikke i den åbne .rux-fil.`, retryIsTheAnswer: false };
    }
  }
  // A network failure: fetch itself threw, before any response existed. The
  // browser's own text (always English) is replaced, never shown.
  if (error instanceof TypeError) {
    return { heading, message: 'Kunne ikke forbinde til serveren.', retryIsTheAnswer: true };
  }
  // Any other failure falls through with its own message. Never the stack: it
  // tells the user nothing.
  return { heading, message: error.message, retryIsTheAnswer: true };
}
