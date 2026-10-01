// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Lets a page that changes the survey ask the shell to re-read
 * `GET /survey/summary`, so the sidebar's review-queue badge follows the
 * page's approvals without the two sharing survey state.
 */

import { createContext, useContext } from 'react';

export interface SurveyCounts {
  /** Re-fetch the survey summary behind the sidebar badges. */
  refresh: () => void;
}

const SurveyCountsContext = createContext<SurveyCounts>({ refresh: () => {} });

export const SurveyCountsProvider = SurveyCountsContext.Provider;

export function useSurveyCounts(): SurveyCounts {
  return useContext(SurveyCountsContext);
}
