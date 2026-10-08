// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { createContext, useContext, useEffect, useState, type ReactNode } from 'react';

import { authApi } from '../api/auth';
import type { CaseRole } from '../api/types';
import { canEdit } from './auth';

/** The signed-in user's role in the open case; undefined while unknown. */
const CaseRoleContext = createContext<CaseRole | null | undefined>(undefined);

/**
 * Reads the caller's role in case @p cid once (`GET /cases/{cid}`), so the
 * screens can hide what the role may not do (S3 review M9). Local mode's
 * implicit user is the owner.
 */
export function CaseRoleProvider({ cid, children }: { cid: string | undefined; children: ReactNode }) {
  const [role, setRole] = useState<CaseRole | null | undefined>(undefined);
  useEffect(() => {
    if (!cid) return;
    const controller = new AbortController();
    authApi.caseInfo(cid, controller.signal).then(
      (info) => {
        if (!controller.signal.aborted) setRole(info.role ?? 'owner');
      },
      () => undefined, // The shell reports a case it cannot read.
    );
    return () => controller.abort();
  }, [cid]);
  return <CaseRoleContext.Provider value={role}>{children}</CaseRoleContext.Provider>;
}

export function useCaseRole(): CaseRole | null | undefined {
  return useContext(CaseRoleContext);
}

/**
 * Whether edit controls are shown. True until the role is known (and
 * outside a case), so an editor never sees them flicker in; a viewer's
 * stray click is still refused by useMutationQueue and the server.
 */
export function useCanEdit(): boolean {
  return editVisible(useCaseRole());
}

/** The rule behind useCanEdit, for tests. */
export function editVisible(role: CaseRole | null | undefined): boolean {
  return role === undefined || canEdit(role);
}
