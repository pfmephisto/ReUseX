// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';
import { MemoryRouter } from 'react-router-dom';

/**
 * Context wrapper for design-system preview cards (design-sync harness only —
 * not part of the app). Router-dependent components (NavRail's NavLinks)
 * render inside a MemoryRouter; everything else passes through unaffected.
 */
export function PreviewProviders({ children }: { children: ReactNode }) {
  return <MemoryRouter>{children}</MemoryRouter>;
}
