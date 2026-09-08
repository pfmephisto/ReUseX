// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Navigate, Route, Routes } from 'react-router-dom';

import { JobsProvider } from './JobsContext';
import { AppShell } from './AppShell';
import { Dashboard } from '../routes/Dashboard';
import { ViewportPage } from '../routes/ViewportPage';

/**
 * Route table.
 *
 * Paths stay extensionless on purpose: `rux gui`'s SPA fallback only rewrites
 * extensionless requests to `index.html`, so that a missing `/assets/app.js`
 * still 404s instead of silently returning HTML. A route containing a dot would
 * break on reload.
 */
export function App() {
  return (
    <JobsProvider>
      <AppShell>
        <Routes>
          <Route path="/" element={<Dashboard />} />
          <Route path="/viewport" element={<ViewportPage />} />
          <Route path="*" element={<Navigate to="/" replace />} />
        </Routes>
      </AppShell>
    </JobsProvider>
  );
}
