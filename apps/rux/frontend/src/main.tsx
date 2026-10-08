// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { BrowserRouter } from 'react-router-dom';

// Token import comes first so every later stylesheet can rely on the custom
// properties already being declared.
import './fonts';
import './tokens.css';
import './base.css';

import { api } from './api/client';
import { App } from './app/App';
import { CasesApp, LegacyRedirect } from './app/CasesApp';
import { caseBasename, parseCaseLocation } from './app/cases';

const container = document.getElementById('root');
if (!container) throw new Error('#root is missing from index.html');

// Which app this page load is: one case (`/sager/:cid/…`), the case list
// (`/sager`), or an old unprefixed path to forward. Entering or leaving a case
// is a page load, so the API client and the events socket are per case.
const where = parseCaseLocation(window.location.pathname);
let app;
if (where.kind === 'case') {
  // Remembered as the last case only once its health check succeeds
  // (AppShell, caseBootAction): an unknown id must not become the redirect
  // target of every old link.
  api.selectCase(where.cid);
  app = (
    <BrowserRouter basename={caseBasename(where.cid)}>
      <App />
    </BrowserRouter>
  );
} else if (where.kind === 'list') {
  app = (
    <BrowserRouter>
      <CasesApp />
    </BrowserRouter>
  );
} else {
  app = <LegacyRedirect pathname={window.location.pathname} search={window.location.search} />;
}

createRoot(container).render(<StrictMode>{app}</StrictMode>);
