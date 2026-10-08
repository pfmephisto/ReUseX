// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Navigate, Route, Routes, useLocation } from 'react-router-dom';

import { JobsProvider } from './JobsContext';
import { LabelQueueProvider } from './LabelQueueContext';
import { AppShell } from './AppShell';
import {
  INDBERETNING_PATH,
  INDSTILLINGER_PATH,
  KORTLAEGNING_PATH,
  MILJOE_PATH,
  OVERBLIK_PATH,
  RAPPORT_PATH,
  SEGMENTERING_PATH,
  SKABELONER_PATH,
} from './links';
import { REDIRECTS } from './navigation';
import { LabelsPage } from '../routes/DataPage';
import { FramesPage } from '../routes/FramesPage';
import { GeometryPage } from '../routes/GeometryPage';
import { GraphViewPage } from '../routes/GraphViewPage';
import { IndberetningPage } from '../routes/IndberetningPage';
import { IndstillingerPage } from '../routes/IndstillingerPage';
import { InstancesPage } from '../routes/InstancesPage';
import { KortlaegningPage } from '../routes/KortlaegningPage';
import { MiljoePage } from '../routes/MiljoePage';
import { OverblikPage } from '../routes/OverblikPage';
import { PipelineLogPage } from '../routes/PipelineLogPage';
import { PipelinePage } from '../routes/PipelinePage';
import { RapportPage } from '../routes/RapportPage';
import { SegmenteringPage } from '../routes/SegmenteringPage';
import { SkabelonerPage } from '../routes/SkabelonerPage';
import { ViewportPage } from '../routes/ViewportPage';

/**
 * Route table of one case.
 *
 * It runs under a router whose basename is the case prefix `/sager/:cid`
 * (`main.tsx`, `app/cases.ts`), so every path here is case-relative and
 * `/kortlaegning` means `/sager/<cid>/kortlaegning`. The case list (`/sager`)
 * is a separate app, `CasesApp`.
 *
 * Paths stay extensionless on purpose: `ruxd --local`'s SPA fallback only rewrites
 * extensionless requests to `index.html`, so that a missing `/assets/app.js`
 * still 404s instead of silently returning HTML. A route containing a dot would
 * break on reload.
 *
 * ## Viewport keep-alive
 *
 * `ViewportPage` is rendered *outside* the `<Routes>` switcher and kept mounted
 * permanently. Navigating to `/geometry`, `/kortlaegning`, etc. CSS-hides it via
 * `display:none` but does not unmount it, so the Three.js scene, point-cloud
 * pages, mesh blobs, and splat blobs all stay in GPU/CPU memory across
 * navigation. Without this, every visit to `/viewport` re-downloads all
 * geometry.
 *
 * `display:none` sets the canvas dimensions to 0×0. The Viewport already
 * contains a ResizeObserver on its container, so when the wrapper becomes
 * visible again the renderer is resized back to the correct dimensions
 * automatically on the next animation frame.
 */
function RoutedContent() {
  const location = useLocation();
  const onViewport = location.pathname === '/viewport';

  return (
    <>
      {/* Always mounted; hidden (not unmounted) when off /viewport */}
      <div
        style={{
          display: onViewport ? 'contents' : 'none',
          height: '100%',
        }}
      >
        <ViewportPage />
      </div>

      {/* Standard switcher for every other page */}
      {!onViewport && (
        <Routes>
          <Route path={OVERBLIK_PATH} element={<OverblikPage />} />
          <Route path={KORTLAEGNING_PATH} element={<KortlaegningPage />} />
          <Route path={SEGMENTERING_PATH} element={<SegmenteringPage />} />
          <Route path={MILJOE_PATH} element={<MiljoePage />} />
          <Route path={RAPPORT_PATH} element={<RapportPage />} />
          <Route path={SKABELONER_PATH} element={<SkabelonerPage />} />
          <Route path={INDSTILLINGER_PATH} element={<IndstillingerPage />} />
          <Route path={INDBERETNING_PATH} element={<IndberetningPage />} />
          <Route path="/pipeline" element={<PipelinePage />} />
          <Route path="/pipeline/log" element={<PipelineLogPage />} />
          <Route path="/frames" element={<FramesPage />} />
          <Route path="/graph-view" element={<GraphViewPage />} />
          <Route path="/geometry" element={<GeometryPage />} />
          <Route path="/instances" element={<InstancesPage />} />
          <Route path="/labels" element={<LabelsPage />} />
          {REDIRECTS.map((r) => (
            <Route key={r.from} path={r.from} element={<Navigate to={r.to} replace />} />
          ))}
          <Route path="*" element={<Navigate to={OVERBLIK_PATH} replace />} />
        </Routes>
      )}
    </>
  );
}

export function App() {
  return (
    <JobsProvider>
      <LabelQueueProvider>
        <AppShell>
          <RoutedContent />
        </AppShell>
      </LabelQueueProvider>
    </JobsProvider>
  );
}
