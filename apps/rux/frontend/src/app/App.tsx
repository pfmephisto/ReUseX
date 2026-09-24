// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Navigate, Route, Routes, useLocation } from 'react-router-dom';

import { JobsProvider } from './JobsContext';
import { LabelQueueProvider } from './LabelQueueContext';
import { AppShell } from './AppShell';
import { Dashboard } from '../routes/Dashboard';
import { LabelsPage } from '../routes/DataPage';
import { ExportPage } from '../routes/ExportPage';
import { FramesPage } from '../routes/FramesPage';
import { GeometryPage } from '../routes/GeometryPage';
import { GraphViewPage } from '../routes/GraphViewPage';
import { InstancesPage } from '../routes/InstancesPage';
import { MaterialsPage } from '../routes/MaterialsPage';
import { PipelineLogPage } from '../routes/PipelineLogPage';
import { PipelinePage } from '../routes/PipelinePage';
import { ViewportPage } from '../routes/ViewportPage';

/**
 * Route table.
 *
 * Paths stay extensionless on purpose: `rux gui`'s SPA fallback only rewrites
 * extensionless requests to `index.html`, so that a missing `/assets/app.js`
 * still 404s instead of silently returning HTML. A route containing a dot would
 * break on reload.
 *
 * ## Viewport keep-alive
 *
 * `ViewportPage` is rendered *outside* the `<Routes>` switcher and kept mounted
 * permanently. Navigating to `/geometry`, `/materials`, etc. CSS-hides it via
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
          <Route path="/" element={<Dashboard />} />
          <Route path="/pipeline" element={<PipelinePage />} />
          <Route path="/pipeline/log" element={<PipelineLogPage />} />
          <Route path="/frames" element={<FramesPage />} />
          <Route path="/graph-view" element={<GraphViewPage />} />
          <Route path="/geometry" element={<GeometryPage />} />
          <Route path="/instances" element={<InstancesPage />} />
          <Route path="/materials" element={<MaterialsPage />} />
          <Route path="/labels" element={<LabelsPage />} />
          <Route path="/export" element={<ExportPage />} />
          <Route path="*" element={<Navigate to="/" replace />} />
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
