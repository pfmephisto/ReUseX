// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `vitest/config` rather than `vite`: it is the same `defineConfig` widened to
// accept the `test` block below. Importing it from `vite` type-errors.
import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';

/**
 * Vite configuration for the ReUseX GUI frontend.
 *
 * The dev proxy is not a convenience — it is required. `rux gui` cannot answer
 * a CORS preflight (Crow 1.3 replies to `OPTIONS` before the request headers
 * are parsed, so it never sees the `Origin`), which means any JSON-bodied
 * cross-origin call from a bare `vite dev` would fail. Proxying `/api` makes
 * the browser talk only to the Vite origin, and CORS never enters into it.
 * See `docs/gui/README.md` § "The frontend must be same-origin".
 *
 * `RUX_GUI_URL` retargets the proxy at a server on another port.
 */
const backend = process.env.RUX_GUI_URL ?? 'http://localhost:8420';

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      // `ws: true` covers /api/v1/events as well as the REST routes.
      '/api': { target: backend, ws: true, changeOrigin: false },
    },
  },
  build: {
    // `rux gui` serves this directory verbatim; --assets points at it.
    outDir: 'dist',
    sourcemap: true,
    rolldownOptions: {
      output: {
        // Split the big vendor libraries out of the app chunk, so no chunk
        // crosses Vite's 500 kB warning limit and an app-only change leaves
        // the vendor chunks' hashes (and the browser's cached copies) intact.
        // The ViewportPage is mounted on every route (see App.tsx), so three.js
        // is needed at start-up either way; this splits it, it does not defer it.
        // three ships as two modules (three.core.js + the WebGL renderer in
        // three.module.js) that together exceed 500 kB; keep them apart.
        // Groups are tried in order, so the core group must come first.
        codeSplitting: {
          groups: [
            {
              name: 'vendor-three-core',
              test: /[\\/]node_modules[\\/]three[\\/]build[\\/]three\.core\.js$/,
            },
            { name: 'vendor-three', test: /[\\/]node_modules[\\/]three[\\/]/ },
            {
              name: 'vendor-react',
              test: /[\\/]node_modules[\\/](react|react-dom|react-router|react-router-dom|scheduler)[\\/]/,
            },
          ],
        },
      },
    },
  },
  test: {
    // The tested modules are pure logic — the API client with an injected
    // fetch, the event reducer, the chunk state machine. No DOM needed, so no
    // jsdom dependency is carried.
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
