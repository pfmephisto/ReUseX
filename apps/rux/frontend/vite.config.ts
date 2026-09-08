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
    // A scan viewport pulls in three.js; the default 500 kB warning is noise.
    chunkSizeWarningLimit: 1500,
    sourcemap: true,
  },
  test: {
    // The tested modules are pure logic — the API client with an injected
    // fetch, the event reducer, the chunk state machine. No DOM needed, so no
    // jsdom dependency is carried.
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
