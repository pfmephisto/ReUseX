# design-sync notes — reusex-gui

- This is a private Vite APP, not a packaged library: no dist entry, converter
  runs in synth-entry mode from `src/components/` (cfg.srcDir).
- `node_modules/reusex-gui -> ../..` self-symlink is REQUIRED for the converter
  to resolve the package (npm won't self-install). `npm ci` wipes it — recreate
  after every install: `ln -sfn .. node_modules/reusex-gui`.
- Config home is apps/rux/frontend/ (not the repo root).
- Tokens: src/tokens.css (design-owned custom properties) + src/base.css; all
  component styling is CSS modules importing those vars.
- `--font-mono` is a SYSTEM font stack by design (ui-monospace, SF Mono, ...);
  JetBrains Mono / Fira Code are opportunistic mid-stack candidates, not shipped
  brand fonts → runtimeFontPrefixes suppresses [FONT_MISSING] honestly.
- JobToaster/NavRail take no props (context-driven): dtsPropsFor pins empty
  props bodies. NavRail previews via PreviewProviders (MemoryRouter, in
  src/preview-support.tsx via extraEntries). JobToaster keeps the FLOOR CARD
  deliberately: its JobsContext is not externally injectable and feeding it
  requires a live rux gui server — do not chase this on re-sync.
- .d.ts contracts come from `npm run build:types` (tsc -p tsconfig.decl.json ->
  dist-types/, wired as package.json "types" + src/index.ts barrel). Re-run it
  (cfg.buildCmd) whenever component props change.

## Preview-authoring learnings (2026-09-09, folded from waves A+B)
- All 15 authored components fed via plain props; JobToaster is the only
  floor card (JobsContext not injectable).
- JobIndicator CSS cascade bug found during authoring (busy overrode the
  connection color) — filed as #324 and FIXED: `.busy` no longer touches
  `background`, it composes as a pulsing ring (box-shadow, --color-status-running)
  around a dot that keeps its connection colour. JobIndicator/TitleBar have
  been re-captured and re-graded against that rendering; DisconnectedWhileBusy
  (red-orange core + ring) and OneJobRunning (green core + ring) are now
  distinct cells. Previews were always honest — only the component CSS moved.
- ApiRequestError is not exported from the package surface, so ErrorBanner's
  tailored 503/501/404 branches are unreachable from previews; cells cover the
  public fallback path. Export it if those branches should be showcased.
- EmptyState WithAction uses a native <button> (no Button export exists) —
  browser-default chrome inside the card is expected, not a bug.
- NavRail: no props + bare MemoryRouter = always renders the "Overview" route
  active; variance ceiling is structural.
- PipelineLogEntry stage names are engine tokens (cloud_reconstruction,
  segment_planes...) — previews must use those, not display names.

## Known render warns
- (none currently — 16/16 render clean as of the final wave build)

## Re-sync risks
- dist-types/ must be regenerated via cfg.buildCmd (npm run build:types)
  whenever component props change, or .d.ts contracts go stale.
- node_modules/reusex-gui self-symlink vanishes on npm ci — recreate
  (see top of this file).
- Preview data inlines realistic wire objects (Job/StageInfo shapes from
  dist-types/api/types.d.ts); if the wire types change, previews compile-fail
  at rebuild — fix the compositions, don't loosen the types.
- JobIndicator's busy signal is a box-shadow ring, not a background swap
  (#324). If a future sync re-introduces a `background` in `.busy`, the
  connection colour is lost again — src/test/jobIndicatorStyles.test.ts
  guards this, so vitest fails before the sheets do.
