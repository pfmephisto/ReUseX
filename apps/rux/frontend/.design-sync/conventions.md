# ReUseX GUI — conventions for building with this design system

This library is the shipped frontend of `rux gui` — a local web app for a 3D
scan-processing pipeline (point clouds → planes → rooms → mesh). Its register
is a precision instrument on a permanently dark canvas: dense data, calm
status colors, monospace figures.

## Setup / wrapping

- Components are self-contained except two: **NavRail** needs a react-router
  context (wrap your app in a router; the preview harness uses `MemoryRouter`
  via the exported `PreviewProviders`), and **JobToaster** renders only inside
  the app's live `JobsProvider` (it subscribes to a running `rux gui` server —
  in a static design, show job UI with `StageCard`/`JobIndicator`/
  `StageProgress` fed by props instead).
- The page background must be `var(--color-canvas)` with `color:
  var(--color-text)` — every component assumes the dark canvas; there is no
  light theme.

## Styling idiom: CSS custom properties only

Component internals are CSS modules (their class names are hashed — never
target them). Style YOUR layout glue exclusively through the tokens defined in
`tokens/tokens.css`:

- **Surfaces**: `--color-canvas` (page), `--color-surface` (cards),
  `--color-surface-raised` / `-sunken` / `-overlay`; borders `--color-border`,
  `--color-border-strong`, focus `--color-border-focus`.
- **Text**: `--color-text`, `--color-text-muted`, `--color-text-faint`,
  `--color-text-inverse`; accent `--color-accent` / `-hover` / `-muted`,
  `--color-on-accent`.
- **Job/stage status** (load-bearing across StageCard, JobIndicator, history
  rows): `--color-status-queued|running|succeeded|failed|cancelled`.
- **Segmentation classes**: `--label-0` … `--label-7` (+ `--label-unlabeled`,
  cycle count `--label-count`) — colorblind-safe categorical scale; use for
  anything class-colored (legends, swatches, viewport overlays).
- **Type**: `--font-sans` for UI, `--font-mono` for values/ids/commands (the
  global `mono` utility class applies it); sizes `--font-size-xs…2xl`, weights
  `--font-weight-regular|medium|bold`, line heights `--line-height-tight|normal`.
- **Space & shape**: `--space-0…7` (4px scale) for all gaps/padding;
  `--radius-sm|md|lg|pill`; `--shadow-sm|md|lg`.
- **Layout constants**: `--layout-nav-width`, `--layout-panel-width` (also the
  minmax floor for card grids), `--layout-titlebar-height`; z-order
  `--z-titlebar|panel|toast`; motion `--duration-fast|normal`,
  `--easing-standard`.

Never hard-code a color, size, or radius — if a value isn't a token, it
doesn't belong in this UI.

## Where the truth lives

Read `styles.css` and its imports (`tokens/tokens.css` = every token with
comments, `tokens/base.css` = global element styles + the `mono` utility)
before styling anything. Each component's API and usage examples are in its
`.d.ts` and `.prompt.md`.

## Idiomatic composition

```tsx
import { StatCard, DataTable, EmptyState, StageCard } from 'reusex-gui';

<main style={{ background: 'var(--color-canvas)', color: 'var(--color-text)',
               padding: 'var(--space-5)', display: 'grid',
               gap: 'var(--space-4)',
               gridTemplateColumns: 'repeat(auto-fill, minmax(var(--layout-panel-width), 1fr))' }}>
  <StatCard label="Point clouds" value={6} hint="42,004 points total" />
  <StatCard label="Meshes" value={0} tone="muted" />
</main>
```

Content conventions: real pipeline vocabulary (`rux create clouds`,
`cloud_reconstruction`, `segment_planes`), pre-formatted numbers
(`toLocaleString`), job ids as short hex in mono. Empty states name the stage
that fills them; errors stay sticky and specific.
