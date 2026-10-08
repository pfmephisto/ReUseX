title: `--label-invalid` needs a design-project value, not a hand-added one
labels: gui, design, frontend

## Problem
The Qt client's 3D view and legend render points whose label is out of
contract — a `-1` that wrapped to `0xFFFFFFFF` in a point cloud (STANDARDS
§3.1) — in a dedicated colour, `--label-invalid`, so they are visually
distinct from both a valid categorical label and `--label-unlabeled` (label
`0`). That token does not belong to the Okabe-Ito categorical scale (it is
not a class), so it was never ruled on by the design project.

It was added directly to `src/tokens.css` during the Qt client's Q3 fix
round, under controller instruction, against the file's own rule ("Never
hand-edit `src/tokens.css`" — SKILL.md, `/design-sync` owns every value).
That is a stopgap, not a resolution:

- The next `/design-sync` overwrites `tokens.css` wholesale and can silently
  drop the token, since the design project does not know it exists.
- The provisional value was picked by deriving it from a neighbouring
  neutral token per theme (`--color-border-strong` in light,
  `--color-border` in dark) so it recedes against the near-black 3D
  canvas rather than competing with the vivid label palette. That is a
  reasonable placeholder, not a design decision — colour, light/dark
  parity, and contrast against the legend panel are all open.

## Proposed fix
- [ ] Design project: rule on `--label-invalid`'s value in both themes — a
      recessive neutral, distinct from `--label-unlabeled`
      (`#4a505c`, theme-invariant), that still reads as a legend swatch on a
      light or dark panel without becoming the loudest thing in the 3D view.
- [ ] Add the ruled values to `src/tokens.css` via the normal `/design-sync`
      flow and drop the "PROVISIONAL" comments once it lands.
- [ ] Confirm `reusex-frontend.md`'s token inventory and `qt-client.md`'s
      gallery shots (`3d-labels`, both themes) still match after the sync.

category=Visualization estimate=2h
