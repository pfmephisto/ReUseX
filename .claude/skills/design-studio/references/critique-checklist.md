# Critique checklist

Use on every screenshot round. Look at the image, not the code. Note the 3–5 worst issues, not every nit.

## Brief
- Does it do its one job? Is the primary action obvious within 3 seconds?
- Does it feel specific to this subject, or could the logo be swapped for any other company's?

## Hierarchy (squint test)
- Blur your eyes: what's 1st, 2nd, 3rd? Does that match importance?
- Exactly one dominant element per screen/section. Competing focal points are a defect.

## Layout and alignment
- Consistent grid and left edges; nothing a few pixels off.
- Related items closer together than unrelated ones (proximity).
- Section spacing follows one rhythm; no random gaps or cramped blocks.
- Nothing clipped, overlapping, or overflowing its container.

## Typography
- Clear type scale (roughly 1.2–1.33 ratio); no two sizes that look almost the same.
- Body line length 45–80 characters; line-height ~1.4–1.6 for body, tighter for display.
- At most two families; weights used intentionally.
- No widows/orphans in headlines; no awkward mid-phrase line breaks.

## Color
- Body text contrast ≥ 4.5:1; large text ≥ 3:1.
- Accent used sparingly for meaning (action, status), not sprinkled.
- Dark mode (if supported) is designed, not just inverted.

## Components and polish
- Border radius has a hierarchy (container > card > control), not one value everywhere.
- Shadows consistent in direction and softness; used for elevation, not decoration.
- Icons one set, one stroke weight, optically aligned with text.
- Buttons, inputs and chips share heights and padding.
- States exist where relevant: hover, focus, active, disabled, empty, loading, error.

## Content
- Real, specific copy; no lorem ipsum or "Feature 1".
- CTAs name the action; the same action keeps the same name throughout.
- Numbers, names and dates look realistic.

## Responsive (check the mobile shot closely)
- No horizontal scroll; nav collapses sensibly.
- Tap targets ≥ 44px; text ≥ 16px for body.
- Images and charts scale; tables scroll inside their own container.
- Order still makes sense when columns stack.

## Restraint
- One memorable move, everything else calm.
- Remove one decorative element before calling it done.

The web frontend's own critique checklist (a `shot.sh` round against
`apps/rux/frontend`) lives in the `rux-frontend` repo's `design-studio` skill,
not here — this repo covers the Qt client only. A few of its correctness
rules still bind code in this repo because the Qt client mirrors the same
design system: the viewport canvas stays near-black (`--color-canvas`) in
both themes, and `--label-0..7` stays the colourblind-safe Okabe-Ito set the
legend and 3D view agree on.

## Qt client (check on every `qt_shot.sh` round)
- **Both themes, at `--scale 2`.** Text and 1 px borders crisp; nothing
  pixel-doubled or blurry (a blurry image means a pixmap without
  `setDevicePixelRatio`).
- **No magenta anywhere.** Magenta is a missing token; `qt_shot.sh` should
  already have failed — if you see it, a token is read in code and the
  lookup went unnoticed.
- **The right faces.** Headings and KPI figures in Oswald, body in Archivo,
  figures in JetBrains Mono. A rounder/wider face means a font fell back —
  check stderr for `font family … not available`. Microscopic text means a
  `font-size` resolved to 0 (dump with `RUX_QT_DUMP_QSS`).
- **No stock-Qt leakage.** Scrollbars, combobox arrows, checkbox ticks,
  spin buttons, menus and tooltips are themed, not grey Fusion bevels.
- **Mnemonics.** No stray underscores where an `&` was meant (`&&`).
- **Caps and tracking** match the web: eyebrows and field labels in
  `CapsLabel` with `--tracking-*`, not `toUpper()` text with no spacing.
- **Canvas** is `--color-canvas` in both themes; a 3D pane under offscreen
  shows the EGL snapshot, not a blank rectangle.
- **Density matches the web**: table rows, header height, button heights and
  inputs share one rhythm; numbers right-aligned and mono.
- **Focus** is visible on buttons and fields (`:focus` border), and the nav
  rail's active row reads at a glance (accent bar + filled dot).
- **Empty states** exist: run the page with no `--project` too.
