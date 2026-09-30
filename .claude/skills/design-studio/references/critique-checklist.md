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

## ReUseX rux GUI (check on every rux screen)
- **Both themes reviewed.** You screenshotted `--theme dark` *and* `--theme light`.
  Nothing looks right in one and broken in the other (light is the default,
  `[data-theme='dark']` re-points the themed roles) — a component that flips
  wrong is hardcoding a colour.
- **Tokens only.** `scripts/token_lint.py` passes: no literal colour, radius,
  spacing or type size in the changed `.module.css`. `tokens.css` untouched.
- **Fits the app.** It looks like it belongs next to the existing routes/panels;
  you reused `DataTable`/`StatCard`/`Sidebar`/`EmptyState`/`ErrorBanner`/… rather
  than inventing a near-duplicate.
- **Viewport stays near-black** (`--color-canvas`) in both themes — point-cloud
  depth read depends on it. An untextured mesh uses `--mesh-surface`, not black.
- **Label colours intact.** `--label-0..7` remain the colourblind-safe Okabe-Ito
  set; the legend and viewport agree. Semantic classes must be distinguishable
  under deuteranopia/protanopia/tritanopia.
- **Dense-data legibility.** Figures use `.mono` / tabular-nums and align in
  columns; tables stay scannable at real row counts, not just 3 demo rows.
- **States present.** Empty (no project / no data), loading, error and running
  states are designed — this is a long-running pipeline tool, not a static page.
- **Real domain copy.** "Back-project depth frames", "Loop closures",
  "Unlabeled points" — not "Feature one". Actions name what they do.
- **Contract-honest.** Data shown matches `docs/gui/openapi.yaml`; nothing
  invented. A view needing an endpoint that doesn't exist yet says so.
- **Chrome text uses chrome tokens.** Text on the navy chrome uses
  `--color-on-chrome` / `--color-on-chrome-muted`, never surface text tokens.
- **TS/inline colours are tokens too.** Colours in TS constants and inline
  `style={…}` are `var(--…)` too — the token linter only sees CSS.
