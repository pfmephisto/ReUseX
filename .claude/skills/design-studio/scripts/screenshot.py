#!/usr/bin/env python3
"""Screenshot a local HTML file or URL at several viewports for visual design review.

Examples:
  python screenshot.py page.html --out shots/
  python screenshot.py http://localhost:5173/viewport --out shots/ --theme dark
  python screenshot.py http://localhost:5173/viewport --out shots/ --theme light
  python screenshot.py deck.html --out shots/ --selector .slide --viewports 1920x1080
  python screenshot.py onepager.html --out shots/ --pdf

For the rux GUI, bring the app up first with `dev_env.sh start`, then capture the
route you changed in BOTH themes (`--theme dark` and `--theme light`) — the light
theme only re-points chrome tokens, so a hardcoded colour looks right in one and
wrong in the other. `--theme` seeds the app's own `reusex-theme` preference in
localStorage before load, so you see the real resolved theme, not merely
`prefers-color-scheme`. Headless Chromium renders with no display attached.

Setup (once): pip install playwright && python -m playwright install chromium
"""
import argparse
import sys
from pathlib import Path

DEFAULT_VIEWPORTS = {
    "desktop": (1440, 900),
    "tablet": (768, 1024),
    "mobile": (390, 844),
}


def to_url(target: str) -> str:
    if target.startswith(("http://", "https://", "file://")):
        return target
    path = Path(target).expanduser().resolve()
    if not path.exists():
        sys.exit(f"Not found: {path}")
    return path.as_uri()


def parse_viewports(spec):
    if not spec:
        return DEFAULT_VIEWPORTS
    out = {}
    for item in spec.split(","):
        item = item.strip()
        if item in DEFAULT_VIEWPORTS:
            out[item] = DEFAULT_VIEWPORTS[item]
        else:
            w, h = item.lower().split("x")
            out[item] = (int(w), int(h))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("target", help="HTML file path or URL")
    p.add_argument("--out", default="shots", help="Output directory (default: shots)")
    p.add_argument("--viewports", help="Comma list: desktop,tablet,mobile or WxH (e.g. 1920x1080)")
    p.add_argument("--viewport-only", action="store_true", help="Capture only the visible viewport, not the full page")
    p.add_argument("--selector", help="Screenshot each element matching this CSS selector (e.g. .slide)")
    p.add_argument("--dark", action="store_true", help="Also capture with prefers-color-scheme: dark")
    p.add_argument("--theme", choices=["light", "dark", "system"],
                   help="Force the rux GUI's own theme preference (seeds localStorage "
                        "'reusex-theme' before load) and matches prefers-color-scheme to it. "
                        "Use for a single deterministic capture per theme.")
    p.add_argument("--wait", type=int, default=500, help="Extra ms to wait after load for fonts/animations (default 500)")
    p.add_argument("--pdf", action="store_true", help="Also export a PDF (print CSS, @page size respected)")
    args = p.parse_args()

    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        sys.exit("Playwright not installed. Run: pip install playwright && python -m playwright install chromium")

    url = to_url(args.target)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # A "pass" is (playwright color_scheme, app theme preference or None, filename tag).
    # --theme forces the app's own preference deterministically; --dark keeps the old
    # prefers-color-scheme-only behaviour; the default is a single light pass.
    if args.theme:
        cs = "dark" if args.theme in ("dark", "system") else "light"
        passes = [(cs, args.theme, args.theme)]
    elif args.dark:
        passes = [("light", None, ""), ("dark", None, "dark")]
    else:
        passes = [("light", None, "")]
    saved = []

    with sync_playwright() as pw:
        try:
            browser = pw.chromium.launch()
        except Exception as e:  # browser binary missing
            sys.exit(f"Could not launch Chromium ({e}). Run: python -m playwright install chromium")

        for scheme, pref, tag in passes:
            for name, (w, h) in parse_viewports(args.viewports).items():
                ctx = browser.new_context(viewport={"width": w, "height": h}, color_scheme=scheme,
                                          device_scale_factor=1, reduced_motion="reduce")
                if pref is not None:
                    # Runs before the page's own scripts (incl. index.html's FOUC guard,
                    # which reads this exact key), so the app boots in the chosen theme.
                    ctx.add_init_script(
                        "try{localStorage.setItem('reusex-theme', %r)}catch(e){}" % pref
                    )
                page = ctx.new_page()
                page.goto(url, wait_until="networkidle")
                page.wait_for_timeout(args.wait)
                suffix = f"{name}" + (f"-{tag}" if tag else "")

                overflow = page.evaluate("document.documentElement.scrollWidth - window.innerWidth")
                if overflow > 1:
                    print(f"WARNING [{suffix}]: horizontal overflow of {overflow}px")

                if args.selector:
                    els = page.query_selector_all(args.selector)
                    if not els:
                        print(f"WARNING: no elements match {args.selector!r}")
                    for i, el in enumerate(els, 1):
                        f = out / f"{suffix}-{i:02d}.png"
                        el.screenshot(path=str(f))
                        saved.append(f)
                else:
                    f = out / f"{suffix}.png"
                    page.screenshot(path=str(f), full_page=not args.viewport_only)
                    saved.append(f)
                ctx.close()

        if args.pdf:
            page = browser.new_page()
            page.goto(url, wait_until="networkidle")
            page.wait_for_timeout(args.wait)
            f = out / "export.pdf"
            page.pdf(path=str(f), print_background=True, prefer_css_page_size=True)
            saved.append(f)

        browser.close()

    print("Saved:")
    for f in saved:
        print(f"  {f}")
    print("Now open each PNG and review it against references/critique-checklist.md.")


if __name__ == "__main__":
    main()
