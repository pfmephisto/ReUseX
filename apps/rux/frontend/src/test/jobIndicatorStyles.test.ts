// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * JobIndicator dot styling — the busy modifier must COMPOSE with the
 * connection colour, not replace it.
 *
 * The component applies `.dot .<connection> .busy` together, all at the same
 * specificity, so a `background` declared inside `.busy` wins purely on source
 * order and repaints the connection colour: `closed` + 1 running job rendered
 * pixel-identically to `open` + 1 running job (#324). That is the single most
 * misleading thing this indicator can say, and it is invisible to any test
 * that only inspects the DOM — the classes are all present either way. So the
 * contract is asserted against the stylesheet itself.
 */

import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const css = readFileSync(
  new URL('../components/JobIndicator.module.css', import.meta.url),
  'utf8',
);

/** Declaration body of the first top-level `selector { ... }` rule. */
function ruleBody(selector: string): string {
  const at = css.indexOf(`${selector} {`);
  expect(at, `${selector} rule is missing`).toBeGreaterThanOrEqual(0);
  return css.slice(at, css.indexOf('}', at));
}

/** Body of an `@keyframes name { ... }` block, braces balanced. */
function keyframesBody(name: string): string {
  const at = css.indexOf(`@keyframes ${name} {`);
  expect(at, `@keyframes ${name} is missing`).toBeGreaterThanOrEqual(0);
  let depth = 0;
  for (let i = css.indexOf('{', at); i < css.length; i++) {
    if (css[i] === '{') depth++;
    else if (css[i] === '}' && --depth === 0) return css.slice(at, i);
  }
  throw new Error(`@keyframes ${name} is unterminated`);
}

describe('JobIndicator.module.css', () => {
  it('JobIndicator_BusyModifier_DeclaresNoBackground', () => {
    // Background is the connection channel. Busy may only spend a different
    // one (the ring + motion), or it silently overrides `.closed`.
    expect(ruleBody('.busy')).not.toMatch(/background/);
  });

  it('JobIndicator_ConnectionStates_EachDeclareTheirOwnBackground', () => {
    for (const state of ['.connecting', '.open', '.closed']) {
      expect(ruleBody(state), `${state} must set the dot colour`).toMatch(
        /background:\s*var\(--color-status-[a-z]+\)/,
      );
    }
  });

  it('JobIndicator_BusyPulse_LeavesTheDotColourAtFullOpacity', () => {
    // A dot fading to 0.35 is a dot whose connection colour is hard to name.
    const animation = /animation:\s*([\w-]+)/.exec(ruleBody('.busy'));
    expect(animation, '.busy must carry the pulse animation').not.toBeNull();
    expect(keyframesBody(animation![1])).not.toMatch(/opacity/);
  });

  it('JobIndicator_ReducedMotion_StopsThePulseButKeepsTheRing', () => {
    // Keyframes ignore the zeroed --duration-* tokens; they must be stopped
    // explicitly, and the static ring is what still says "busy" afterwards.
    expect(css).toMatch(
      /@media \(prefers-reduced-motion: reduce\)[\s\S]*\.busy\s*\{[^}]*animation:\s*none/,
    );
    expect(ruleBody('.busy')).toMatch(/box-shadow/);
  });
});
