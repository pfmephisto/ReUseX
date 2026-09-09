// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { StageParameter } from '../api/types';
import {
  buildParameters,
  changedCount,
  formErrors,
  initialFormState,
  isDefault,
  parseField,
} from '../pipeline/params';
import { STAGES } from './fixtures';

/** The recorded descriptors for one stage, as the server actually sends them. */
function parametersOf(stage: string): StageParameter[] {
  const found = STAGES.stages.find((entry) => entry.stage === stage);
  if (!found) throw new Error(`no recorded stage '${stage}'`);
  return found.parameters;
}

function parameterOf(stage: string, key: string): StageParameter {
  const found = parametersOf(stage).find((parameter) => parameter.key === key);
  if (!found) throw new Error(`no recorded parameter '${stage}.${key}'`);
  return found;
}

describe('initialFormState', () => {
  it('starts every field at the server-declared default', () => {
    const parameters = parametersOf('planes');
    const state = initialFormState(parameters);

    for (const parameter of parameters) {
      if (parameter.type === 'boolean') expect(state[parameter.key]).toBe(parameter.default);
      else if (parameter.default === null) expect(state[parameter.key]).toBe('');
      else expect(state[parameter.key]).toBe(String(parameter.default));
    }
  });

  it('reports every field as unchanged', () => {
    const parameters = parametersOf('rooms');
    expect(changedCount(parameters, initialFormState(parameters))).toBe(0);
    expect(formErrors(parameters, initialFormState(parameters))).toEqual({});
  });
});

describe('buildParameters', () => {
  it('sends nothing at all when the form is untouched', () => {
    // This is the whole contract of the form: an untouched run must be
    // byte-identical to `rux create planes` with no flags.
    const parameters = parametersOf('planes');
    expect(buildParameters(parameters, initialFormState(parameters))).toEqual({});
  });

  it('omits a presence-sensitive key left at its default', () => {
    // #214: sending `plane_dist_threshold` AT ALL pins it and switches off
    // adaptive derivation. A form that echoed defaults back would silently
    // disable adaptivity for every GUI-started run, and the GUI and the CLI
    // would disagree about what "default" means.
    const parameters = parametersOf('planes');
    const pinned = parameterOf('planes', 'plane_dist_threshold');
    expect(pinned.presence_sensitive).toBe(true);

    const state = initialFormState(parameters);
    // Same number, typed with a trailing zero — still the default, still omitted.
    state.plane_dist_threshold = `${String(pinned.default)}0`;

    const body = buildParameters(parameters, state);
    expect(body).not.toHaveProperty('plane_dist_threshold');
    expect(body).toEqual({});
  });

  it('sends a presence-sensitive key the moment its value really differs', () => {
    const parameters = parametersOf('planes');
    const state = initialFormState(parameters);
    state.plane_dist_threshold = '0.02';

    expect(buildParameters(parameters, state)).toEqual({ plane_dist_threshold: 0.02 });
  });

  it('sends only the fields that differ', () => {
    const parameters = parametersOf('rooms');
    const state = initialFormState(parameters);
    state.resolution = '1.5';
    state.max_iter = '250';

    expect(buildParameters(parameters, state)).toEqual({ resolution: 1.5, max_iter: 250 });
    expect(changedCount(parameters, state)).toBe(2);
  });

  it('sends a toggled boolean as a boolean', () => {
    const parameters = parametersOf('planes');
    const state = initialFormState(parameters);
    state.adaptive = false;

    expect(buildParameters(parameters, state)).toEqual({ adaptive: false });
  });

  it('parses a comma-separated label list into numbers', () => {
    const parameters = parametersOf('instances');
    const state = initialFormState(parameters);
    state.labels = '3, 7,  11';

    expect(buildParameters(parameters, state)).toEqual({ labels: [3, 7, 11] });
  });

  it('leaves an untouched optional field out rather than sending an empty one', () => {
    const parameters = parametersOf('planes');
    const filter = parameterOf('planes', 'filter');
    expect(filter.default).toBeNull();

    const state = initialFormState(parameters);
    expect(state.filter).toBe('');
    expect(buildParameters(parameters, state)).toEqual({});
  });

  it('skips an unparseable field instead of sending garbage', () => {
    const parameters = parametersOf('rooms');
    const state = initialFormState(parameters);
    state.grid_size = 'not a number';
    state.beta = '0.2';

    expect(buildParameters(parameters, state)).toEqual({ beta: 0.2 });
  });
});

describe('parseField', () => {
  const gridSize = () => parameterOf('rooms', 'grid_size');
  const maxIter = () => parameterOf('rooms', 'max_iter');

  it('refuses a value below the declared minimum', () => {
    const parameter = gridSize();
    expect(parameter.minimum).not.toBeNull();
    const result = parseField(parameter, String((parameter.minimum as number) - 1));
    expect(result.ok).toBe(false);
  });

  it('refuses a value above the declared maximum', () => {
    const parameter = gridSize();
    const result = parseField(parameter, String((parameter.maximum as number) + 1));
    expect(result.ok).toBe(false);
  });

  it('accepts the declared bounds themselves', () => {
    const parameter = gridSize();
    expect(parseField(parameter, String(parameter.minimum)).ok).toBe(true);
    expect(parseField(parameter, String(parameter.maximum)).ok).toBe(true);
  });

  it('refuses a fractional value for an integer parameter', () => {
    expect(parseField(maxIter(), '10.5')).toEqual({
      ok: false,
      error: 'Must be a whole number',
    });
  });

  it('refuses a blank required field rather than silently defaulting it', () => {
    // Deleting the contents of a required box is an edit, and quietly running
    // with the old default would ignore what the user just did.
    const result = parseField(gridSize(), '');
    expect(result.ok).toBe(false);
  });

  it('treats a blank optional field as absent', () => {
    expect(parseField(parameterOf('planes', 'filter'), '')).toEqual({
      ok: true,
      value: undefined,
    });
  });

  it('refuses a negative or fractional label', () => {
    const labels = parameterOf('instances', 'labels');
    expect(parseField(labels, '3,-1').ok).toBe(false);
    expect(parseField(labels, '3,1.5').ok).toBe(false);
  });

  it('trims surrounding whitespace before parsing', () => {
    expect(parseField(gridSize(), '  0.4  ')).toEqual({ ok: true, value: 0.4 });
  });
});

describe('isDefault', () => {
  it('treats an equal number typed differently as unchanged', () => {
    const parameter = parameterOf('clouds', 'resolution');
    expect(parameter.default).toBe(0.05);
    expect(isDefault(parameter, '0.05')).toBe(true);
    expect(isDefault(parameter, '0.050')).toBe(true);
    expect(isDefault(parameter, ' 0.05 ')).toBe(true);
    expect(isDefault(parameter, '0.06')).toBe(false);
  });

  it('compares strings exactly', () => {
    const parameter = parameterOf('instances', 'semantic_cloud');
    expect(isDefault(parameter, 'labels')).toBe(true);
    expect(isDefault(parameter, 'labels2')).toBe(false);
  });
});

describe('formErrors', () => {
  it('reports one entry per bad field and nothing when the form is clean', () => {
    const parameters = parametersOf('instances');
    const state = initialFormState(parameters);
    expect(formErrors(parameters, state)).toEqual({});

    state.cluster_tolerance = 'x';
    state.min_cluster_size = '0';
    const errors = formErrors(parameters, state);
    expect(Object.keys(errors).sort()).toEqual(['cluster_tolerance', 'min_cluster_size']);
  });
});
