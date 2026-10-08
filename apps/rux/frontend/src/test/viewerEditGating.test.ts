// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A viewer of a case sees no control that writes to it (final review #8).
 *
 * The rule — `useCanEdit()` wherever a mutation button renders — is about
 * where a hook is called, which no DOM-less test can observe by rendering.
 * So it is asserted against the sources: every component that writes to the
 * case (a mutating `api.*` call, or the mutation queue) calls `useCanEdit()`,
 * or is listed below with the reason it need not. A new screen that writes
 * fails here until it hides its controls from viewers or says why not.
 */

import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';

const SRC = fileURLToPath(new URL('..', import.meta.url));

function sources(dir: string): string[] {
  return readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    if (statSync(path).isDirectory()) return name === 'test' ? [] : sources(path);
    return /\.tsx?$/.test(name) && !/\.test\.tsx?$/.test(name) ? [path] : [];
  });
}

const read = (rel: string) => readFileSync(join(SRC, rel), 'utf8');

/**
 * The client's methods that send a write: their bodies use POST, PUT, PATCH
 * or DELETE. Read from client.ts itself, so a new endpoint is covered.
 */
function mutatingApiMethods(): string[] {
  const client = read('api/client.ts');
  const heads = [...client.matchAll(/^ {2}(private )?(?:async )?(\w+)\(/gm)];
  return heads
    .map((m, i) => ({
      helper: m[1] !== undefined,
      name: m[2],
      body: client.slice(m.index, heads[i + 1]?.index ?? client.length),
    }))
    .filter(({ helper }) => !helper)
    .filter(({ body }) =>
      /this\.(postJson|putJson|patchJson|deleteNoContent)\b|method: '(POST|PUT|PATCH|DELETE)'/.test(body),
    )
    .map(({ name }) => name);
}

/** Writers that need no useCanEdit, and why. */
const EXEMPT: Record<string, string> = {
  'app/useMutationQueue.ts': 'the queue itself: refuses a viewer write before the request',
  'app/CaseRoleContext.tsx': 'defines useCanEdit',
  'routes/LoginPage.tsx': 'no case is open',
  'routes/SagerPage.tsx': 'creating and uploading a case is server-level: any signed-in user may',
  'components/ApiTokens.tsx': "the user's own API tokens, not the case",
  'routes/IndstillingerPage.tsx': 'members: gated by canManageMembers (owners only)',
  'routes/SkabelonerPage.tsx': 'its controls render in TemplateList, TemplateEditor and ColumnList',
  'components/FramePairInspector.tsx': 'a compute-only POST (descriptor match); it writes nothing',
};

/** Components that render write controls for a page that does the writing. */
const BUTTON_HOSTS = [
  'components/skabeloner/TemplateList.tsx',
  'components/skabeloner/TemplateEditor.tsx',
  'components/skabeloner/ColumnList.tsx',
  'components/kortlaegning/SurveyTable.tsx',
  'components/kortlaegning/DetailPanel.tsx',
  'components/kortlaegning/EditDialog.tsx',
  'components/miljoe/SampleCard.tsx',
  'components/overblik/CaseHero.tsx',
  'components/rapport/DataExportPanel.tsx',
];

describe('viewer edit gating', () => {
  const methods = mutatingApiMethods();
  const writes = new RegExp(`\\b(?:api\\.(?:${methods.join('|')})|\\bmutate|useMutationQueue)\\(`);

  it('finds the mutating client methods', () => {
    expect(methods).toEqual(expect.arrayContaining(['patchSample', 'deleteTemplate', 'submitJob', 'addPoseGraphEdge']));
    expect(methods).not.toContain('samples');
  });

  const writers = sources(SRC)
    .map((path) => relative(SRC, path).split('\\').join('/'))
    .filter((rel) => !rel.startsWith('api/'))
    .filter((rel) => writes.test(read(rel)));

  it('finds the writers', () => {
    expect(writers).toEqual(
      expect.arrayContaining([
        'components/LabelsPane.tsx',
        'components/PoseGraphEdgeInspector.tsx',
        'routes/MiljoePage.tsx',
        'routes/KortlaegningPage.tsx',
      ]),
    );
  });

  it('every component that writes to the case calls useCanEdit, or says why not', () => {
    const offenders = writers.filter((rel) => !(rel in EXEMPT) && !read(rel).includes('useCanEdit('));
    expect(offenders).toEqual([]);
  });

  it('the members page gates on canManageMembers', () => {
    expect(read('routes/IndstillingerPage.tsx')).toContain('canManageMembers(');
  });

  it.each(BUTTON_HOSTS)('%s hides its write controls from a viewer', (rel) => {
    expect(read(rel)).toContain('useCanEdit(');
  });
});
