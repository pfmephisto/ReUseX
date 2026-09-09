// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sparse-patch construction for the two editors on `/data`.
 *
 * Both `PATCH /clouds/{name}/labels` and `PATCH /materials/{guid}` take a
 * *sparse* map and leave everything they are not told about alone. That is the
 * contract's central decision and it puts a real obligation on this client:
 * sending back the whole draft would work — the values are the same — right up
 * until a passport carries a MaterialEPAS field this GUI never rendered, at
 * which point a full send is indistinguishable from a deliberate edit of a
 * field the user never saw.
 *
 * So the diff below is the load-bearing piece, and it is a pure function for
 * exactly that reason: what is *omitted* is as much a part of the request as
 * what is sent, and neither is visible from a rendered component.
 */

/**
 * The one cloud whose label names the server refuses to rename.
 *
 * `create instances` writes names of the form `SM<class>-<id> (<n>p)`, and both
 * the v10 schema migration and `io/export_scene.cpp` parse the semantic class
 * back out of that string. The server answers 409, and this constant is what
 * lets the UI disable the fields *as well* — offering an edit that is always
 * refused is a worse experience than not offering it, and handling only the
 * 409 would mean the user learns the rule by breaking it.
 */
export const UNRENAMABLE_CLOUD = 'instances';

/** False for the clouds whose legend is a record rather than a caption. */
export function isLegendEditable(cloud: string): boolean {
  return cloud !== UNRENAMABLE_CLOUD;
}

/**
 * Keys whose value differs from the original.
 *
 * Deletion is not expressible here — this is the legend patch, where the only
 * operation is a rename, and the server rejects both an empty name and an id
 * that is not already in the legend. A key edited and then edited back to its
 * original value is **omitted**, so a user who undoes their typing sends
 * nothing rather than a no-op write that still takes the project's writer lock.
 */
export function diffLabels(
  original: Readonly<Record<string, string>>,
  draft: Readonly<Record<string, string>>,
): Record<string, string> {
  const patch: Record<string, string> = {};
  for (const [id, name] of Object.entries(draft)) {
    if (!(id in original)) continue; // Not renamable: the server 400s on it.
    if (original[id] !== name) patch[id] = name;
  }
  return patch;
}

/** One reason a legend draft cannot be sent, addressed to the user. */
export interface LabelIssue {
  id: string;
  message: string;
}

/**
 * Validate a legend draft against what the server will accept.
 *
 * Checked here rather than left to the 400 because the failures are typing
 * mistakes with obvious fixes — an emptied field, whitespace where a name was
 * — and a round trip that takes the writer lock to tell the user they cleared
 * a box is a worse answer than saying so in the field.
 *
 * This does **not** duplicate the server's authority: the same conditions are
 * still refused there, and the 400 path is still handled. It only avoids
 * spending a request to discover something already on screen.
 */
export function labelIssues(
  original: Readonly<Record<string, string>>,
  draft: Readonly<Record<string, string>>,
): LabelIssue[] {
  const issues: LabelIssue[] = [];
  for (const [id, name] of Object.entries(draft)) {
    if (!(id in original)) {
      issues.push({
        id,
        message: `Label ${id} is not in this cloud's legend — classes can be renamed, not created.`,
      });
      continue;
    }
    if (name.trim() === '') {
      issues.push({ id, message: 'A label name cannot be empty.' });
    }
  }
  return issues;
}

/**
 * The material patch: changed values as strings, removed keys as `null`.
 *
 * The draft models "cleared" as the key being **absent**, not as an empty
 * string, because those are different requests: `null` deletes the property,
 * while `""` stores an empty one. A form that conflated them would make
 * clearing a field unreachable through the UI.
 *
 * Three cases, all of which the tests pin:
 *
 *  - present in both, value changed → the new string
 *  - present in the original, gone from the draft → `null`
 *  - only in the draft → the new string (a property the passport never had)
 *
 * and one non-case: present in both and unchanged → omitted entirely, so an
 * edit typed and undone sends nothing.
 */
export function diffProperties(
  original: Readonly<Record<string, string>>,
  draft: Readonly<Record<string, string>>,
): Record<string, string | null> {
  const patch: Record<string, string | null> = {};

  for (const [key, value] of Object.entries(draft)) {
    if (!(key in original) || original[key] !== value) patch[key] = value;
  }
  for (const key of Object.keys(original)) {
    if (!(key in draft)) patch[key] = null;
  }
  return patch;
}

/** One reason a property draft cannot be sent. */
export interface PropertyIssue {
  key: string;
  message: string;
}

/**
 * Validate a property draft.
 *
 * Only the property *name* is constrained by the server (it may not be empty);
 * an empty value is a legitimate stored value. A name that is only whitespace
 * is caught here too — it passes the server's `empty()` check but is
 * indistinguishable from a blank row on screen, so accepting it would create a
 * property the user cannot find again.
 */
export function propertyIssues(draft: Readonly<Record<string, string>>): PropertyIssue[] {
  const issues: PropertyIssue[] = [];
  for (const key of Object.keys(draft)) {
    if (key.trim() === '') {
      issues.push({ key, message: 'A property name cannot be empty.' });
    }
  }
  return issues;
}

/**
 * One editable row of the property form.
 *
 * The form is an ordered list rather than the map it becomes, because a map has
 * no stable place to put a row whose name the user has not finished typing yet.
 * `added` marks a row the user created, which is the only kind whose *name* is
 * editable — renaming a stored property is expressible (delete the old key, set
 * the new one) but is not what a user editing a passport means to do, and
 * offering it would make a typo in a name look like an edit of a value.
 */
export interface PropertyRow {
  /**
   * Identity of the row itself, stable across renames and reorderings.
   *
   * Not the property name: a row whose name the user is halfway through typing
   * has no name yet, and keying the form by name would remount the input on
   * every keystroke and lose the caret.
   */
  id: string;
  key: string;
  value: string;
  /** True for a row the user added — the only kind whose name is editable. */
  added: boolean;
}

/**
 * Seed the form from a passport's stored properties.
 *
 * Ids are derived from the stored name, which is unique within a passport, so
 * a re-seed after a save keeps every row's identity and the form does not
 * remount under the user.
 */
export function propertyRows(properties: Readonly<Record<string, string>>): PropertyRow[] {
  return Object.entries(properties).map(([key, value]) => ({
    id: `stored:${key}`,
    key,
    value,
    added: false,
  }));
}

/**
 * Collapse the form's rows into the map the diff compares against.
 *
 * Rows with a blank name are dropped: a half-typed new row is not yet a
 * property, and including it would send `"": value` for the server to reject.
 * On a duplicate name the last row wins, matching what the server would end up
 * storing — but see {@link duplicateRowKeys}, which is what stops the form
 * getting there.
 */
export function rowsToDraft(rows: readonly PropertyRow[]): Record<string, string> {
  const draft: Record<string, string> = {};
  for (const row of rows) {
    if (row.key.trim() === '') continue;
    draft[row.key] = row.value;
  }
  return draft;
}

/**
 * Names that appear on more than one row.
 *
 * Silently collapsing them would drop one of the user's two edits with no
 * indication which, so the form refuses to save while any exist.
 */
export function duplicateRowKeys(rows: readonly PropertyRow[]): string[] {
  const seen = new Set<string>();
  const duplicates = new Set<string>();
  for (const row of rows) {
    const key = row.key.trim();
    if (key === '') continue;
    if (seen.has(key)) duplicates.add(key);
    seen.add(key);
  }
  return [...duplicates];
}

/** True when a patch would change nothing and so must not be sent at all. */
export function isEmptyPatch(patch: Readonly<Record<string, unknown>>): boolean {
  return Object.keys(patch).length === 0;
}

/**
 * Apply a sparse property patch locally — the optimistic half of the write.
 *
 * The same semantics the server applies, so the screen after an optimistic
 * update matches the screen after the response lands and the pane does not
 * flicker between two truths. Rolling back is just keeping the pre-patch map
 * and putting it back, which is why this returns a new object rather than
 * mutating.
 */
export function applyPropertyPatch(
  properties: Readonly<Record<string, string>>,
  patch: Readonly<Record<string, string | null>>,
): Record<string, string> {
  const next: Record<string, string> = { ...properties };
  for (const [key, value] of Object.entries(patch)) {
    if (value === null) delete next[key];
    else next[key] = value;
  }
  return next;
}

/** Apply a sparse legend patch locally. Renames only — nothing is removed. */
export function applyLabelPatch(
  labels: Readonly<Record<string, string>>,
  patch: Readonly<Record<string, string>>,
): Record<string, string> {
  return { ...labels, ...patch };
}
