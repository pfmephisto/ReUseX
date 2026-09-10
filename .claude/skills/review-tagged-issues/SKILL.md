---
name: review-tagged-issues
description: Rewrite GitHub issues tagged "reword" from author shorthand into fully-contextualized, actionable issues; remove the tag when done. Use when invoked directly or when asked to process reword-tagged issues.
---

# Review Tagged Issues

## Purpose

Issues tagged `reword` were written in shorthand and need to be expanded into well-contextualized, actionable issues. For each such issue:

1. Investigate the codebase and project context
2. Find related issues
3. Rewrite the title and body with full detail
4. Remove the `reword` label

---

## Process

### 1. Discover

```bash
gh issue list --label reword --limit 100
```

### 2. Investigate each issue

For every issue found:

```bash
gh issue view <number>
```

Then gather context from:

- **Codebase** — `find`/`grep` for relevant source files, headers, CLI commands
- **Docs** — `docs/DIRECTION.md`, `docs/STANDARDS.md`, `docs/CONTRACTS.md`
- **Memory** — check the project memory index for relevant prior knowledge
- **Related issues** — search by keywords and by workstream label:
  ```bash
  gh issue list --search "<keywords>" --state all --limit 20
  gh issue list --label "workstream: <name>" --state all --limit 20
  ```
- **PRs** — check whether work is already in progress or merged:
  ```bash
  gh pr list --state all --search "<keywords>" --limit 10
  ```

### 3. Rewrite

Match the depth and format to the complexity of the issue.

**For features, integrations, or significant changes:**

```markdown
## Context

Why this is being tracked: the problem or need, and what prompted it.

## Current state

What exists today — link to source files, reference relevant CLI commands or API endpoints.

## Proposed approach

How to implement it. If there are open design questions or competing approaches, name them.

## Acceptance criteria

- Concrete, testable checklist of what "done" looks like

## Related

- #NNN — brief relationship note
```

**For explorations or feasibility studies:**

```markdown
Brief 1-2 sentence summary of what to investigate and why.

## Context

Current state and what gap this explores.

## Scope

What the investigation should produce (a recommendation, a prototype, a list of follow-up issues).

## Related

- #NNN — brief relationship note
```

### 4. Apply the edit

```bash
gh issue edit <number> --title "New title" --body "$(cat <<'EOF'
...body...
EOF
)"

gh issue edit <number> --remove-label reword
```

If the issue is missing a workstream label, add the appropriate one:

```bash
gh issue edit <number> --add-label "workstream: gui"
```

---

## Writing good titles

- Imperative or noun phrase: specific, not vague
- Name the component, module, or behavior
- Under 80 characters
- Examples:
  - "Add Gaussian splat viewer to rux GUI frontend" (not "Add gsplat viewer")
  - "Evaluate Potree-Next vs RUXP streaming for LOD rendering" (not "Explore using potree")

---

## Reference: well-written issues in this repo

- **#265** — GUI application plan: shows Context / Current state / Approach / Acceptance structure
- **#320** — Voxel LOD: shows Problem / Scope / Notes structure for a well-bounded technical issue
- **#240** — Gaussian splatting workstream: shows Motivation / Approach / Licensing / Acceptance
