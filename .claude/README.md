<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Claude Code Configuration

This directory contains Claude Code configuration for the ReUseX project.

## Directory Structure

```
.claude/
├── README.md                 # This file
├── settings.local.json       # Project-specific permissions and settings
├── memory/                   # Auto memory (project-specific, persistent)
│   └── MEMORY.md            # Key learnings and patterns
├── agents/                   # Custom agent definitions (if any)
└── agent-memory/            # Agent-specific memory files
```

## Configuration Files

### settings.local.json

Project-specific Claude Code settings:

- **Permissions**: Pre-approved commands (git, cmake, ctest, etc.)
- **Security**: Controls what Claude can do without prompting
- **Workflow**: Optimized for C++/CMake development

To add more permissions, edit this file or use the Claude Code UI.

### memory/MEMORY.md

Persistent project memory that survives across conversations:

- Database architecture patterns
- Known API quirks (RTABMap, PCL, etc.)
- Build system conventions
- Lessons learned from past issues

**Keep it concise!** Only the first 200 lines are loaded into system prompt.

## Usage

### For Developers

These files are checked into git to share project-specific Claude Code configuration across the team:

- Everyone gets the same pre-approved permissions
- Everyone benefits from documented learnings in MEMORY.md
- Consistent AI assistance experience

### For Claude

When working on this project:

1. **Always consult MEMORY.md** before making architectural decisions
2. **Update MEMORY.md** when you discover new patterns or fix tricky bugs
3. **Follow patterns** documented in CLAUDE.md and CONTRIBUTING_AI.md

## See Also

- `../CLAUDE.md` - High-level project guide for Claude Code
- `../CONTRIBUTING_AI.md` - AI-specific development patterns
- `memory/MEMORY.md` - Project-specific learnings
