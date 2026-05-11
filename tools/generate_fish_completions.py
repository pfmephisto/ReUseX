#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Generate fish shell completions for the rux CLI by parsing help output.

This script builds a command tree by recursively invoking `rux --help` and
`rux [subcommand...] --help`, then generates fish completion rules that
provide context-aware tab completion for all commands and options.
"""

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Option:
    """Represents a command-line option (flag)."""
    short: Optional[str] = None
    long: Optional[str] = None
    description: str = ""
    requires_arg: bool = False
    arg_type: Optional[str] = None  # FILE, INT, FLOAT, etc.


@dataclass
class Subcommand:
    """Represents a subcommand in the CLI hierarchy."""
    name: str
    description: str
    parent_chain: List[str] = field(default_factory=list)
    options: List[Option] = field(default_factory=list)
    subcommands: List['Subcommand'] = field(default_factory=list)


class HelpParser:
    """Parse CLI11 help output to extract commands and options."""

    # Match option lines: "  -s,        --long :TYPE [default]  description"
    # CLI11 format: short flag (optional), comma, whitespace, long flag, type info, description
    OPTION_PATTERN = re.compile(
        r'^\s*(?:-([a-zA-Z]),)?\s*(?:--([a-z0-9-]+))?\s*(?::?([A-Z]+))?.*?\s{2,}(.*)$'
    )

    # Match subcommand lines in "Subcommands:" section
    # Format: "  subcommand                              Description text"
    SUBCOMMAND_PATTERN = re.compile(r'^\s+(\w+)\s{2,}(.+)$')

    def parse_help(self, help_text: str) -> Tuple[List[Option], List[Tuple[str, str]]]:
        """
        Parse help output to extract options and subcommands.

        Returns:
            (options, subcommands) where subcommands is [(name, description), ...]
        """
        options = []
        subcommands = []

        lines = help_text.split('\n')
        in_options = False
        in_subcommands = False

        for line in lines:
            # Detect section headers (CLI11 uses "OPTIONS:" and "SUBCOMMANDS:" in uppercase)
            if line.startswith('OPTIONS:') or line.startswith('Options:'):
                in_options = True
                in_subcommands = False
                continue
            elif line.startswith('SUBCOMMANDS:') or line.startswith('Subcommands:'):
                in_options = False
                in_subcommands = True
                continue
            elif line and not line.startswith(' '):
                # New section started
                in_options = False
                in_subcommands = False

            # Parse options
            if in_options and line.strip():
                match = self.OPTION_PATTERN.match(line)
                if match:
                    short, long, arg_type, desc = match.groups()
                    if short or long:
                        options.append(Option(
                            short=short,
                            long=long,
                            description=desc.strip() if desc else "",
                            requires_arg=bool(arg_type),
                            arg_type=arg_type
                        ))

            # Parse subcommands
            elif in_subcommands and line.strip():
                match = self.SUBCOMMAND_PATTERN.match(line)
                if match:
                    name, desc = match.groups()
                    subcommands.append((name, desc.strip()))

        return options, subcommands


class CommandTreeBuilder:
    """Build hierarchical command tree by recursively invoking help."""

    def __init__(self, rux_binary: str):
        self.rux_binary = rux_binary
        self.parser = HelpParser()

    def get_help(self, command_chain: List[str]) -> str:
        """Get help output for a command chain."""
        cmd = [self.rux_binary] + command_chain + ['--help']
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            print(f"Warning: Timeout getting help for {' '.join(cmd)}", file=sys.stderr)
            return ""
        except FileNotFoundError:
            print(f"Error: rux binary not found at {self.rux_binary}", file=sys.stderr)
            sys.exit(1)

    def build_tree(self, command_chain: List[str] = None) -> Subcommand:
        """Recursively build command tree starting from command_chain."""
        if command_chain is None:
            command_chain = []

        help_text = self.get_help(command_chain)
        options, subcommand_list = self.parser.parse_help(help_text)

        # Create root or subcommand node
        if not command_chain:
            node = Subcommand(name="rux", description="ReUseX CLI", parent_chain=[])
        else:
            node = Subcommand(
                name=command_chain[-1],
                description="",  # Will be filled by parent
                parent_chain=command_chain[:-1],
                options=options
            )

        # Add options to node
        node.options = options

        # Recursively build subcommands
        for subcmd_name, subcmd_desc in subcommand_list:
            child_chain = command_chain + [subcmd_name]
            child = self.build_tree(child_chain)
            child.description = subcmd_desc
            node.subcommands.append(child)

        return node


class FishCompletionGenerator:
    """Generate fish shell completion file from command tree."""

    def __init__(self, root: Subcommand):
        self.root = root
        self.lines: List[str] = []

    def generate(self) -> str:
        """Generate complete fish completion script."""
        self._emit_header()
        self._emit_global_options()
        self._emit_subcommands(self.root, [])
        return '\n'.join(self.lines)

    def _emit_header(self):
        """Emit file header and setup."""
        self.lines.extend([
            "# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen",
            "# SPDX-License-Identifier: GPL-3.0-or-later",
            "",
            "# Fish shell completions for rux CLI",
            "# Generated by scripts/generate_fish_completions.py",
            "",
            "# Disable default file completions",
            "complete -c rux -f",
            "",
        ])

    def _emit_global_options(self):
        """Emit global options that are always available."""
        self.lines.append("# Global options (available at all levels)")
        for opt in self.root.options:
            self._emit_option("rux", opt, None)
        self.lines.append("")

    def _emit_subcommands(self, node: Subcommand, parent_chain: List[str]):
        """Recursively emit subcommand completions."""
        if not node.subcommands:
            # Leaf node - emit options only
            if node.options and parent_chain:
                self._emit_command_options(node, parent_chain)
            return

        # Emit subcommand names
        self._emit_subcommand_names(node, parent_chain)

        # Emit options for this level
        if node.options and parent_chain:
            self._emit_command_options(node, parent_chain)

        # Recurse into children
        for child in node.subcommands:
            self._emit_subcommands(child, parent_chain + [child.name])

    def _emit_subcommand_names(self, node: Subcommand, parent_chain: List[str]):
        """Emit completion rules for subcommand names."""
        if not node.subcommands:
            return

        # Build list of all subcommand names at this level
        subcmd_names = [sc.name for sc in node.subcommands]

        # Build condition
        if parent_chain:
            # We're in a subcommand - only show if parent seen and no child seen yet
            parent_cond = self._build_seen_condition(parent_chain)
            not_seen_cond = self._build_not_seen_condition(subcmd_names)
            condition = f"{parent_cond}; and {not_seen_cond}"
        else:
            # Top-level - only show if no subcommand seen yet
            all_top_level = [sc.name for sc in self.root.subcommands]
            condition = self._build_not_seen_condition(all_top_level)

        # Emit section header
        level_name = " ".join(parent_chain) if parent_chain else "top-level"
        self.lines.append(f"# {level_name} subcommands")

        # Emit completion for each subcommand
        for sc in node.subcommands:
            self.lines.append(
                f'complete -c rux -n "{condition}" '
                f'-a "{sc.name}" -d "{self._escape(sc.description)}"'
            )
        self.lines.append("")

    def _emit_command_options(self, node: Subcommand, parent_chain: List[str]):
        """Emit options for a specific command."""
        if not node.options:
            return

        # Build condition - must have seen the full parent chain
        condition = self._build_seen_condition(parent_chain)

        self.lines.append(f"# Options for: {' '.join(parent_chain)}")
        for opt in node.options:
            self._emit_option("rux", opt, condition)
        self.lines.append("")

    def _emit_option(self, cmd: str, opt: Option, condition: Optional[str]):
        """Emit completion rule for a single option."""
        parts = [f"complete -c {cmd}"]

        if condition:
            parts.append(f'-n "{condition}"')

        if opt.short:
            parts.append(f"-s {opt.short}")

        if opt.long:
            parts.append(f"-l {opt.long}")

        if opt.description:
            parts.append(f'-d "{self._escape(opt.description)}"')

        # Handle argument requirements
        if opt.requires_arg:
            parts.append("-r")  # Requires argument, no file completion
            # Special case: file paths should allow file completion
            if opt.arg_type in ["FILE", "PATH", "DIR"]:
                parts.append("-F")

        self.lines.append(" ".join(parts))

    def _build_seen_condition(self, chain: List[str]) -> str:
        """Build fish condition for 'has seen this command chain'."""
        if not chain:
            return ""
        if len(chain) == 1:
            return f"__fish_seen_subcommand_from {chain[0]}"
        # Multiple levels - need to check all
        conditions = [f"__fish_seen_subcommand_from {cmd}" for cmd in chain]
        return "; and ".join(conditions)

    def _build_not_seen_condition(self, names: List[str]) -> str:
        """Build fish condition for 'has not seen any of these commands'."""
        return f"not __fish_seen_subcommand_from {' '.join(names)}"

    def _escape(self, text: str) -> str:
        """Escape text for fish string literals."""
        return text.replace('"', '\\"').replace('$', '\\$')


def main():
    parser = argparse.ArgumentParser(
        description="Generate fish shell completions for rux CLI"
    )
    parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output file path (e.g., completions/rux.fish)"
    )
    parser.add_argument(
        "--rux-binary",
        default="rux",
        help="Path to rux executable (default: rux)"
    )

    args = parser.parse_args()

    print(f"Building command tree from {args.rux_binary}...", file=sys.stderr)
    builder = CommandTreeBuilder(args.rux_binary)
    root = builder.build_tree()

    print(f"Generating fish completions...", file=sys.stderr)
    generator = FishCompletionGenerator(root)
    completions = generator.generate()

    print(f"Writing to {args.output}...", file=sys.stderr)
    with open(args.output, 'w') as f:
        f.write(completions)

    print(f"Success! Generated {len(completions.splitlines())} lines of completions.", file=sys.stderr)
    print(f"\nTo install:", file=sys.stderr)
    print(f"  mkdir -p ~/.config/fish/completions", file=sys.stderr)
    print(f"  cp {args.output} ~/.config/fish/completions/", file=sys.stderr)
    print(f"  fish -c 'fish_update_completions'", file=sys.stderr)


if __name__ == "__main__":
    main()
