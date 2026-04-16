#!/usr/bin/env fish
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

# Test fish shell completions for rux CLI
# Usage: fish tests/test_completions.fish

# Load the completions
source completions/rux.fish

set -g test_exit_code 0

function test_completion
    set -l description $argv[1]
    set -l command $argv[2]
    set -l expected_pattern $argv[3]

    set -l result (complete -C "$command" | string match -r "$expected_pattern")

    if test -n "$result"
        echo "✓ $description"
    else
        echo "✗ $description"
        echo "  Command: $command"
        echo "  Expected pattern: $expected_pattern"
        set -g test_exit_code 1
    end
end

echo "Testing rux fish completions..."
echo ""

# Test global options
test_completion "Global option -v available" "rux -" "^-v"
test_completion "Global option --verbose available" "rux --" "^--verbose"
test_completion "Global option --project available" "rux --" "^--project"

# Test top-level subcommands
test_completion "Top-level: create available" "rux " "^create"
test_completion "Top-level: import available" "rux " "^import"
test_completion "Top-level: export available" "rux " "^export"
test_completion "Top-level: get available" "rux " "^get"
test_completion "Top-level: view available" "rux " "^view"

# Test create subcommands
test_completion "create: clouds subcommand available" "rux create " "^clouds"
test_completion "create: planes subcommand available" "rux create " "^planes"
test_completion "create: mesh subcommand available" "rux create " "^mesh"
test_completion "create: annotate subcommand available" "rux create " "^annotate"

# Test import subcommands
test_completion "import: rtabmap subcommand available" "rux import " "^rtabmap"
test_completion "import: materialepas subcommand available" "rux import " "^materialepas"

# Test export subcommands
test_completion "export: rhino subcommand available" "rux export " "^rhino"
test_completion "export: speckle subcommand available" "rux export " "^speckle"

# Test command-specific options
test_completion "create clouds: --grid option available" "rux create clouds --" "^--grid"
test_completion "create planes: --min-cluster-size option available" "rux create planes --" "^--min-cluster-size"
test_completion "export rhino: --output option available" "rux export rhino --" "^--output"

# Test that subcommands don't appear in wrong context
set -l wrong_context (complete -C "rux get " | string match -r "^clouds")
if test -z "$wrong_context"
    echo "✓ Subcommands don't appear in wrong context (rux get)"
else
    echo "✗ Subcommands appearing in wrong context"
    set -g test_exit_code 1
end

echo ""
if test $test_exit_code -eq 0
    echo "All tests passed!"
else
    echo "Some tests failed!"
end

exit $test_exit_code
