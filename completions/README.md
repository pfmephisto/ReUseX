# Shell Completions for rux CLI

This directory contains shell completion files for the `rux` command-line tool.

## Fish Shell Completions

### Installation

**Manual installation** (copy to user directory):
```bash
mkdir -p ~/.config/fish/completions
cp rux.fish ~/.config/fish/completions/
fish -c 'fish_update_completions'
```

**System-wide installation** (requires root):
```bash
sudo cp rux.fish /usr/share/fish/vendor_completions.d/
```

### Generation

Completions are automatically generated from the CLI11 help output by parsing `rux --help` and recursively building the command tree.

**Prerequisites**:
- Built `rux` binary at `build/apps/rux/rux`
- Python 3 and fish shell (provided by `nix develop .#completions`)

**Generate completions**:
```bash
# Using Nix dev shell (recommended)
nix develop .#completions
generate-completions

# Or run script directly
python3 scripts/generate_fish_completions.py \
  -o completions/rux.fish \
  --rux-binary build/apps/rux/rux
```

### Testing

Run the test suite to verify completions work correctly:
```bash
fish tests/test_completions.fish
```

## Features

The fish completions provide:
- ✅ Global options available at all command levels (`-v`, `-p`, `-D`, etc.)
- ✅ Top-level subcommands with descriptions
- ✅ Hierarchical nested subcommands (e.g., `rux create clouds`)
- ✅ Context-aware option completions
- ✅ Automatic file path completion for file/path arguments
- ✅ No false positives (options only appear in correct context)

## Future Enhancements

- [ ] Add bash completion generation
- [ ] Add zsh completion generation
- [ ] Implement dynamic completions (database resource names)
- [ ] Add completion for filter expressions
- [ ] Consider runtime completion command (`rux --completion fish`)
