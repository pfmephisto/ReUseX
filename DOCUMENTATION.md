# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ReUseX API Documentation Guide

This guide explains how to generate and use the ReUseX API documentation.

## Overview

ReUseX uses Doxygen to automatically generate comprehensive API documentation from the C++ source code. The documentation includes:

- **API Reference**: Complete documentation for all classes, functions, and namespaces
- **Class Diagrams**: Inheritance hierarchies and collaboration diagrams
- **Source Browser**: Browse and search through the source code
- **Module Organization**: Documentation organized by functional modules
- **Cross-references**: Links between related functions, classes, and files

## Prerequisites

### Required
- **Doxygen** (version 1.8.0 or higher)

### Optional
- **Graphviz** (dot tool): Required for generating class diagrams and dependency graphs

### Installation

**Ubuntu/Debian:**
```bash
sudo apt-get install doxygen graphviz
```

**Fedora/RHEL:**
```bash
sudo dnf install doxygen graphviz
```

**macOS (Homebrew):**
```bash
brew install doxygen graphviz
```

**NixOS/Nix:**
```bash
nix-shell -p doxygen graphviz
# Or enter the development shell:
nix develop
```

## Generating Documentation

### Method 1: Using CMake (Recommended)

From your build directory:

```bash
# Configure with documentation enabled (ON by default)
cmake -B build -DBUILD_DOCUMENTATION=ON

# Generate the documentation
cmake --build build --target doc
```

### Method 2: Using Doxygen Directly

From the project root:

```bash
doxygen Doxyfile
```

## Output Location

Generated documentation is placed in the `doc/` directory:
- **HTML**: `doc/html/index.html` - Main entry point for the HTML documentation
- **Search**: Full-text search available in the HTML output

## Viewing Documentation

Open the documentation in your web browser:

```bash
# Linux
xdg-open doc/html/index.html

# macOS
open doc/html/index.html

# Windows
start doc/html/index.html
```

## Documentation Structure

The generated documentation is organized into several sections:

1. **Main Page**: Overview from README.md
2. **Namespaces**: Organized by namespace hierarchy
3. **Classes**: Complete class reference with inheritance diagrams
4. **Files**: Documentation organized by source files
5. **Examples**: Code examples (when available)

### Key Namespaces

- **ReUseX**: Core library functionality
  - **ReUseX::io**: Input/output operations
  - **ReUseX::core**: Core data structures and algorithms
- **rux**: Command-line interface utilities
- **pcl**: PCL extensions (custom implementations)
- **spdmon**: Monitoring utilities

## CMake Options

Control documentation generation with CMake options:

```bash
# Disable documentation generation
cmake -B build -DBUILD_DOCUMENTATION=OFF

# Enable documentation (default)
cmake -B build -DBUILD_DOCUMENTATION=ON
```

## Customizing Documentation

### Doxyfile Configuration

The `Doxyfile` in the project root contains all Doxygen settings. Key configurations:

- **PROJECT_NAME**: ReUseX
- **INPUT**: Source directories (include/ReUseX, include/rux, etc.)
- **OUTPUT_DIRECTORY**: doc/
- **GENERATE_HTML**: YES (HTML output enabled)
- **GENERATE_LATEX**: NO (LaTeX output disabled by default)
- **RECURSIVE**: YES (recurse into subdirectories)
- **EXTRACT_ALL**: YES (document all entities)
- **SOURCE_BROWSER**: YES (include source code browsing)
- **HAVE_DOT**: YES (use Graphviz for diagrams)

### Modifying Settings

Edit `Doxyfile` to customize:
- Output formats (HTML, LaTeX, XML)
- Extraction settings (private members, static functions)
- Diagram generation options
- Search engine configuration

## Writing Documentation

### Documentation Comments

Doxygen parses special comment blocks in the source code:

```cpp
/**
 * @brief Brief description of the function
 * 
 * Detailed description of what the function does,
 * including any important notes or warnings.
 * 
 * @param param1 Description of first parameter
 * @param param2 Description of second parameter
 * @return Description of return value
 * @throws std::runtime_error When something goes wrong
 * 
 * @code
 * // Example usage:
 * auto result = myFunction(10, 20);
 * @endcode
 */
int myFunction(int param1, int param2);
```

### Common Tags

- `@brief`: Short description
- `@param`: Parameter documentation
- `@return`: Return value description
- `@throws`: Exception documentation
- `@note`: Important notes
- `@warning`: Warnings
- `@see`: Cross-references
- `@code/@endcode`: Code examples
- `@todo`: TODO items

## Troubleshooting

### Common Issues

**Issue**: "Doxygen not found"
- **Solution**: Install Doxygen using your package manager

**Issue**: "dot not found" or missing diagrams
- **Solution**: Install Graphviz for diagram generation

**Issue**: Documentation is empty or incomplete
- **Solution**: Check that source files have proper documentation comments

**Issue**: Build fails with Doxygen warnings
- **Solution**: Review warnings and fix documentation syntax in source files

### Disabling Documentation

If you don't want to generate documentation:

```bash
cmake -B build -DBUILD_DOCUMENTATION=OFF
```

## Contributing to Documentation

When adding new code:

1. Add documentation comments to all public APIs
2. Include brief descriptions for all functions and classes
3. Document parameters, return values, and exceptions
4. Provide usage examples for complex functionality
5. Regenerate documentation to verify: `cmake --build build --target doc`

## Additional Resources

- [Doxygen Manual](https://www.doxygen.nl/manual/)
- [Doxygen Comment Syntax](https://www.doxygen.nl/manual/docblocks.html)
- [Graphviz Documentation](https://graphviz.org/documentation/)

## Version Control

The `doc/` output directory is excluded from version control (.gitignore).
Only the Doxyfile configuration and this guide are tracked in the repository.
