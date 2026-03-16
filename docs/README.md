# ReUseX Documentation

Welcome to the ReUseX documentation! This directory contains comprehensive documentation for users and developers.

## For Users

### Getting Started
- **Installation Guide**: See main [README.md](../README.md) for build instructions
- **Process Walkthrough**: [guides/Process walk through.md](guides/Process%20walk%20through.md)
- **CLI Usage**: Run `rux --help` for command-line interface documentation

### User Guides
- [Data Loading](guides/DataLoader.md) - Working with datasets
- [OpenCV Integration](guides/OpenCV.md) - Computer vision operations
- [Testing Guide](guides/TESTING.md) - Running and writing tests
- [Documentation Guide](guides/DOCUMENTATION.md) - Building API docs

## For Developers

### Architecture
- [Architecture Overview](../ARCHITECTURE.md) - High-level repository structure
- [Detailed Architecture](design/architecture.md) - Module organization and data flow
- [API Design Review](design/api-design-review.md) - Public API audit and refactoring recommendations
- [Subproject Structure](design/subproject-structure.md) - Build system organization
- [Database Design](design/database-design.md) - RTABMapDatabase detailed design

### Contributing
- [Contributing Guide](../CONTRIBUTING.md) - Code standards and workflow
- [Testing Guide](../tests/README.md) - Writing and running tests
- Coverage tools: `./tools/coverage/generate_coverage.sh`

### API Reference

Build the API documentation with:
```bash
cmake --build build --target docs
xdg-open docs/api/html/index.html
```

Or view online: [GitHub Pages](https://your-org.github.io/ReUseX/) (if deployed)

## Repository Structure

```
ReUseX/
├── libs/reusex/        # Core C++ library
├── apps/rux/           # CLI application
├── bindings/python/    # Python bindings (future)
├── tests/              # Test suite
├── docs/               # This directory
│   ├── api/           # Doxygen output (generated)
│   ├── guides/        # User guides
│   └── design/        # Architecture docs
└── tools/             # Development tools
```

## Quick Links

- **Main README**: [../README.md](../README.md)
- **Architecture**: [../ARCHITECTURE.md](../ARCHITECTURE.md)
- **Contributing**: [../CONTRIBUTING.md](../CONTRIBUTING.md)
- **Test Documentation**: [../tests/README.md](../tests/README.md)
- **Python Bindings Status**: [../bindings/python/README.md](../bindings/python/README.md)

## Building Documentation

### API Documentation (Doxygen)
```bash
cmake -B build -DBUILD_DOCUMENTATION=ON
cmake --build build --target docs
```

Output: `docs/api/html/index.html`

### User Guides

User guides are written in Markdown and located in `docs/guides/`. They are human-readable and don't require building.

## Getting Help

- **Issues**: [GitHub Issues](https://github.com/your-org/ReUseX/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-org/ReUseX/discussions)
- **Questions**: Open an issue with the `question` label

## Documentation Standards

When contributing documentation:
- Use clear, concise language
- Include code examples where appropriate
- Keep guides focused on one topic
- Update this README when adding new documents
- Follow Markdown best practices

## License

Documentation is licensed under the same terms as the project: GPL-3.0-or-later

See [LICENSE.md](../LICENSE.md) for details.
