# Contributing to ReUseX

Thank you for your interest in contributing to ReUseX! This document provides guidelines and best practices for contributing.

## Code of Conduct

Be respectful, inclusive, and collaborative. We welcome contributions from everyone.

## Getting Started

1. **Fork the repository**
2. **Clone your fork**: `git clone https://github.com/YOUR_USERNAME/ReUseX.git`
3. **Create a branch**: `git checkout -b feature/your-feature-name`
4. **Make changes** following the guidelines below
5. **Test your changes** (see Testing section)
6. **Submit a pull request**

## Repository Structure

ReUseX uses a subproject structure:
- `libs/reusex/`: Core C++ library
- `apps/rux/`: CLI application
- `bindings/python/`: Python bindings (future)
- `tests/`: Test suite
- `docs/`: Documentation

See `ARCHITECTURE.md` and `docs/design/subproject-structure.md` for details.

## Code Standards

### C++ Style

**Modern C++20**:
- Use C++20 features where appropriate
- Prefer STL over custom implementations
- Use smart pointers for ownership
- RAII for resource management

**Naming Conventions**:
- Classes/Structs: `PascalCase`
- Functions/Methods: `camelCase`
- Variables: `camelCase`
- Constants: `UPPER_SNAKE_CASE`
- Private members: trailing underscore `member_`

**Example**:
```cpp
namespace ReUseX::vision {

class DetectionBox {
public:
    DetectionBox(float x, float y, float width, float height);
    
    float getArea() const;
    bool contains(float x, float y) const;

private:
    float x_, y_, width_, height_;
};

} // namespace ReUseX::vision
```

**Formatting**:
- Use `.clang-format` configuration (if provided)
- 4 spaces for indentation (no tabs)
- 100 character line limit (soft guideline)
- Braces on same line for functions/classes

### Namespaces

- Maximum 3 levels: `ReUseX::module::component`
- Use `detail` namespace for internal utilities
- Avoid deeply nested namespaces

**Good**:
```cpp
namespace ReUseX::vision::object {
    class DetectionBox { /* ... */ };
}
```

**Avoid**:
```cpp
namespace ReUseX::vision::common::object::detection {  // Too deep!
    class Box { /* ... */ };
}
```

### Headers and Includes

**Include Guards**:
Use `#pragma once` (simpler, widely supported)

**Include Order**:
1. Corresponding header (for .cpp files)
2. C++ standard library
3. Third-party libraries
4. ReUseX headers

**Example**:
```cpp
#include <ReUseX/vision/object.hpp>  // Corresponding header

#include <vector>                     // C++ stdlib
#include <string>

#include <opencv2/core.hpp>           // Third-party

#include <ReUseX/io/Database.hpp>     // Other ReUseX headers
```

### Documentation

**Public API Documentation**:
Use Doxygen-style comments for all public APIs:

```cpp
/**
 * @brief Detects objects in an image using YOLO
 * 
 * @param image Input image (CV_8UC3 RGB)
 * @param confidenceThreshold Minimum confidence (0.0-1.0)
 * @return Vector of detected objects
 * 
 * @throws std::runtime_error if model not initialized
 */
std::vector<DetectionBox> detectObjects(
    const cv::Mat& image,
    float confidenceThreshold = 0.5f
);
```

**Implementation Comments**:
- Explain "why", not "what"
- Document non-obvious algorithms
- Reference papers/sources where applicable

### Error Handling

**Exceptions**:
- Use for unrecoverable errors (file not found, out of memory)
- Derive from `std::exception` or standard exception types
- Provide meaningful error messages

**Return Values**:
- Use for expected failures (node not found, empty result)
- Use `std::optional` for nullable returns
- Use `std::expected` (C++23) or custom Result type if available

**Example**:
```cpp
// Exception for unrecoverable error
RTABMapDatabase(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Database not found: " + path);
    }
}

// Return value for expected failure
std::optional<cv::Mat> getImage(int nodeId) {
    if (nodeExists(nodeId)) {
        return loadImage(nodeId);
    }
    return std::nullopt;
}
```

### Memory Management

- **Use smart pointers**: `unique_ptr`, `shared_ptr`
- **RAII**: Acquire resources in constructors, release in destructors
- **Avoid raw `new`/`delete`**: Use `make_unique`, `make_shared`
- **CUDA memory**: Use RAII wrappers, ensure cleanup

### Threading

- Document thread-safety requirements
- Use `std::mutex` for synchronization
- Prefer lock-free data structures where appropriate
- Avoid global mutable state

## Testing

**All new code must include tests** (see `tests/README.md`)

### Writing Tests

Place tests in appropriate module directory:
```
tests/unit/
├── core/         # Core module tests
├── geometry/     # Geometry tests
├── io/           # I/O tests
├── utils/        # Utility tests
├── vision/       # Vision tests
└── visualize/    # Visualization tests
```

**Test Template**:
```cpp
#include <catch2/catch_test_macros.hpp>
#include <ReUseX/module/component.hpp>

TEST_CASE("Component behaves correctly", "[module]") {
    SECTION("specific behavior") {
        // Arrange
        auto obj = ReUseX::module::Component();
        
        // Act
        auto result = obj.method();
        
        // Assert
        REQUIRE(result == expected);
    }
}
```

### Running Tests

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

### Coverage

Check coverage before submitting:
```bash
./tools/coverage/generate_coverage.sh
xdg-open build/coverage_html/index.html
```

**Coverage Requirements**:
- New code should be covered by tests
- Aim for 70%+ coverage on new modules
- Don't decrease overall project coverage

## Git Workflow

### Branching

- `main`: Stable release branch
- `dev`: Development branch (default)
- Feature branches: `feature/your-feature-name`
- Bug fixes: `fix/issue-description`

### Commits

**Commit Messages**:
Follow conventional commits:

```
type(scope): Short description

Longer description if needed. Explain why, not what.
Reference issues: Fixes #123
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `refactor`: Code refactoring
- `test`: Adding/updating tests
- `build`: Build system changes
- `ci`: CI/CD changes

**Example**:
```
feat(io): Add RTABMapDatabase class

Implements unified database access for RTABMap + segmentation labels.
Uses pimpl idiom to hide RTABMap dependencies from public headers.

Fixes #45
```

### Pull Requests

**Before submitting**:
1. ✅ All tests pass
2. ✅ Code follows style guidelines
3. ✅ New code includes tests
4. ✅ Documentation updated
5. ✅ No compiler warnings

**PR Title**: Same format as commit messages

**PR Description**:
- **What**: What does this PR do?
- **Why**: Why is this change needed?
- **How**: How was it implemented?
- **Testing**: How was it tested?

**Review Process**:
- Address review comments
- Keep PR focused (one feature/fix per PR)
- Rebase on dev before merging

## Documentation

### User Guides

Add user guides to `docs/guides/`:
- Getting started tutorials
- Usage examples
- Best practices

### Design Documentation

Document architectural decisions in `docs/design/`:
- Module overviews
- Design patterns
- API design rationale

### API Reference

- Update Doxygen comments for public APIs
- Build docs to verify: `cmake --build build --target doc`

## Subproject-Specific Guidelines

### libs/reusex (Core Library)

- Minimize dependencies
- Keep public headers clean
- Use pimpl for complex implementations
- Ensure ABI stability for releases

### apps/rux (CLI Tool)

- User-friendly error messages
- Progress indicators for long operations
- Validate user input
- Follow CLI11 conventions

### bindings/python (Future)

- Follow pybind11 best practices
- Provide Pythonic API
- Include type hints
- Add Python tests

## Performance Considerations

- Profile before optimizing
- Document performance requirements
- Use CUDA for GPU acceleration where appropriate
- Avoid premature optimization

## Security

- Validate all external input
- Avoid buffer overflows
- Use safe string operations
- Check return values from C APIs

## Questions?

- **General questions**: Open a discussion on GitHub
- **Bug reports**: Open an issue with the `bug` label
- **Feature requests**: Open an issue with the `enhancement` label
- **Architecture questions**: See `docs/design/` or open a discussion

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (GPL-3.0-or-later).

Thank you for contributing to ReUseX! 🚀
