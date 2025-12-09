#!/bin/bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Script to run C++ tests for ReUseX project

set -e  # Exit on error

echo "=================================="
echo "Running ReUseX C++ Test Suite"
echo "=================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Run C++ tests if build directory exists
if [ -d "build" ] && [ -f "build/tests/reusex_tests" ]; then
    echo "Running C++ Tests..."
    echo "--------------------------------"
    if cd build && ctest --output-on-failure; then
        echo ""
        echo -e "${GREEN}✓ C++ tests passed${NC}"
        cd ..
        echo "=================================="
        exit 0
    else
        echo ""
        echo -e "${RED}✗ C++ tests failed${NC}"
        cd ..
        echo "=================================="
        exit 1
    fi
else
    echo "C++ tests not built."
    echo ""
    echo "To build and run tests:"
    echo "  cmake -B build -DBUILD_TESTS=ON"
    echo "  cmake --build build"
    echo "  cd build && ctest --output-on-failure"
    echo "=================================="
    exit 1
fi
