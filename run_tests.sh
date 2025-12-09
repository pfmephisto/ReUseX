#!/bin/bash
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Script to run all tests for ReUseX project

set -e  # Exit on error

echo "=================================="
echo "Running ReUseX Test Suite"
echo "=================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Track test results
PYTHON_PASSED=0
CPP_PASSED=0

# Run Python tests
echo "Running Python Tests..."
echo "--------------------------------"
if python3 -m pytest python/tests/ -v; then
    echo -e "${GREEN}✓ Python tests passed${NC}"
    PYTHON_PASSED=1
else
    echo -e "${RED}✗ Python tests failed${NC}"
fi
echo ""

# Run C++ tests if build directory exists
if [ -d "build" ] && [ -f "build/tests/reusex_tests" ]; then
    echo "Running C++ Tests..."
    echo "--------------------------------"
    if cd build && ctest --output-on-failure; then
        echo -e "${GREEN}✓ C++ tests passed${NC}"
        CPP_PASSED=1
        cd ..
    else
        echo -e "${RED}✗ C++ tests failed${NC}"
        cd ..
    fi
else
    echo "C++ tests not built. Run 'cmake -B build -DBUILD_TESTS=ON && cmake --build build' first."
fi
echo ""

# Summary
echo "=================================="
echo "Test Summary"
echo "=================================="
if [ $PYTHON_PASSED -eq 1 ]; then
    echo -e "Python: ${GREEN}PASSED${NC}"
else
    echo -e "Python: ${RED}FAILED${NC}"
fi

if [ $CPP_PASSED -eq 1 ]; then
    echo -e "C++:    ${GREEN}PASSED${NC}"
elif [ -d "build" ] && [ -f "build/tests/reusex_tests" ]; then
    echo -e "C++:    ${RED}FAILED${NC}"
else
    echo -e "C++:    ${NC}NOT BUILT${NC}"
fi
echo "=================================="

# Exit with error if any tests failed
if [ $PYTHON_PASSED -eq 0 ]; then
    exit 1
fi

if [ -d "build" ] && [ -f "build/tests/reusex_tests" ] && [ $CPP_PASSED -eq 0 ]; then
    exit 1
fi

exit 0
