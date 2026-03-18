#!/usr/bin/env bash

# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Code Coverage Report Generator
# ===============================================
# Generates HTML coverage reports using lcov/gcov
# Requires: lcov, gcov (install via: apt install lcov)
# ===============================================

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ReUseX Coverage Report Generator${NC}"
echo -e "${GREEN}========================================${NC}"

# Check for lcov
if ! command -v lcov &> /dev/null; then
    echo -e "${RED}Error: lcov not found${NC}"
    echo "Install with: sudo apt install lcov"
    exit 1
fi

# Project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
COVERAGE_DIR="$BUILD_DIR/coverage"
HTML_DIR="$BUILD_DIR/coverage_html"

cd "$PROJECT_ROOT"

# Clean previous build
echo -e "${YELLOW}Cleaning previous build...${NC}"
rm -rf "$BUILD_DIR"

# Configure with coverage
echo -e "${YELLOW}Configuring with coverage enabled...${NC}"
cmake -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DENABLE_COVERAGE=ON \
    -DBUILD_TESTS=ON \
    -DBUILD_VISUALIZATION=OFF

# Build
echo -e "${YELLOW}Building...${NC}"
cmake --build "$BUILD_DIR" -j$(nproc)

# Run tests
echo -e "${YELLOW}Running tests...${NC}"
cd "$BUILD_DIR"
ctest --output-on-failure

# Create coverage directory
mkdir -p "$COVERAGE_DIR"
cd "$COVERAGE_DIR"

# Capture coverage data
echo -e "${YELLOW}Capturing coverage data...${NC}"
lcov --capture \
    --directory "$BUILD_DIR" \
    --output-file coverage.info \
    --rc lcov_branch_coverage=1

# Filter out system headers and test code
echo -e "${YELLOW}Filtering coverage data...${NC}"
lcov --remove coverage.info \
    '/usr/*' \
    '*/tests/*' \
    '*/catch2/*' \
    '*/vcpkg_installed/*' \
    '*/build/*' \
    --output-file coverage_filtered.info \
    --rc lcov_branch_coverage=1

# Generate HTML report
echo -e "${YELLOW}Generating HTML report...${NC}"
genhtml coverage_filtered.info \
    --output-directory "$HTML_DIR" \
    --title "ReUseX Coverage Report" \
    --legend \
    --show-details \
    --rc lcov_branch_coverage=1

# Summary
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Coverage Report Generated${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Report location: ${YELLOW}$HTML_DIR/index.html${NC}"
echo ""

# Extract summary
lcov --summary coverage_filtered.info --rc lcov_branch_coverage=1

echo ""
echo -e "Open report with: ${YELLOW}xdg-open $HTML_DIR/index.html${NC}"
echo -e "${GREEN}========================================${NC}"
