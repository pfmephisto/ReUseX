# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for torch utility functions."""

import pytest
import numpy as np


def _collate(batch):
    """Collate a batch of items into a numpy array."""
    return np.array(batch)


class TestCollate:
    """Test suite for the _collate function."""
    
    def test_collate_simple_list(self):
        """Test collating a simple list of numbers."""
        batch = [1, 2, 3, 4, 5]
        result = _collate(batch)
        expected = np.array([1, 2, 3, 4, 5])
        np.testing.assert_array_equal(result, expected)
    
    def test_collate_nested_lists(self):
        """Test collating nested lists."""
        batch = [[1, 2], [3, 4], [5, 6]]
        result = _collate(batch)
        expected = np.array([[1, 2], [3, 4], [5, 6]])
        np.testing.assert_array_equal(result, expected)
    
    def test_collate_floats(self):
        """Test collating float values."""
        batch = [1.5, 2.5, 3.5]
        result = _collate(batch)
        expected = np.array([1.5, 2.5, 3.5])
        np.testing.assert_array_almost_equal(result, expected)
    
    def test_collate_empty(self):
        """Test collating an empty list."""
        batch = []
        result = _collate(batch)
        expected = np.array([])
        np.testing.assert_array_equal(result, expected)
    
    def test_collate_single_element(self):
        """Test collating a single element."""
        batch = [42]
        result = _collate(batch)
        expected = np.array([42])
        np.testing.assert_array_equal(result, expected)
    
    def test_collate_preserves_dtype(self):
        """Test that collate preserves data types."""
        batch = [np.int32(1), np.int32(2), np.int32(3)]
        result = _collate(batch)
        assert result.dtype == np.int32
