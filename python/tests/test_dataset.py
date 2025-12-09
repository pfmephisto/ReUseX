# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for Dataset utility functions."""

import pytest
import numpy as np


def make_pairs(data):
    """Create unique pairs from data excluding self-pairs and duplicates.
    
    Args:
        data: List of lists where each sublist contains target indices.
        
    Returns:
        Numpy array of unique pairs (source, target) with duplicates removed.
    """
    # Convert to an array where indices are repeated
    indices = np.arange(len(data))
    repeats = np.array([len(lst) for lst in data])
    source = np.repeat(indices, repeats)
    
    # Flatten the list of lists to get target indices
    target = np.concatenate(data)
    
    pairs = np.column_stack((source, target))
    
    # Mask where elements in both columns are equal
    mask = pairs[:, 0] != pairs[:, 1]
    
    # Remove self check 
    pairs = pairs[mask]
    
    # Remove duplicates
    pairs = np.sort(pairs, axis=1)
    pairs = np.unique(pairs, axis=0)
    
    return pairs


class TestMakePairs:
    """Test suite for the make_pairs function."""
    
    def test_make_pairs_basic(self):
        """Test basic pair creation."""
        data = [[1, 2], [0, 2], [0, 1]]
        result = make_pairs(data)
        # Expected pairs after removing self-pairs and sorting: (0,1), (0,2), (1,2)
        expected = np.array([[0, 1], [0, 2], [1, 2]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_with_self_reference(self):
        """Test that self-references are removed."""
        data = [[0, 1], [1], [0, 2]]
        result = make_pairs(data)
        # Should remove (0,0) and (1,1)
        expected = np.array([[0, 1], [0, 2]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_with_duplicates(self):
        """Test that duplicate pairs are removed."""
        data = [[1, 2], [0, 2], [1]]  # (0,1), (0,2), (1,2), (2,1) -> after dedup: (0,1), (0,2), (1,2)
        result = make_pairs(data)
        expected = np.array([[0, 1], [0, 2], [1, 2]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_single_pair(self):
        """Test with data that produces a single pair."""
        data = [[1], []]
        result = make_pairs(data)
        expected = np.array([[0, 1]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_empty_sublists(self):
        """Test with some empty sublists."""
        data = [[], [2], [1]]
        result = make_pairs(data)
        expected = np.array([[1, 2]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_all_empty(self):
        """Test with all empty sublists."""
        data = [[], [], []]
        result = make_pairs(data)
        assert result.shape == (0, 2)
    
    def test_make_pairs_complex(self):
        """Test with more complex data."""
        data = [[1, 2, 3], [0, 2], [0, 1], [0]]
        result = make_pairs(data)
        # Expected unique pairs: (0,1), (0,2), (0,3), (1,2)
        expected = np.array([[0, 1], [0, 2], [0, 3], [1, 2]])
        np.testing.assert_array_equal(result, expected)
    
    def test_make_pairs_sorted_output(self):
        """Test that output pairs are sorted."""
        data = [[2, 1], [2]]
        result = make_pairs(data)
        # All pairs should have smaller index first
        for pair in result:
            assert pair[0] < pair[1]
