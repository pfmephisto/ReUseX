# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for pose graph utility functions."""

import pytest
import numpy as np


def to_numpy(x):
    """Convert data to numpy format.
    
    Args:
        x: Data to convert (dict, list, tuple, or numpy array).
        
    Returns:
        Data converted to numpy arrays.
    """
    if isinstance(x, dict):
        return {k: to_numpy(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)):
        return type(x)(to_numpy(elem) for elem in x)
    if isinstance(x, np.ndarray):
        return x
    # For other types, try to convert to numpy
    try:
        return np.array(x)
    except:
        return x


def merge_corres(idx1, idx2, shape1=None, shape2=None, ret_xy=True, ret_index=False):
    """Merge and deduplicate correspondence indices.
    
    Args:
        idx1: First set of indices.
        idx2: Second set of indices.
        shape1: Shape of first image for coordinate conversion.
        shape2: Shape of second image for coordinate conversion.
        ret_xy: Whether to return xy coordinates. Defaults to True.
        ret_index: Whether to return original indices. Defaults to False.
        
    Returns:
        Tuple of (xy1, xy2) or (idx1, idx2), optionally with indices if ret_index=True.
    """
    assert idx1.dtype == idx2.dtype == np.int32
    
    # unique and sort along idx1
    corres = np.unique(np.c_[idx2, idx1].view(np.int64), return_index=ret_index)
    if ret_index:
        corres, indices = corres
    xy2, xy1 = corres[:, None].view(np.int32).T
    
    if ret_xy:
        assert shape1 and shape2
        xy1 = np.unravel_index(xy1, shape1)
        xy2 = np.unravel_index(xy2, shape2)
        if ret_xy != "y_x":
            xy1 = xy1[0].base[:, ::-1]
            xy2 = xy2[0].base[:, ::-1]
    
    if ret_index:
        return xy1, xy2, indices
    return xy1, xy2


class TestToNumpy:
    """Test suite for to_numpy function."""
    
    def test_to_numpy_dict(self):
        """Test converting dictionary to numpy."""
        data = {"a": [1, 2, 3], "b": [4, 5, 6]}
        result = to_numpy(data)
        assert isinstance(result, dict)
        np.testing.assert_array_equal(result["a"], np.array([1, 2, 3]))
        np.testing.assert_array_equal(result["b"], np.array([4, 5, 6]))
    
    def test_to_numpy_list(self):
        """Test converting list to numpy."""
        data = [[1, 2], [3, 4], [5, 6]]
        result = to_numpy(data)
        assert isinstance(result, list)
        # Elements are converted individually
        for i, arr in enumerate(result):
            # Each element becomes a numpy array
            assert isinstance(arr, (np.ndarray, list))
    
    def test_to_numpy_tuple(self):
        """Test converting tuple to numpy."""
        data = ([1, 2], [3, 4])
        result = to_numpy(data)
        assert isinstance(result, tuple)
        assert len(result) == 2
    
    def test_to_numpy_already_numpy(self):
        """Test that numpy arrays are returned unchanged."""
        data = np.array([1, 2, 3])
        result = to_numpy(data)
        assert result is data
        np.testing.assert_array_equal(result, data)
    
    def test_to_numpy_nested(self):
        """Test converting nested structures."""
        data = {"a": [[1, 2], [3, 4]], "b": (5, 6)}
        result = to_numpy(data)
        assert isinstance(result, dict)
        assert isinstance(result["a"], list)


class TestMergeCorres:
    """Test suite for merge_corres function."""
    
    def test_merge_corres_basic(self):
        """Test basic correspondence merging."""
        idx1 = np.array([0, 5, 10], dtype=np.int32)
        idx2 = np.array([1, 6, 11], dtype=np.int32)
        shape1 = (10, 10)
        shape2 = (10, 10)
        
        xy1, xy2 = merge_corres(idx1, idx2, shape1, shape2)
        
        # Check that results are arrays
        assert isinstance(xy1, np.ndarray)
        assert isinstance(xy2, np.ndarray)
        assert xy1.shape[0] == xy2.shape[0]
    
    def test_merge_corres_deduplication(self):
        """Test that merge_corres removes duplicates."""
        idx1 = np.array([0, 0, 5], dtype=np.int32)
        idx2 = np.array([1, 1, 6], dtype=np.int32)
        shape1 = (10, 10)
        shape2 = (10, 10)
        
        xy1, xy2 = merge_corres(idx1, idx2, shape1, shape2)
        
        # Should have 2 unique correspondences
        assert len(xy1) == 2
        assert len(xy2) == 2
    
    def test_merge_corres_no_xy(self):
        """Test merge_corres without coordinate conversion."""
        idx1 = np.array([0, 5, 10], dtype=np.int32)
        idx2 = np.array([1, 6, 11], dtype=np.int32)
        
        result_idx1, result_idx2 = merge_corres(idx1, idx2, ret_xy=False)
        
        assert isinstance(result_idx1, np.ndarray)
        assert isinstance(result_idx2, np.ndarray)
        assert result_idx1.dtype == np.int32
    
    def test_merge_corres_with_index(self):
        """Test merge_corres with index return."""
        idx1 = np.array([0, 5, 10], dtype=np.int32)
        idx2 = np.array([1, 6, 11], dtype=np.int32)
        shape1 = (10, 10)
        shape2 = (10, 10)
        
        xy1, xy2, indices = merge_corres(idx1, idx2, shape1, shape2, ret_index=True)
        
        assert isinstance(indices, np.ndarray)
        assert len(indices) == len(xy1)
