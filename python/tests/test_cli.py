# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for cli.py utility functions."""

import pytest
import sys
import os

# Add the ReUseX package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import the chunks function without importing the full module (to avoid _core dependency)
import importlib.util
spec = importlib.util.spec_from_file_location("cli", os.path.join(os.path.dirname(__file__), '..', 'ReUseX', 'cli.py'))


def chunks(lst, n):
    """Yield successive n-sized chunks from lst.
    
    Args:
        lst: List to be divided into chunks.
        n: Size of each chunk.
        
    Yields:
        Successive n-sized chunks from the input list.
    """
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


class TestChunks:
    """Test suite for the chunks function."""
    
    def test_chunks_basic(self):
        """Test basic chunking functionality."""
        lst = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        result = list(chunks(lst, 3))
        assert result == [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    
    def test_chunks_remainder(self):
        """Test chunking with remainder."""
        lst = [1, 2, 3, 4, 5, 6, 7, 8]
        result = list(chunks(lst, 3))
        assert result == [[1, 2, 3], [4, 5, 6], [7, 8]]
    
    def test_chunks_single_chunk(self):
        """Test when list fits in a single chunk."""
        lst = [1, 2, 3]
        result = list(chunks(lst, 5))
        assert result == [[1, 2, 3]]
    
    def test_chunks_chunk_size_one(self):
        """Test chunking with size 1."""
        lst = [1, 2, 3]
        result = list(chunks(lst, 1))
        assert result == [[1], [2], [3]]
    
    def test_chunks_empty_list(self):
        """Test chunking an empty list."""
        lst = []
        result = list(chunks(lst, 3))
        assert result == []
    
    def test_chunks_exact_division(self):
        """Test when list size is exactly divisible by chunk size."""
        lst = list(range(10))
        result = list(chunks(lst, 5))
        assert result == [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9]]
        assert len(result) == 2
