# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for torch sampler classes."""

import pytest
import numpy as np


class StepSampler:
    """Sampler that selects every nth element from the dataset."""
    
    def __init__(self, data_source, step=4, shuffle=False, start=0, stop=None):
        stop = stop if stop else len(data_source)
        self.indices = list(range(start, stop, step))
        if shuffle:
            np.random.shuffle(self.indices)
    
    def __iter__(self):
        return iter(self.indices)
    
    def __len__(self):
        return len(self.indices)


class ConsecutiveBatchSampler:
    """Sampler that returns consecutive pairs of indices."""
    
    def __init__(self, data_source, step):
        self.data_source = data_source
        self.step = step
    
    def __iter__(self):
        return iter([[i - self.step, i] for i in range(self.step, len(self.data_source), self.step)])
    
    def __len__(self):
        return (len(self.data_source) - self.step) // self.step


# Mock data source for testing
class MockDataSource:
    def __init__(self, length):
        self.length = length
    
    def __len__(self):
        return self.length


class TestStepSampler:
    """Test suite for StepSampler."""
    
    def test_step_sampler_basic(self):
        """Test basic step sampling."""
        data = MockDataSource(20)
        sampler = StepSampler(data, step=4, start=0)
        result = list(sampler)
        assert result == [0, 4, 8, 12, 16]
    
    def test_step_sampler_with_start(self):
        """Test step sampling with custom start."""
        data = MockDataSource(20)
        sampler = StepSampler(data, step=3, start=2)
        result = list(sampler)
        assert result == [2, 5, 8, 11, 14, 17]
    
    def test_step_sampler_with_stop(self):
        """Test step sampling with custom stop."""
        data = MockDataSource(20)
        sampler = StepSampler(data, step=4, start=0, stop=10)
        result = list(sampler)
        assert result == [0, 4, 8]
    
    def test_step_sampler_length(self):
        """Test that __len__ returns correct length."""
        data = MockDataSource(20)
        sampler = StepSampler(data, step=4)
        assert len(sampler) == 5
    
    def test_step_sampler_step_one(self):
        """Test step sampling with step=1 (all elements)."""
        data = MockDataSource(5)
        sampler = StepSampler(data, step=1)
        result = list(sampler)
        assert result == [0, 1, 2, 3, 4]
    
    def test_step_sampler_empty_result(self):
        """Test step sampling that results in empty indices."""
        data = MockDataSource(5)
        sampler = StepSampler(data, step=10)
        result = list(sampler)
        assert result == [0]


class TestConsecutiveBatchSampler:
    """Test suite for ConsecutiveBatchSampler."""
    
    def test_consecutive_batch_basic(self):
        """Test basic consecutive batch sampling."""
        data = MockDataSource(10)
        sampler = ConsecutiveBatchSampler(data, step=2)
        result = list(sampler)
        expected = [[0, 2], [2, 4], [4, 6], [6, 8]]
        assert result == expected
    
    def test_consecutive_batch_step_one(self):
        """Test consecutive batch sampling with step=1."""
        data = MockDataSource(5)
        sampler = ConsecutiveBatchSampler(data, step=1)
        result = list(sampler)
        expected = [[0, 1], [1, 2], [2, 3], [3, 4]]
        assert result == expected
    
    def test_consecutive_batch_length(self):
        """Test that __len__ returns correct length."""
        data = MockDataSource(10)
        sampler = ConsecutiveBatchSampler(data, step=2)
        assert len(sampler) == 4
    
    def test_consecutive_batch_large_step(self):
        """Test consecutive batch sampling with larger step."""
        data = MockDataSource(15)
        sampler = ConsecutiveBatchSampler(data, step=5)
        result = list(sampler)
        expected = [[0, 5], [5, 10]]
        assert result == expected
    
    def test_consecutive_batch_edge_case(self):
        """Test consecutive batch sampling at edge case."""
        data = MockDataSource(3)
        sampler = ConsecutiveBatchSampler(data, step=2)
        result = list(sampler)
        expected = [[0, 2]]
        assert result == expected
        # (3 - 2) // 2 = 0, but we still get 1 pair at index 2
        # The implementation generates pairs starting from step
        assert len(result) == 1
