# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

from typing import Union

import numpy as np
from torch.utils.data import Sampler

# Custom sampler to select every nth element
class StepSampler(Sampler):
    """Sampler that selects every nth element from the dataset.
    
    This sampler allows selecting elements at regular intervals (steps)
    from the dataset, optionally with shuffling.
    
    Attributes:
        indices: List of indices to sample.
    """
    
    def __init__(self, data_source, step=4, shuffle=False, start: int = 0, stop: Union[int | None] = None):
        """Initialize the StepSampler.
        
        Args:
            data_source: The dataset to sample from.
            step: Interval between samples. Defaults to 4.
            shuffle: Whether to shuffle the indices. Defaults to False.
            start: Starting index. Defaults to 0.
            stop: Ending index. If None, uses length of data_source.
        """

        stop = stop if stop else len(data_source)
        self.indices = list(range(start, stop, step))

        if shuffle:
            np.random.shuffle(self.indices)

    def __iter__(self):
        """Return an iterator over the indices.
        
        Returns:
            Iterator over sampled indices.
        """
        return iter(self.indices)

    def __len__(self):
        """Return the number of samples.
        
        Returns:
            Number of indices to sample.
        """
        return len(self.indices)


class ConsecutiveBatchSampler(Sampler):
    """Sampler that returns consecutive pairs of indices.
    
    This sampler creates batches of consecutive element pairs separated
    by a specified step size.
    
    Attributes:
        data_source: The dataset to sample from.
        step: Step size between consecutive elements.
    """
    
    def __init__(self, data_source, step):
        """Initialize the ConsecutiveBatchSampler.
        
        Args:
            data_source: The dataset to sample from.
            step: Step size between elements in each pair.
        """
        self.data_source = data_source
        self.step = step

    def __iter__(self):
        """Return an iterator over consecutive index pairs.
        
        Returns:
            Iterator yielding [i - step, i] pairs.
        """
        return iter([[i - self.step, i] for i in range(self.step, len(self.data_source), self.step)])

    def __len__(self):
        """Return the number of pairs.
        
        Returns:
            Number of consecutive pairs.
        """
        return (len(self.data_source) - self.step) // self.step
