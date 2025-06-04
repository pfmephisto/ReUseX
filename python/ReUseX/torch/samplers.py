from typing import Union

import numpy as np
from torch.utils.data import Sampler

# Custom sampler to select every 4th element
class StepSampler(Sampler):
    def __init__(self, data_source, step=4, shuffle=False, start:int=0, stop:Union[int|None]=None):

        stop = stop if stop else len(data_source)
        self.indices = list(range(start, stop, step))

        if shuffle:
            np.random.shuffle(self.indices)

    def __iter__(self):
        return iter(self.indices)

    def __len__(self):
        return len(self.indices)


class ConsecutiveBatchSampler(Sampler):
    def __init__(self, data_source, step):
        self.data_source = data_source
        self.step = step

    def __iter__(self):
        return iter([[i - self.step, i] for i in range(self.step, len(self.data_source), self.step)])

    def __len__(self):
        return (len(self.data_source) - self.step) // self.step
