import torch
from torch.utils.data import Dataset as TorchDataset
from .._core import Dataset as ReUseXDataset


class Dataset(TorchDataset):
    """This is implementation of the torch Dataset class"""

    def __init__(self, path: str):
        self.dataset = ReUseXDataset(path)

    def __len__(self):
        return len(self.dataset)


    def __getitem__(self, idx):
        data = self.dataset[idx] 
        data = dict(data)

        # Covert openCV image
        if Field.COLOR in data:
            data[Field.COLOR] = np.asarray(data[Field.COLOR].toImage()).copy()
        return (idx, data, )


class ImigePairDataset(TorchDataset):

    def __init__(self, dataset, pairs):
        self.dataset = dataset
        self.pairs = pairs

    def __len__(self):
        return len(self.pairs)

    def __getitem__(self, idx):

        data1 = self.dataset[self.pairs[idx,0]] 
        data1 = dict(data1)

        data2 = self.dataset[self.pairs[idx,1]] 
        data2 = dict(data2)

        # Covert openCV image
        for data in [data1, data2]:
            if Field.COLOR in data:
                data[Field.COLOR] = np.asarray(data[Field.COLOR].toImage()).copy()

        return (data1, data2)


