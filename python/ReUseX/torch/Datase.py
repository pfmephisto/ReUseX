import torch
from torch.utils.data import Dataset as TorchDataset
from .._core import Dataset as ReUseXDataset

class Dataset(TorchDataset):
    def __init__(self, path: str):
        self.dataset = ReUseXDataset(path)

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):
        return self.dataset[idx]