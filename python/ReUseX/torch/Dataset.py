# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

import torch
from torch.utils.data import Dataset as TorchDataset


from .._core import Dataset as ReUseXDataset, Field
from .._core import DataItem

from ..pose_graph.pose_graph import *


def make_pairs(data):
    """Create unique pairs from data excluding self-pairs and duplicates.
    
    Args:
        data: List of lists where each sublist contains target indices.
        
    Returns:
        Numpy array of unique pairs (source, target) with duplicates removed.
    """
    # Convert to an array where indices are repeated
    indices = np.arange(len(data))  # [0, 1, 2, 3]
    repeats = np.array([len(lst) for lst in data])  # [3, 2, 3, 2]
    source = np.repeat(indices, repeats)  # Expands indices: [0, 0, 0, 1, 1, 2, 2, 2, 3, 3]
    
    # Flatten the list of lists to get target indices
    target = np.concatenate(data)

    pairs = np.column_stack((source, target))

    # Mask where elements in both columns are equal
    mask = pairs[:, 0] != pairs[:, 1]

    # Remove self check 
    pairs =  pairs[mask]

    # Remove duplicats
    pairs = np.sort(pairs, axis=1)
    pairs = np.unique(pairs, axis=0)

    return pairs 


def make_pytorch_dict(data: DataItem):
    """Convert DataItem to PyTorch-compatible dictionary.
    
    Converts OpenCV images to numpy arrays for PyTorch processing.
    
    Args:
        data: DataItem to convert.
        
    Returns:
        Dictionary with converted data suitable for PyTorch.
    """
    data = dict(data)
    
    # Covert openCV image to numpy array
    if Field.COLOR in data:
        data[Field.COLOR] = np.asarray(data[Field.COLOR].toImage()).copy()

    return data


class Dataset(TorchDataset):
    """PyTorch Dataset wrapper for ReUseX datasets.
    
    This class adapts a ReUseX Dataset to the PyTorch Dataset interface.
    
    Attributes:
        dataset: The underlying ReUseX dataset.
    """

    def __init__(self, dataset: ReUseXDataset):
        """Initialize the Dataset wrapper.
        
        Args:
            dataset: ReUseX dataset to wrap.
        """
        self.dataset = dataset

    def __len__(self):
        """Return the number of items in the dataset.
        
        Returns:
            Number of items in the dataset.
        """
        return len(self.dataset)

    def __getitem__(self, idx):
        """Get item at specified index.
        
        Args:
            idx: Index of the item to retrieve.
            
        Returns:
            Data item at the specified index.
        """
        data = self.dataset[idx] 

        #if isinstance(data, tuple):
        #    data = tuple(make_pytorch_dict(e) for e in data)
        #else:
        #    data = make_pytorch_dict(data)

        return data



class ImigePairDataset(TorchDataset):
    """PyTorch Dataset for image pairs.
    
    This dataset returns pairs of images prepared for processing.
    
    Attributes:
        dataset: The underlying dataset containing images.
        pairs: Array of index pairs to retrieve.
    """
    
    def __init__(self, dataset, pairs):
        """Initialize the image pair dataset.
        
        Args:
            dataset: Source dataset containing images.
            pairs: Array of (idx1, idx2) pairs.
        """
        self.dataset = dataset
        self.pairs = pairs
 
    def __len__(self):
        """Return the number of pairs.
        
        Returns:
            Number of pairs in the dataset.
        """
        return len(self.pairs)

    def __getitem__(self, idx):
        """Get a pair of images at the specified index.
        
        Args:
            idx: Index of the pair to retrieve.
            
        Returns:
            Tuple of (data1, data2) dictionaries containing image data and metadata.
        """

        data1 = self.dataset[self.pairs[idx, 0]] 
        data1 = dict(data1)

        data2 = self.dataset[self.pairs[idx,1]] 
        data2 = dict(data2)

        # # Covert openCV image
        # for data in [data1, data2]:
        #     if Field.COLOR in data:
        #         data[Field.COLOR] = np.asarray(data[Field.COLOR].toImage()).copy()

        img1 = resize(data1[Field.COLOR].toImage(), 512)
        arr1 = ImgNorm(img1) #.permute(1, 2, 0)
        #print("Orig Shape", img1.size)
        true_shape1 = torch.from_numpy(np.int32(img1.size[::-1]))
        #print("True Shape", true_shape1)
        view1 = dict(img=arr1.to(device), true_shape=true_shape1,  idx=idx, instance=str(data1[Field.INDEX]))

        img2 = resize(data2[Field.COLOR].toImage(), 512)
        arr2 = ImgNorm(img2) #.permute(1, 2, 0)
        true_shape2 = torch.from_numpy(np.int32(img2.size[::-1]))
        view2 = dict(img=arr2.to(device), true_shape=true_shape2, idx=idx, instance=str(data2[Field.INDEX]))


        data1.update(view1)
        data2.update(view2)

        del data1[Field.COLOR]
        del data2[Field.COLOR]

        return (data1, data2)


class SpeckleDataset(TorchDataset):
    """PyTorch Dataset for Speckle data streams.
    
    This dataset reads data from a Speckle stream and provides it in a
    format suitable for PyTorch processing.
    
    Attributes:
        wrapper: Speckle stream wrapper.
        client: Speckle API client.
        data: Base data from the Speckle commit.
    """

    def __init__(self, url: str):
        """Initialize the Speckle dataset.
        
        Args:
            url: URL of the Speckle stream to load.
        """

        from specklepy.api.wrapper import StreamWrapper
        from specklepy.api import operations

        self.wrapper = StreamWrapper(url)
        self.client = self.wrapper.get_client()

        commit_objects = self.client.version.get(self.wrapper.commit_id, self.wrapper.stream_id)
        base = operations.receive(obj_id=commit_objects.referenced_object, remote_transport=self.wrapper.get_transport())
        self.data = base["Data"]["@{0}"]

    def __len__(self):
        """Return the number of items in the Speckle dataset.
        
        Returns:
            Number of items in the dataset.
        """
        return len(self.data)

    def __getitem__(self, idx: int):
        """Get item(s) from the Speckle dataset.
        
        Args:
            idx: Index or list of indices to retrieve.
            
        Returns:
            Dictionary or tuple of dictionaries containing data fields.
        """

        if isinstance(idx, list):
            R = tuple([{Field.INDEX:i} for i in idx])

            for d in R:
                obj = self.data[d[Field.INDEX]]
    
                if "pos" in obj.get_dynamic_member_names() and "quat" in obj.get_dynamic_member_names():
                    d[Field.ODOMETRY] =  np.array([[0,d[Field.INDEX], 
                                                    obj["pos"][0],
                                                    obj["pos"][1],
                                                    obj["pos"][2],
                                                    obj["quat"][0],
                                                    obj["quat"][1],
                                                    obj["quat"][2],
                                                    obj["quat"][3]
                                                    ]])
    


        else:
            obj = self.data[idx]

            R = {Field.INDEX:idx}

            if "pos" in obj.get_dynamic_member_names() and "quat" in obj.get_dynamic_member_names():
                R[Field.ODOMETRY] =  np.array([[0,idx, 
                                                obj["pos"][0],
                                                obj["pos"][1],
                                                obj["pos"][2],
                                                obj["quat"][0],
                                                obj["quat"][1],
                                                obj["quat"][2],
                                                obj["quat"][3]
                                                ]])
                R[Field.DEPTH] = np.array([])
                R[Field.COLOR] = np.array([])
                R[Field.CONFIDENCE] = np.array([])

        return R









