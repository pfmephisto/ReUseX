import torch
from torch.utils.data import Dataset as TorchDataset


from .._core import Dataset as ReUseXDataset, Field
from .._core import DataItem

from ..pose_graph.pose_graph import *


def make_pairs(data):
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
    data = dict(data)
    
    # Covert openCV image to numpy array
    if Field.COLOR in data:
        data[Field.COLOR] = np.asarray(data[Field.COLOR].toImage()).copy()

    return data


class Dataset(TorchDataset):
    """This is implementation of the torch Dataset class"""

    def __init__(self, dataset: ReUseXDataset):
        self.dataset = dataset

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, idx):
        data = self.dataset[idx] 

        if isinstance(data, tuple):
            data = tuple(make_pytorch_dict(e) for e in data)
        else:
            data = make_pytorch_dict(data)

        return data



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

    def __init__(self, url: str):

        from specklepy.api.wrapper import StreamWrapper
        from specklepy.api import operations

        self.wrapper = StreamWrapper(url)
        self.client =  self.wrapper.get_client()

        commit_objects = self.client.version.get(self.wrapper.commit_id, self.wrapper.stream_id)
        base = operations.receive(obj_id=commit_objects.referencedObject, remote_transport=self.wrapper.get_transport())
        self.data = base["Data"]["@{0}"]

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx: int):

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

        return R









