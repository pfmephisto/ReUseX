from typing import List

import os

from .._core import Brep, PointCloud, extract_instances
from .brep import BrepToBase
from .pointcloud import PointCloudtoBase

from specklepy.objects.other import Collection


def export(cloud: PointCloud, breps: List[Brep], name: str = "Export") -> Collection:

    data = extract_instances(cloud)

    collection_elements = Collection(
        name ="Elements",
        collectionType = "Collection",
        elements =  [Collection( name = key, collectionType = "PointCloud", elements=[PointCloudtoBase(c) 
            for c in val] ) for key, val in data.items() if key != "person" ] )
    
    collection_background = Collection(
        name = "Background",
        collectionType = "PointCloud",
        elements = [PointCloudtoBase(c) for c in data["person"]]
    )
    
    collection_volumes = Collection(
        name = "Volumes",
        collectionType = "Brep",
        elements = [ BrepToBase(brep) for brep in breps]
    )

    
    return Collection(
        name = name,
        collectionType = "Collection",
        elements = [collection_elements, collection_volumes, collection_background]
    )