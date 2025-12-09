# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

from typing import List, Union, Dict, Tuple

from .._core import DataItem
import logging

import numpy as np
from  numpy.typing import NDArray

logger = logging.getLogger(__name__)

def _collate(batch: List) -> NDArray:
    """Collate a batch of items into a numpy array.
    
    Args:
        batch: List of items to collate.
        
    Returns:
        Numpy array containing the batch data.
    """
    return np.array(batch)


def numpy_collate_fn(batch) -> Union[Dict, Tuple, NDArray, DataItem]:
    """Custom collate function for numpy arrays and DataItems.
    
    This function handles batching of various data types including DataItem,
    tuples, dictionaries, and numpy arrays.
    
    Args:
        batch: Batch of data to collate.
        
    Returns:
        Collated data in appropriate format (Dict, Tuple, NDArray, or DataItem).
    """
    elem = batch[0]
    #logger.debug("Element of type: %s", type(elem))
    #logger.debug("batch len() => %i", len(batch))

    if isinstance(elem, DataItem) and len(batch) == 1:
        #logger.debug("Returning DataItem")
        return elem

    if isinstance(elem, DataItem):
        return {key: _collate([ item[key] for item  in batch]) for key in elem }


    elif isinstance(elem, tuple):
        #logger.debug("Returning Tuple")
        return numpy_collate_fn([v1 for v1,_ in batch]), numpy_collate_fn([v2 for _,v2 in batch])

    elif isinstance(elem, dict):
        #logger.debug("Returning dict")
        return {k: _collate([elem[k] for elem in batch]) for k in elem }

    logger.debug("Error no match for numpy_collate_fn")
