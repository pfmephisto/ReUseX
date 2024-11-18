from __future__ import annotations

import logging



from ._core import *
from ._core import __doc__, __version__

# # Define the logger format
# logging.basicConfig(level=logging.INFO, format=log_format)

#only enable logging for this module
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
console_logger = logging.StreamHandler()
console_logger.setLevel(logging.INFO)
console_logger.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger.addHandler(console_logger)




from .cli import run

__all__ = ["__doc__", "__version__", "speckle", "export", "run"]

logger.info("LinkML-Py loaded")

