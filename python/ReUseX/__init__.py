from __future__ import annotations

import logging
from os import getpid

# # Define the logger format
# logging.basicConfig(level=logging.INFO, format=log_format)

#only enable logging for this module
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Disable propagation to prevent duplicate messages from root handlers
logger.propagate = False

console_logger = logging.StreamHandler()
console_logger.setLevel(logging.DEBUG)
console_logger.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger.addHandler(console_logger)

# Ensure no duplicate handlers are added
if not logger.hasHandlers():
    logger.addHandler(console_logger)

logger.debug("Thread ID: %s", getpid())
logger.debug("Loading ReUseX")


from ._core import *
from ._core import __doc__, __version__


from .cli import run, ml

__all__ = ["__doc__", "__version__", "export", "run"]

logger.info("ReUseX loaded")

