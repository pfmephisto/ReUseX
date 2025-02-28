from __future__ import annotations

#import logging
from os import getpid

import spdlog
logger = spdlog.ConsoleLogger("ReUseX")
logger.set_level(spdlog.LogLevel.DEBUG)

from ._core import *
from ._core import __doc__, __version__

_core.register_logger(logger.get_underlying_logger())
#logger.set_pattern("[%l] %n: %v")
# register logger with the function from mylib, so it is accessible inside MyClass


# # Define the logger format
# logging.basicConfig(level=logging.INFO, format=log_format)

##only enable logging for this module
#logger = logging.getLogger(__name__)
#logger.setLevel(logging.DEBUG)
#
## Disable propagation to prevent duplicate messages from root handlers
#logger.propagate = False
#
#console_logger = logging.StreamHandler()
#console_logger.setLevel(logging.DEBUG)
#console_logger.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
#logger.addHandler(console_logger)
#
## Ensure no duplicate handlers are added
#if not logger.hasHandlers():
#    logger.addHandler(console_logger)


logger.debug("Thread ID: %s", getpid())
logger.debug("Loading ReUseX")


from .cli import run, ml

__all__ = ["__doc__", "__version__", "export", "run"]

logger.info("ReUseX loaded")

