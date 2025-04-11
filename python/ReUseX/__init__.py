from __future__ import annotations

import sys
from os import getpid
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--log-level", choices=["TRACE" "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], default="DEBUG")
args = parser.parse_args()

import spdlog

logger = spdlog.ConsoleLogger(__name__)
logger.set_level(getattr(spdlog.LogLevel, args.log_level))
logger.set_level(spdlog.LogLevel.TRACE)
logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%=7l%$] %v")

from ._core import *
from ._core import __doc__, __version__

_core.register_logger(logger.get_underlying_logger())
_core.set_default_logger(logger.get_underlying_logger())

logger.info("ReUseX loaded")
logger.debug(f"Thread ID: {getpid()}")


if "torch" in sys.modules:
    device = 'cuda' if sys.modules["torch"].cuda.is_available() else 'cpu'
    if sys.modules["torch"].cuda.is_available():
        device = sys.modules['torch'].cuda.device('cuda')
        device_name = sys.modules['torch'].cuda.get_device_name(device)
        logger.debug(f"Cuda device avaliable: {device_name}")
    else:
        logger.warn("Cuda not avaliable")
else:
    logger.debug("pyTorch has not been imported")



from .cli import run, ml

__all__ = ["__doc__", "__version__", "export", "run"]
