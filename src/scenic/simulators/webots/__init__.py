"""Scenic world models for the Webots robotics simulator.

This module contains common code for working with Webots, e.g. parsing WBT files,
as well as a generic dynamic simulator interface and world model for Webots.
More detailed world models for particular types of scenarios are in submodules.
"""

from .simulator import WebotsSimulator
from .utils import ENU, EUN, NUE
