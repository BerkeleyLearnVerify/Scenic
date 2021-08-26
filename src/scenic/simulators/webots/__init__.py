"""Scenic world models for the Webots robotics simulator.

This module contains common code for working with Webots, e.g. parsing WBT files,
as well as a generic dynamic simulator interface and world model for Webots.
More detailed world models for particular types of scenarios are in submodules.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   simulator
   model
   actions
   mars
   road
   guideways
   utils
   world_parser
"""

from .utils import scenicToWebotsPosition, scenicToWebotsRotation
from .simulator import WebotsSimulator
