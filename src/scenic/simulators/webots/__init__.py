"""Scenic world models for the Webots robotics simulator.

This module contains common code for working with Webots, e.g. parsing WBT files.
World models for particular uses of Webots are in submodules.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   mars
   road
   guideways
   common
   world_parser
"""

from .common import scenicToWebotsPosition, scenicToWebotsRotation
