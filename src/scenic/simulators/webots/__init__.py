"""Scenic world models for the Webots robotics simulator.

This module contains common code for working with Webots, e.g. parsing WBT files.
World models for particular uses of Webots are in submodules.

Submodules
==========

.. autosummary::
   :toctree: _autosummary

   mars
   road
   common
   world_parser
"""

from .common import scenicToWebotsPosition, scenicToWebotsRotation
