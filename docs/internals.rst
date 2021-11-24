..  _internals:

Scenic Internals
================

This section of the documentation describes the implementation of Scenic.
Much of this information will probably only be useful for people who need to make some change to the language (e.g. adding a new type of distribution).
However, the detailed documentation on Scenic's abstract application domains (in `scenic.domains`) and simulator interfaces (in `scenic.simulators`) may be of interest to people using those features.

The documentation is organized by the submodules of the main ``scenic`` module:

.. autosummary::
   :toctree: modules

   scenic.core
   scenic.domains
   scenic.formats
   scenic.simulators
   scenic.syntax

The ``scenic`` module itself provides the top-level API for using Scenic: see :doc:`api`.
