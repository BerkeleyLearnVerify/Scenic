Scenic Internals
================

This section of the documentation describes the implementation of Scenic.
It is not intended for ordinary users of Scenic, and will probably only be useful for people who need to make some change to the language (e.g. adding a new type of distribution).

The documentation is organized by the submodules of the main ``scenic`` module:

.. autosummary::
   :toctree: _autosummary

   scenic.core
   scenic.simulators
   scenic.syntax

The ``scenic`` module itself provides two functions as the top-level interface to Scenic:

.. autofunction:: scenic.scenarioFromFile

.. autofunction:: scenic.scenarioFromString
