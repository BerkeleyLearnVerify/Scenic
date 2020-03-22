Welcome to Scenic's documentation!
==================================

Scenic is a domain-specific probabilistic programming language for modeling the environments of cyber-physical systems like robots and autonomous cars.
A Scenic program defines a distribution over *scenes*, configurations of physical objects and agents; sampling from this distribution yields concrete scenes which can be simulated to produce training or testing data.

Scenic was designed and implemented by Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.
For a description of the language and some of its applications, see `our PLDI 2019 paper <https://arxiv.org/abs/1809.09310>`_; a more in-depth discussion is in Chapters 5 and 8 of `this thesis <https://people.ucsc.edu/~dfremont/papers/thesis.pdf>`_.

If you have any problems using Scenic, please contact Daniel at dfremont@ucsc.edu or submit an issue to `our GitHub repository <https://github.com/BerkeleyLearnVerify/Scenic>`_.

Table of Contents
=================

.. toctree::
   :maxdepth: 1

   quickstart
   tutorial
   simulators
   new_simulator
   internals

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

License
=======

Scenic is distributed under the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`_.
However, it currently uses the `GPC library`_ (through the `Polygon3 package`_), which is free only for non-commercial use.
GPC is used only in the `scenic.core.geometry.triangulatePolygon` function, and you can alternatively plug in any algorithm of your choice for triangulation of polygons with holes.
We plan to replace GPC with a BSD-compatible library in the near future.

.. _GPC library: http://www.cs.man.ac.uk/~toby/alan/software/

.. _Polygon3 package: https://pypi.org/project/Polygon3/
