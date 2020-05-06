Welcome to Scenic's documentation!
==================================

Scenic is a domain-specific probabilistic programming language for modeling the environments of cyber-physical systems like robots and autonomous cars.
A Scenic program defines a distribution over *scenes*, configurations of physical objects and agents; sampling from this distribution yields concrete scenes which can be simulated to produce training or testing data.

Scenic was designed and implemented by Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.
For a description of the language and some of its applications, see `our PLDI 2019 paper <https://arxiv.org/abs/1809.09310>`_; a more in-depth discussion is in Chapters 5 and 8 of `this thesis <https://people.ucsc.edu/~dfremont/papers/thesis.pdf>`_.

If you have any problems using Scenic, please submit an issue to `our GitHub repository <https://github.com/BerkeleyLearnVerify/Scenic>`_ or contact Daniel at dfremont@ucsc.edu.

Table of Contents
=================

.. toctree::
   :maxdepth: 1

   quickstart
   tutorial
   syntax_guide
   syntax_details
   simulators
   new_simulator
   internals
   credits

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

License
=======

Scenic is distributed under the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`_.
