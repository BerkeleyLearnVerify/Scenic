Welcome to Scenic's documentation!
==================================

Scenic is a domain-specific probabilistic programming language for modeling the environments of cyber-physical systems like robots and autonomous cars.
A Scenic program defines a distribution over *scenes*, configurations of physical objects and agents; sampling from this distribution yields concrete scenes which can be simulated to produce training or testing data.
Scenic can also define (probabilistic) policies for dynamic agents, allowing modeling scenarios where agents take actions over time in response to the state of the world.

Scenic was designed and implemented by Daniel J. Fremont, Edward Kim, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.
For a description of the language and some of its applications, see `our journal paper <https://link.springer.com/article/10.1007/s10994-021-06120-5>`_, which extends `our PLDI 2019 paper <https://arxiv.org/abs/1809.09310>`_ on Scenic 1.x.
Our :doc:`publications <publications>` page lists additional papers using Scenic.

.. note::

   The syntax of Scenic 2.x is not completely backwards-compatible with 1.x, which was used in our papers prior to late 2020. See :doc:`new` for a list of syntax changes and new features.
   If your existing code no longer works, install the latest 1.x release from
   `GitHub <https://github.com/BerkeleyLearnVerify/Scenic/releases>`__.

If you have any problems using Scenic, please submit an issue to `our GitHub repository <https://github.com/BerkeleyLearnVerify/Scenic>`_ or contact Daniel at dfremont@ucsc.edu.

Table of Contents
=================

.. toctree::
   :maxdepth: 1
   :caption: Introduction

   quickstart
   install_notes

.. toctree::
   :maxdepth: 1
   :caption: Tutorials

   tutorials/tutorial
   tutorials/dynamics
   tutorials/composition

.. toctree::
   :maxdepth: 1
   :caption: Language and Tool Reference

   syntax_guide
   syntax_details
   options
   api
   developing
   internals

.. toctree::
   :maxdepth: 1
   :caption: Libraries and Simulators

   libraries
   simulators
   new_simulator

.. toctree::
   :maxdepth: 1
   :caption: General Information

   new
   publications
   credits

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :doc:`glossary`

License
=======

Scenic is distributed under the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`_.
