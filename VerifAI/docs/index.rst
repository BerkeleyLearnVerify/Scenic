############################################
Welcome to VerifAI's documentation!
############################################

VerifAI is a software toolkit for the formal design and analysis of systems that include artificial intelligence (AI) and machine learning (ML) components.
VerifAI particularly seeks to address challenges with applying formal methods to perception and ML components, including those based on neural networks, and to model and analyze system behavior in the presence of environment uncertainty.
The current version of the toolkit performs intelligent simulation guided by formal models and specifications, enabling a variety of use cases including temporal-logic falsification (bug-finding), model-based systematic fuzz testing, parameter synthesis, counterexample analysis, and data set augmentation.
Our `CAV 2019 paper <https://people.eecs.berkeley.edu/~sseshia/pubs/b2hd-verifai-cav19.html>`_, which is the basis of the tutorial below, illustrates all of these use cases: see our :doc:`publications <publications>` page for further applications.

VerifAI was designed and implemented by Tommaso Dreossi, Daniel J. Fremont, Shromona Ghosh, Edward Kim, Hadi Ravanbakhsh, Marcell Vazquez-Chanlatte, and Sanjit A. Seshia.

If you use VerifAI in your work, please cite our `CAV 2019 paper`_ and link to `our GitHub repository <https://github.com/BerkeleyLearnVerify/VerifAI>`_.

If you have any problems using VerifAI, please submit an issue to `the repository <https://github.com/BerkeleyLearnVerify/VerifAI>`_ or contact Daniel Fremont at dfremont@ucsc.edu or Edward Kim at ek65@berkeley.edu.


Table of Contents
=================

.. toctree::
   :maxdepth: 2

   installation
   basic_usage
   tutorial
   feature_api
   samplers
   server_client
   dynamics
   parallel_falsification
   multi_objective
   publications


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

License
=======

VerifAI is distributed under the `3-Clause BSD License <https://opensource.org/licenses/BSD-3-Clause>`_.
