# Scenic

[![Documentation Status](https://readthedocs.org/projects/scenic-lang/badge/?version=latest)](https://scenic-lang.readthedocs.io/en/latest/?badge=latest)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A compiler and scenario generator for the Scenic scenario description language.
Please see the [documentation](https://scenic-lang.readthedocs.io/) for installation instructions, as well as tutorials and other information about the Scenic language, its implementation, and its interfaces to various simulators.

For a description of the language and some of its applications, see [our preprint](https://arxiv.org/abs/2010.06580), which extends our [PLDI 2019 paper](https://arxiv.org/abs/1809.09310) (*note:* the syntax of Scenic has changed slightly since that paper, and many features such as support for dynamic scenarios have been added; these are described in the preprint).
Scenic was designed and implemented by Daniel J. Fremont, Edward Kim, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.

If you have any problems using Scenic, please submit an issue to [our GitHub repository](https://github.com/BerkeleyLearnVerify/Scenic) or contact Daniel at <dfremont@ucsc.edu>.

The repository is organized as follows:

* the _src/scenic_ directory contains the package proper;
* the _examples_ directory has many examples of Scenic programs;
* the _docs_ directory contains the sources for the documentation;
* the _tests_ directory contains tests for the Scenic compiler.
