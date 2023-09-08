..  _libraries:

****************
Scenic Libraries
****************

One of the strengths of Scenic is its ability to reuse functions, classes, and behaviors
across many scenarios, simplifying the process of writing complex scenarios. This page
describes the libraries built into Scenic to facilitate scenario writing by end users.

Simulator Interfaces
====================

Many of the simulator interfaces provide utility functions which are useful when writing
scenarios for particular simulators. See the documentation for each simulator on the
:ref:`simulators` page, as well as the corresponding module under `scenic.simulators`.

.. _domains:

Abstract Domains
================

To enable cross-platform scenarios which are not specific to one simulator, Scenic
defines *abstract domains* which provide APIs for particular application domains like
driving scenarios. An abstract domain defines a protocol which can be implemented by
various simulator interfaces so that scenarios written for that domain can be executed in
those simulators. For example, a scenario written for our
:ref:`driving domain <driving_domain>` can be run in both LGSVL and CARLA.

A domain provides a Scenic :term:`world model` which defines Scenic classes for the various types
of objects that occur in its scenarios. The model also provides a simulator-agnostic way
to access the geometry of the simulated world, by defining regions, vector fields, and
other objects as appropriate (for example, the driving domain provides a `Network` class
abstracting a road network). For domains which support dynamic scenarios, the model will
also define a set of simulator-agnostic actions for dynamic agents to use.

..  _driving_domain:

Driving Domain
--------------

The driving domain, `scenic.domains.driving`, is designed to support scenarios taking
place on or near roads. It defines generic classes for cars and pedestrians, and provides
a representation of a road network that can be loaded from standard map formats (e.g.
`OpenDRIVE <https://www.asam.net/standards/detail/opendrive/>`_). The domain supports
dynamic scenarios, providing actions for agents which can drive and walk as well as
implementations of common behaviors like lane following and collision avoidance. See the
documentation of the `scenic.domains.driving` module for further details.
