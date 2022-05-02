..  _objects_and_classes:

*****************************
Objects and Classes Reference
*****************************

This page describes the classes built into Scenic, representing :ref:`points <Point>`, :ref:`oriented points <OrientedPoint>`, and physical :ref:`objects <Object>`, and how they are instantiated to create objects.

.. note::

    The documentation given here describes only the public properties and methods provided by the built-in classes.
    If you are working on Scenic's internals, you can find more complete documentation in the :mod:`scenic.core.object_types` module.

.. _objectCreate:

Instance Creation
-----------------

::

    <class> [<specifier> [, <specifier>]*]

Instantiates a Scenic object from a Scenic class.
The properties of the object are determined by the given set of zero or more specifiers.
For details on the available specifiers and how they interact, see the :ref:`specifiers`.

Instantiating an instance of `Object` has a side effect: the object is added to the scenario being defined.

Names of Scenic classes followed immediately by punctuation are not considered instance creations.
This allows us to refer to a Scenic class without creating an instance of that class in the environment, which is useful for expressions like ``isinstance(obj, Car)``, ``[Taxi, Truck]``, ``Car.staticMethod``, etc.

Built-in Classes
----------------

..
    N.B. the following cross-reference target deliberately has the same name as the Point class.
    In 'conf.py' we introduce our own reference resolver for the :any: role which makes :ref: targets take precedence over :obj: targets normally, but the other way around when rendering a document in the 'modules' folder.
    This has the effect of making a cross-reference like `Point` in the main documentation link to the high-level description here, but the same reference in the auto-generated "Scenic Internals" section will instead link to the internal documentation for the Point class.

.. _Point:

Point
+++++

Locations in space.
This class provides the fundamental property ``position`` and several associated properties.

.. autoclass:: scenic.core.object_types.Point
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: visibleRegion

.. _OrientedPoint:

OrientedPoint
+++++++++++++

A location along with an orientation, defining a local coordinate system.
This class subclasses `Point`, adding the fundamental property ``heading`` and several associated properties.

.. autoclass:: scenic.core.object_types.OrientedPoint
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: visibleRegion

.. _Object:

Object
++++++

A physical object.
This class subclasses `OrientedPoint`, adding a variety of properties including:

* ``width`` and ``length`` to define the bounding box of the object;
* ``allowCollisions``, ``requireVisible``, and ``regionContainedIn`` to control the built-in requirements that apply to the object;
* ``behavior``, specifying the object's :term:`dynamic behavior` if any;
* ``speed``, ``velocity``, and other properties capturing the dynamic state of the object during simulations.

The built-in requirements applying to each object are:

* The object must be completely contained within its *container*, the region specified as its ``regionContainedIn`` property (by default the entire workspace).
* The object must be visible from the ego object, unless its ``requireVisible`` property is set to `False`.
* The object must not intersect another object (i.e., their bounding boxes must not overlap), unless either of the two objects has their ``allowCollisions`` property set to `True`.

.. autoclass:: scenic.core.object_types.Object
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: startDynamicSimulation, visibleRegion
