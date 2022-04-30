..  _objects_and_classes:

*****************************
Objects and Classes Reference
*****************************

This page describes the classes built into Scenic, and how they are instantiated to create objects.
Physical objects present in a scenario are represented by instances of `Object` (the default superclass of user-defined classes), a subclass of `OrientedPoint`, which in turn derives from the base class `Point`.

.. _objectCreate:

Instance Creation
-----------------

::

    <class> [<specifier> [, <specifier>]*]

Instantiates a Scenic object from a Scenic class.
The properties of the object are determined by the given set of zero or more specifiers.
For details on the available specifiers and how they interact, see the :ref:`specifiers`.

Names of Scenic classes followed immediately by punctuation are not considered instance creations.
This allows us to refer to a Scenic class without creating an instance of that class in the environment, which is useful for expressions like ``isinstance(obj, Car)``, ``[Taxi, Truck]``, ``Car.staticMethod``, etc.

Built-in Classes
----------------

Point
+++++

Locations in space.
This class provides the fundamental property ``position`` and several associated properties.

.. autoclass:: scenic.core.object_types.Point
    :noindex:

OrientedPoint
+++++++++++++

A location along with an orientation, defining a local coordinate system.
This class subclasses `Point`, adding the fundamental property ``heading`` and several associated properties.

.. autoclass:: scenic.core.object_types.OrientedPoint
    :noindex:

Object
++++++

A physical object.
This class subclasses `OrientedPoint`, adding a variety of properties including:

* ``width`` and ``length`` to define the bounding box of the object;
* ``allowCollisions``, ``requireVisible``, and ``regionContainedIn`` to control the built-in requirements that apply to the object;
* ``behavior``, specifying the object's :term:`dynamic behavior` if any;
* ``speed``, ``velocity``, and other properties capturing the dynamic state of the object during simulations.

.. autoclass:: scenic.core.object_types.Object
    :noindex:
