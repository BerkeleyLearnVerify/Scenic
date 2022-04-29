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

OrientedPoint
+++++++++++++

Object
++++++
