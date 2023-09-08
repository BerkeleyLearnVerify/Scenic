..  _objects_and_classes:

*****************************
Objects and Classes Reference
*****************************

This page describes the classes built into Scenic, representing `points <Point>`, `oriented points <OrientedPoint>`, and physical `objects <Object>`, and how they are instantiated to create objects.

.. note::

    The documentation given here describes only the public properties and methods provided by the built-in classes.
    If you are working on Scenic's internals, you can find more complete documentation in the :mod:`scenic.core.object_types` module.

.. _objectCreate:
.. _new:

Instance Creation
-----------------

.. code-block:: scenic-grammar

    new <class> [<specifier> [, <specifier>]*]

Instantiates a Scenic object from a Scenic class.
The properties of the object are determined by the given set of zero or more specifiers.
For details on the available specifiers and how they interact, see the :ref:`specifiers`.

Instantiating an instance of `Object` has a side effect: the object is added to the scenario being defined.

.. versionchanged:: 3.0

    Instance creation now requires the ``new`` keyword. As a result, Scenic classes can be referred to without creating an instance.

Built-in Classes
----------------

..
    N.B. the following cross-reference target deliberately has the same name as the Point class.
    In 'conf.py' we introduce our own reference resolver for the :any: role which makes :ref: targets take precedence over :obj: targets normally, but the other way around when rendering a document in the 'modules' folder.
    This has the effect of making a cross-reference like `Point` in the main documentation link to the high-level description here, but the same reference in the auto-generated "Scenic Internals" section will instead link to the internal documentation for the Point class.

.. _Point:
.. _points:

Point
+++++

Locations in space.
This class provides the fundamental property :prop:`position` and several associated properties.

.. autoscenicclass:: scenic.core.object_types.Point
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: visibleRegion

.. _OrientedPoint:
.. _oriented points:

OrientedPoint
+++++++++++++

A location along with an orientation, defining a local coordinate system.
This class subclasses `Point`, adding the fundamental property :prop:`orientation` and several associated properties.

.. autoscenicclass:: scenic.core.object_types.OrientedPoint
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: visibleRegion

.. _Object:
.. _objects:

Object
++++++

A physical object.
This class subclasses `OrientedPoint`, adding a variety of properties including:

* :prop:`width`, :prop:`length`, and :prop:`height` to define the dimensions of the object;
* :prop:`shape` to define the `Shape` of the object;
* :prop:`allowCollisions`, :prop:`requireVisible`, and :prop:`regionContainedIn` to control the built-in requirements that apply to the object;
* :prop:`behavior`, specifying the object's :term:`dynamic behavior` if any;
* :prop:`speed`, :prop:`velocity`, and other properties capturing the dynamic state of the object during simulations.

The built-in requirements applying to each object are:

* The object must be completely contained within its :term:`container`, the region specified as its :prop:`regionContainedIn` property (by default the entire :term:`workspace`).
* The object must be visible from the ego object if the :prop:`requireVisible` property is set to `True` (default value `False`).
* The object must not intersect another object (i.e., their bounding boxes must not overlap), unless either of the two objects has their :prop:`allowCollisions` property set to `True`.

.. versionchanged:: 3.0

    :prop:`requireVisible` is now `False` by default.

.. autoscenicclass:: scenic.core.object_types.Object
    :noindex:
    :no-show-inheritance:
    :no-members:
    :members: startDynamicSimulation, visibleRegion
