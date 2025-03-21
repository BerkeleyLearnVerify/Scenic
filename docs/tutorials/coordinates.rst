..  _coordinates:

Coordinate Systems
==================
Scenic's coordinate system consists of a global coordinate system based on intrinsic Euler angles and a local coordinate system
which can built and used via differnt properties and specifiers. We will describe the coordinate systems in the more detail below.

Global Coordinate System
------------------------
All objects in Scenic have an :prop:`orientation` property, which is their orientation in global coordinates.
Scenic represents orientations internally using quaternions, though for convenience they can be created using Euler angles. Scenic follows the right hand rule with the Z,X,Y order of rotations. In other words, Euler angles are given as (Yaw, Pitch, Roll), in radians, and applied in that order.

:prop:`orientation` is derived from the values of 4 other properties, :prop:`parentOrientation`, :prop:`yaw`, :prop:`pitch`, and :prop:`roll`. 
The :prop:`parentOrientation` property defines the parent orientation of the object, which is the orientation with respect to which the intrinsic Euler angles :prop:`yaw`, :prop:`pitch`, and :prop:`roll` are interpreted. 
Specifically, :prop:`orientation` is obtained as follows:

  1. start from :prop:`parentOrientation`;
  2. apply a yaw (a :abbr:`CCW (counter-clockwise)` rotation around the positive Z axis) of :prop:`yaw`;
  3. apply a pitch (a CCW rotation around the resulting positive X axis) of :prop:`pitch`;
  4. apply a roll (a CCW rotation around the resulting positive Y axis) of :prop:`roll`.

Below is an illustration of the global coordinate system

.. image:: /images/global_coords.jpg
	:width: 50%

By default, :prop:`parentOrientation` is aligned with the global coordinate system, so that :prop:`yaw` for example is just the angle by which to rotate the object around the Z axis.

This corresponds to the :prop:`heading` property in older versions of Scenic. :prop:`heading` represents yaw in the global XY plane. Scenic represents headings in radians, measured anticlockwise from North, so that a heading of 0 is due North and a heading of π/2 is due West. 

For more information on orientations and heading please consult:

    * Tutorial on Orientations :ref:`orientations_tutorial`
    * Reference on Orientations: :ref:`Orientation`.
    * Reference on Heading: :ref:`Heading`

Local Coordinate System
-----------------------
If you want to specify an objects coordinate system relative to other objects, there are specifiers for the :prop:`orientation` to do so. 

    1. The :specifier:`on {region}` specifier specifies :prop:`parentOrientation` whenever the region in question has a :term:`preferred orientation`: a `vector field` (another primitive Scenic type) which defines an orientation at each point in the region.
    2. The :specifier:`ahead of` specifier only specifies :prop:`parentOrientation` *optionally*, giving it a new default value: if you want a different value, you can override that default by explicitly writing :specifier:`with parentOrientation {value}`.
    3. The general operator :scenic:`{X} relative to {Y}` can interpret vectors and orientations as being in a variety of local coordinate systems
    4. The `OrientedPoint` class adds :prop:`position` and :prop:`orientation` (plus :prop:`parentOrientation`, :prop:`yaw`, etc.) to define a local coordinate sytem.

For more information on defining a local coordinate system, please consult:

    * Tutorial on :scenic:`relative to`: :ref:`dependencies_and_modifying_specifiers`
    * Reference on :scenic:`relative to` for headings: :ref:`operators`
    * Reference on `OrientedPoint`: :ref:`OrientedPoint`

