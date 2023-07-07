:orphan:

.. _porting to Scenic 3:

Porting to Scenic 3
===================

As described in :ref:`whats_new`, Scenic 2 programs are not compatible with Scenic 3 due to a few changes in syntax and semantics.
See that page for a complete list of backwards-incompatibilities and explanations of how you can change your code.
This page describes two tools that assist in the migration process: the Scenic 2-to-3 converter and 2D compatibility mode.
The former converts a Scenic 2 program into a syntactically-valid Scenic 3 program; the latter is a compiler option that changes the semantics of several Scenic constructs so that they behave as in Scenic 2.
The goal of these tools is that applying them together to a Scenic 2 program makes it possible to run the program under Scenic 3 and get largely the same behavior.

.. note::

    2D compatibility mode does *not* exactly emulate the behavior of Scenic 2: for example, it does not disable fixes for bugs.
    It is intended as a temporary measure to help easily run old Scenic programs without fully porting them to 3D geometry.
    If it is essential for your application that you can reproduce the exact behavior of your Scenic 2 scenarios, you should not upgrade to Scenic 3: old releases of Scenic are always available on PyPI and in our GitHub repository.
    For most use cases, however, upgrading to Scenic 3 and using 2D mode until you can digest our documentation and port your scenarios will work just fine.

Scenic 2-to-3 Converter
+++++++++++++++++++++++

This tool reads a Scenic 2 program and adjusts its syntax so that it parses under Scenic 3.
It requires the Scenic 2 parser in order to work, so it is not included in Scenic 3: you must either use the tool before upgrading, or temporarily check out the ``2.x`` branch of our repository to get the latest Scenic 2 release (you can switch back to the ``main`` branch afterward).

To run the tool and see the list of options it supports, run this command in the environment where you have Scenic installed:

.. code-block:: console

    $ python -m scenic.syntax.scenic2to3 --help

Note that due to the nature of Scenic 2 parsing, the tool must actually execute your Scenic 2 program, so you will have to ensure your program runs before you can convert it.

.. _2D compatibility mode:

2D Compatibility Mode
+++++++++++++++++++++

Running the :command:`scenic` command with the :option:`--2d` option enables 2D compatibility mode.
This mode changes several aspects of Scenic's semantics in order to more closely match the historical behavior of Scenic 2.
Specifically:

* The :prop:`baseOffset` and :prop:`contactTolerance` properties of `Object` are zeroed, so that the specifier :sampref:`on {region}` places the :prop:`position` of the object within the region, as it did in Scenic 2 (vs. the Scenic 3 behavior of placing the object *above* that position so that the base of the object lies on the region).

* The :prop:`requireVisible` property of `Object` is true by default, as it was in Scenic 2.

* The :prop:`occluding` property of `Object` is false by default, so that objects do not occlude each other for the purpose of visibility checks (as Scenic 2 did not account for occlusion).

* The :term:`visible regions` of `Point`, `OrientedPoint`, and `Object` are 2D regions as in Scenic 2 (either a `CircularRegion` or a `SectorRegion`).

* Default values for :prop:`heading` in class definitions are replaced with default values for :prop:`parentOrientation`.

* The specifier :specifier:`with heading {X}` is replaced with :specifier:`facing {X}`.

Note that despite these changes, Scenic will still use 3D geometry internally.
For example, if you write :scenic:`ego = new Object at (1, 2)` the value of :scenic:`ego.position` will be the 3D vector :scenic:`(1, 2, 0)`.
