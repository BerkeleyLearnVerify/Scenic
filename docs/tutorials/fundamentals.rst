..  _tutorial:

Scenic Fundamentals
===================

This tutorial motivates and illustrates the main features of Scenic, focusing on aspects
of the language that make it particularly well-suited for describing geometric scenarios.
We begin by walking through Scenic's core features from first principles, using simple
toy examples displayed in Scenic's built-in visualizer. We then consider discuss two case studies in depth: using Scenic to generate traffic scenes to test and train autonomous cars (as in [F22]_, [F19]_),
and testing a motion planning algorithm for a Mars rover able to climb over rocks. These examples
show Scenic interfacing with actual simulators, and demonstrate how it can be applied to real problems.

We'll focus here on the *spatial* aspects of scenarios; for adding *temporal* dynamics to a scenario, see our page on :ref:`dynamics`.

Objects, Geometry, and Specifiers
---------------------------------

To start with, we'll construct a very basic Scenic program:

.. code-block:: scenic
	:linenos:

	ego = new Object

Running this program should cause a window to pop up, looking like this:

.. figure:: /images/ego_box.png
  :width: 40%
  :figclass: align-center
  :alt: Simple scenario with an ego box, rendered with Scenic's built-in visualizer.

You can rotate and move the camera of the visualizer around using the mouse. The only `Object` currently present is the one we created using the ``new`` keyword
(rendered as a green box). Since we assigned this object to the :scenic:`ego` name, it has special significance to Scenic, as we'll see later. For now it only has the effect of highlighting the
object green in Scenic's visualizer. Pressing :kbd:`w` will render all objects as wireframes, which will allow you to see the coordinate axes in the center of 
the ego object (at the origin).

Since we didn't provide any additional information to Scenic about this object, its **properties** like :prop:`position`, :prop:`orientation`, :prop:`width`, etc. were assigned default values from the object's class: here, the built-in class `Object`, representing a physical object.
So we end up with a generic cube at the origin.
To define the properties of an object, Scenic provides a flexible system of **specifiers** based on the many ways one can describe the position and orientation of an object in natural language.
We can see a few of these specifiers in action in the following slightly more complex program (see the :ref:`syntax_guide` for a summary of all the specifiers, and the :ref:`specifiers` for detailed definitions):

.. code-block:: scenic
	:linenos:

	ego = new Object with shape ConeShape(),
	        with width 2,
	        with length 2,
	        with height 1.5,
	        facing (-90 deg, 45 deg, 0)

	chair = new Object at (4,0,2),
	            with shape MeshShape.fromFile(localPath("meshes/chair.obj"), type="obj",
	                initial_rotation=(0,90 deg,0), dimensions=(1,1,1))

	plane_shape = MeshShape.fromFile(path=localPath("meshes/plane.obj"), type="obj")

	plane = new Object left of chair by 1,
	            with shape plane_shape,
	            with width 2,
	            with length 2,
	            with height 1,
	            facing directly toward ego

This should generate the following scene:

.. figure:: /images/cone_plane_chair.png
  :width: 60%
  :figclass: align-center
  :alt: A slightly more complicated scenario showing the use of specifiers.

The first object we create, the :scenic:`ego`, has a cone shape. Scenic provides several built-in shapes like
this (see `Shape` for a list). We then set the object's dimensions
using the :specifier:`with` specifier, which can set any property (even properties not built into Scenic, which you might access in your own code or which a particular simulator might understand). Finally,
we set the object's global orientation (its :prop:`orientation` property) using the :specifier:`facing` specifier. The tuple after :specifier:`facing`
contains the Euler angles of the desired orientation (yaw, pitch, roll).

The second object we create is first placed at a specific point in space using the :specifier:`at` specifier (setting the object's :prop:`position` property).
We then set its shape to one imported from a mesh file, using the `MeshShape` class, applying an initial rotation to tell Scenic which side of the chair is its front.
We also set default dimensions of the shape, which the object will then
automatically inherit.
If we hadn't set these default dimensions, Scenic would automatically infer the dimensions
from the mesh file.

On line 11 we load a shape from a file, specifically to highlight that since Scenic is built on top of Python,
we can write arbitrary Python expressions in Scenic (with some exceptions).

For our third and final object, we use the :specifier:`left of` specifier to place it to the left of ``chair`` (the second object) by 1 unit.
We set its shape and dimensions, similar to before, and then orient it to face directly toward the ego object using the :specifier:`facing directly toward` specifier.
This gives a first hint of the power of specifiers, with Scenic automatically working out how to compute the object's :prop:`orientation` so that it faces the :scenic:`ego` regardless of how we specified its :prop:`position` (in fact, we could move the :specifier:`left of` specifier to be after the :specifier:`facing directly toward` and the code would still work).

Scenic will automatically reject scenarios that don't make physical sense, for instance when objects intersect each other [#f1]_.
For an example of this, try changing the code above to have a much larger ego object, to the point where it would intersect
with the plane. While this isn't too important in the scenarios we've seen so far, it becomes very useful when we start constructing
*random* scenarios.

Randomness, Regions, and More Specifiers
----------------------------------------

So far all of our Scenic programs have defined concrete scenes, i.e. they uniquely define all the aspects of a scene, so every time we run the program we'll get the same scene.
This is because so far we haven't introduced any *randomness*. Scenic is a *probabilistic programming language*,
meaning a single Scenic program can in fact define a probability distribution over many possible scenes.

Let's look at a simple Scenic program with some random elements:

.. code-block:: scenic
	:linenos:

	ego = new Object with shape Uniform(BoxShape(), SpheroidShape(), ConeShape()),
			 with width Range(1,2),
			 with length Range(1,2),
			 with height Range(1,3),
			 facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg)

This will generate an object with a shape that is either a box, a spheroid, or a cone (each with equal probability).
It will have a random width, length, and height within the ranges specified, and uniformly random rotation angles.
Some examples:

.. image:: /images/simple_random_1.png
   :width: 32%
.. image:: /images/simple_random_2.png
   :width: 32%
.. image:: /images/simple_random_3.png
   :width: 32%

Random values can be used almost everywhere in Scenic; the major exception is that control flow (e.g. :keyword:`if` statements and :keyword:`for` loops) cannot depend on random values.
This restriction enables more efficient sampling (see [F19]_) and can often be worked around: for example it is still possible to select random elements satisfying desired criteria from lists (see :ref:`filter_func`).

Another key construct in Scenic is a `Region`, which represents a set of points in space.
Having defined a region of interest, for example a lane of a road, you can then sample points from it, check whether objects are contained in it, etc.
You can also use a region to define the **workspace**, a designated region which all objects in the scenario must be contained in (useful, for example, if the simulated world has fixed obstacles that Scenic objects should not intersect).
For example, the following code:

.. code-block:: scenic
	:linenos:

	region = RectangularRegion((0,0,0), 0, 10, 10)
	workspace = Workspace(region)

	new Object in region, with shape SpheroidShape()
	new Object in region, with shape SpheroidShape()
	new Object in region, with shape SpheroidShape()

should generate a scene similar to this:

.. figure:: /images/spheres_in_region.png
  :width: 60%
  :figclass: align-center
  :alt: Three spheres in a rectangular region


We first create a 10-unit square `RectangularRegion`, and set it as the scenario's workspace. `RectangularRegion` is a 2D region,
meaning it does not have a volume and therefore can't really contain objects.
It is still a valid workspace, however, since for containment checks involving 2D regions, Scenic automatically uses the region's *footprint*, which extends infinitely in the positive and negative Z directions.
We then create 3 spherical objects and place them using the :specifier:`in` specifier, which sets the :prop:`position`
of an object (its center) to a uniformly-random point in the given region.

Similarly, we can use the :specifier:`on` specifier to place the *base* of an object uniformly at random in a region,
where the base is by default the center of the bottom side of its bounding box.
The :specifier:`on` specifier is also overloaded
to work on objects, by default extracting the top surface of the object's mesh and placing the object on that.
This can lead to very compact syntax for randomly placing objects on others, as seen in the following example:

.. code-block:: scenic
	:linenos:

	workspace = Workspace(RectangularRegion((0,0,0), 0, 4, 4))
	floor = workspace

	chair = new Object on floor,
	            with shape MeshShape.fromFile(path=localPath("meshes/chair.obj"), type="obj",
	                dimensions=(1,1,1), initial_rotation=(0, 90 deg, 0))

	ego = new Object on chair,
	            with shape ConeShape(dimensions=(0.25,0.25,0.25))

which might generate something like this:

.. figure:: /images/on_chair.png
  :width: 80%
  :figclass: align-center
  :alt: A cone on a chair

Notice how in this example the cone is oriented to be tangent with the curved surface of the chair, even though we
never set the cone's orientation.
This works because the :specifier:`on` specifier, like many other specifiers that specify :prop:`position`, also specifies :prop:`parentOrientation`.
The :prop:`parentOrientation` property determines the local coordinate system that a Scenic object is oriented with respect to.
By default, it is identical to the global coordinate system, but in our example, :specifier:`on` specifies it to be tangent to the surface of the chair.
We can then apply further rotations with respect to that coordinate system by specifying the :prop:`yaw`, :prop:`pitch`, and :prop:`roll` properties of the object: for example, adding :scenic:`with yaw 30 deg` would rotate the cone 30 degrees from its orientation above while leaving it tangent to the chair.
If instead we don't care about the cone being tangent to the chair and want to orient it in the global coordinate system, the :specifier:`facing` specifier will still work as above.

Classes
-------

In the previous example placing spheres in a region, we explicitly wrote out the specifiers for each object we created
even though they were all identical.
Such repetition can often be avoided by using functions and loops, and by defining a **class** of object providing new default values for properties of interest.
Our example can be equivalently written:

.. code-block:: scenic
	:linenos:

	workspace = Workspace(RectangularRegion((0,0,0), 0, 10, 10))

	class SphereObject:
	    position: new Point in workspace
	    shape: SpheroidShape()

	for i in range(3):
	    new SphereObject

Here we define the ``SphereObject`` class, providing new default values for the :prop:`position` and :prop:`shape` properties, overriding those inherited from `Object` (the default superclass if none is explicitly given).
So for example the default :prop:`position` for a ``SphereObject`` is the expression :scenic:`new Point in workspace`, which evaluates to a uniformly-random point in the workspace (`Point` is a superclass of `Object` that has a :prop:`position` but no :prop:`orientation`).
Note that this is a random expression, and it is evaluated independently each time a ``SphereObject`` is defined: so the loop creates 3 objects which will all have different positions (and as usual Scenic will ensure they do not overlap).
We can still override the default value as needed: adding the line :scenic:`new SphereObject at (0,0,5)` would create a ``SphereObject`` which still used the default value of :prop:`shape` but whose :prop:`position` is exactly :scenic:`(0,0,5)`.

Models and Simulators
---------------------

For the next part of this tutorial, we'll move beyond the internal Scenic visualizer to an actual simulator.
Specifically, we will consider examples from our case study using Scenic to generate traffic scenes in GTA V to test and train autonomous cars ([F19]_, [F22]_).

To start, suppose we want scenes of one car viewed from another on the road. We can write
this very concisely in Scenic:

.. py:currentmodule:: scenic.simulators.gta.model

.. code-block:: scenic
	:linenos:

	from scenic.simulators.gta.model import Car
	ego = new Car
	new Car visible

Line 1 imports the GTA :term:`world model`, a Scenic library defining everything specific to our
GTA interface. This includes the definition of the class :obj:`Car`, as well as information
about the road geometry that we'll see later. We'll suppress this :scenic:`import` statement in
subsequent examples.

Line 2 then creates a :scenic:`Car` and assigns it to the special variable :scenic:`ego` specifying the
*ego object*, as seen before. This is the reference point for the scenario: our simulator interfaces
typically use it as the viewpoint for rendering images, and many of Scenic's geometric
operators use :scenic:`ego` by default when a position is left implicit [#f2]_.

Finally, line 3 creates a second :scenic:`Car`. Compiling this scenario with Scenic, sampling a
scene from it, and importing the scene into GTA V yields an image like this:

.. figure:: /images/simplest2.jpg
  :width: 80%
  :figclass: align-center
  :alt: Simple car scenario image.

  A scene sampled from the simple car scenario, rendered in GTA V.

Note that both the :scenic:`ego` car (where the camera is located) and the second car are both
located on the road and facing along it, despite the fact that the code above does not
specify the position or any other properties of the two cars. This is because reasonable default values for these properties have already
been defined in the :scenic:`Car` definition, which begins:

.. code-block::
	:linenos:

	class Car:
	    position: new Point on road
	    heading: roadDirection at self.position    # note: can only set `heading` in 2D mode
	    width: self.model.width
	    height: self.model.height
	    model: CarModel.defaultModel()	# a distribution over several car models


Here ``road`` is a region defined in the `gta` model to specify which points in the workspace 
are on a road. Similarly, ``roadDirection`` is a **vector field** (another built-in Scenic datatype) specifying the nominal traffic direction 
at such points. The operator :scenic:`{F} at {X}` simply gets the direction of the field *F* at point *X*, so line 3
sets a :scenic:`Car`'s default heading to be the road direction at its :prop:`position`. The default
:prop:`position`, in turn, is a :scenic:`new Point on road`, which means a uniformly random point on the road. 
Thus, in our simple scenario above both cars will be placed on the road facing a reasonable direction, without our having to
specify this explicitly.

Declarative Hard and Soft Constraints
-------------------------------------

Notice that in the scenarios above we never explicitly ensured that two cars will not
intersect each other. Despite this, Scenic will never generate such scenes. This is
because Scenic enforces several *default requirements*, as mentioned above:

	* All objects must be contained in the :term:`workspace`, or a particular specified region (its :term:`container`).
	  For example, we can define the :scenic:`Car` class so that all of its instances must be
	  contained in the region ``road`` by default.

	* Objects must not intersect each other (unless explicitly allowed).

Scenic also allows the user to define custom requirements checking arbitrary conditions
built from various geometric predicates. For example, the following scenario produces a
car headed roughly towards the camera, while still facing the nominal road direction::

	ego = new Car on road
	car2 = new Car offset by (Range(-10, 10), Range(20, 40)), with viewAngle 30 deg
	require car2 can see ego

Here we have used the :scenic:`{X} can see {Y}` predicate, which in this case is checking
that the ego car is inside the 30° view cone of the second car.

Requirements, called *observations* in other probabilistic programming languages, are
very convenient for defining scenarios because they make it easy to restrict attention to
particular cases of interest. Note how difficult it would be to write the scenario above
without the :scenic:`require` statement: when defining the ego car, we would have to somehow
specify those positions where it is possible to put a roughly-oncoming car 20--40 meters
ahead (for example, this is not possible on a one-way road). Instead, we can simply place
:scenic:`ego` uniformly over all roads and let Scenic work out how to condition the
distribution so that the requirement is satisfied [#f3]_. As this example illustrates,
the ability to declaratively impose constraints gives Scenic greater versatility than
purely-generative formalisms. Requirements also improve encapsulation by allowing us to
restrict an existing scenario without altering it. For example::

	from myScenarioLib import genericTaxiScenario    # import another Scenic scenario
	fifthAvenue = ...             # extract a Region from a map here
	require genericTaxiScenario.taxi in fifthAvenue

The constraints in our examples above are *hard requirements* which must always be
satisfied. Scenic also allows imposing *soft requirements* that need only be true with
some minimum probability::

	require[0.5] car2 can see ego	# condition only needs to hold with prob. >= 0.5

Such requirements can be useful, for example, in ensuring adequate representation of a
particular condition when generating a training set: for instance, we could require that
at least 90% of generated images have a car driving on the right side of the road.

Mutations
---------

A common testing paradigm is to randomly generate *variations* of existing tests. Scenic
supports this paradigm by providing syntax for performing mutations in a compositional
manner, adding variety to a scenario without changing its code. For example, given a
complex scenario involving a taxi, we can add one additional line::

	from bigScenario import taxi
	mutate taxi

The :scenic:`mutate` statement will add Gaussian noise to the :prop:`position` and :prop:`orientation`
properties of ``taxi``, while still enforcing all built-in and custom requirements. The
standard deviation of the noise can be scaled by writing, for example,
:scenic:`mutate taxi by 2` (which adds twice as much noise), and in fact can be controlled
separately for :prop:`position` and :prop:`orientation` (see `scenic.core.object_types.Mutator`).

A Worked Example
----------------

We conclude with a larger example of a Scenic program which also illustrates the
language's utility across domains and simulators. Specifically, we consider the problem
of testing a motion planning algorithm for a Mars rover able to climb over rocks. Such
robots can have very complex dynamics, with the feasibility of a motion plan depending on
exact details of the robot's hardware and the geometry of the terrain. We can use Scenic
to write a scenario generating challenging cases for a planner to solve in simulation.
Some of the specifiers and operators we'll use have not been discussed before in the tutorial; as usual, information about them can be found in the :ref:`syntax_guide`.

We will write a scenario representing a rubble field of rocks and pipes with a
bottleneck between the rover and its goal that forces the path planner to consider
climbing over a rock. First, we import a small Scenic library for the Webots robotics
simulator (`scenic.simulators.webots.mars.model`) which defines the (empty) workspace
and several types of objects: the :scenic:`Rover` itself, the :scenic:`Goal` (represented by a flag), and
debris classes :scenic:`Rock`, :scenic:`BigRock`, and :scenic:`Pipe`. :scenic:`Rock` and :scenic:`BigRock` have fixed sizes, and
the rover can climb over them; :scenic:`Pipe` cannot be climbed over, and can represent a pipe of
arbitrary length, controlled by the :prop:`length` property (which corresponds to Scenic's
*Y* axis).

.. code-block::
	:linenos:

	model scenic.simulators.webots.mars.model

Here we've used the :keyword:`model` statement to select the :term:`world model` for the scenario: it is equivalent to :scenic:`from scenic.simulators.webots.mars.model import *` except that the choice of model can be overridden from the command line when compiling the scenario (using the :option:`--model` option).
This is useful for scenarios that use one of Scenic's :ref:`domains`: the scenario can be written once in a simulator-agnostic manner, then used with different simulators by selecting the appropriate simulator-specific world model.

We next create the rover at a fixed position and the goal at a random position on the
other side of the workspace:

.. code-block::
	:lineno-start: 2

	ego = new Rover at (0, -2)
	goal = new Goal at (Range(-2, 2), Range(2, 2.5))

Next we pick a position for the bottleneck, requiring it to lie roughly on the way from
the robot to its goal, and place a rock there. Since this is a 2D scenario, we can use the simple form of :specifier:`facing` which takes a scalar argument, effectively setting the yaw of the object in the global coordinate system (so that :scenic:`0 deg` is due North, for example, and :scenic:`90 deg` is due West).

.. code-block::
	:lineno-start: 4

	bottleneck = new OrientedPoint offset by (Range(-1.5, 1.5), Range(0.5, 1.5)),
	                           facing Range(-30, 30) deg
	require abs((angle to goal) - (angle to bottleneck)) <= 10 deg
	new BigRock at bottleneck

Note how we define ``bottleneck`` as an :scenic:`OrientedPoint`, with a range of possible
orientations: this is to set up a local coordinate system for positioning the pipes
making up the bottleneck. Specifically, we position two pipes of varying lengths on
either side of the bottleneck, with their ends far enough apart for the robot to be able
to pass between:

.. code-block::
	:lineno-start: 8

	halfGapWidth = (1.2 * ego.width) / 2
	leftEnd = new OrientedPoint left of bottleneck by halfGapWidth,
	                        facing Range(60, 120) deg relative to bottleneck
	rightEnd = new OrientedPoint right of bottleneck by halfGapWidth,
	                         facing Range(-120, -60) deg relative to bottleneck
	new Pipe ahead of leftEnd, with length Range(1, 2)
	new Pipe ahead of rightEnd, with length Range(1, 2)

Finally, to make the scenario slightly more interesting, we add several additional
obstacles, positioned either on the far side of the bottleneck or anywhere at random
(recalling that Scenic automatically ensures that no objects will overlap).

.. code-block::
	:lineno-start: 15

	new BigRock beyond bottleneck by (Range(-0.5, 0.5), Range(0.5, 1))
	new BigRock beyond bottleneck by (Range(-0.5, 0.5), Range(0.5, 1))
	new Pipe
	new Rock
	new Rock
	new Rock

This completes the scenario, which can also be found in the Scenic repository under
:file:`examples/webots/mars/narrowGoal.scenic`. Several scenes generated from the
scenario and visualized in Webots are shown below.

.. figure:: /images/mars1.jpg
  :width: 80%
  :figclass: align-center
  :alt: Mars rover scenario image.

  A scene sampled from the Mars rover scenario, rendered in Webots.

.. image:: /images/mars3.jpg
   :width: 32%
.. image:: /images/mars4.jpg
   :width: 32%
.. image:: /images/mars5.jpg
   :width: 32%

Further Reading
---------------

This tutorial illustrated the syntax of Scenic through several simple examples. Much more
complex scenarios are possible, such as the platoon and bumper-to-bumper traffic GTA V
scenarios shown below. For many further examples using a variety of simulators, see the
:file:`examples` folder, as well as the links in the :ref:`simulators` page.

.. image:: /images/platoon2.jpg
   :width: 32%
.. image:: /images/platoon3.jpg
   :width: 32%
.. image:: /images/platoon4.jpg
   :width: 32%

.. image:: /images/btb1.jpg
   :width: 32%
.. image:: /images/btb3.jpg
   :width: 32%
.. image:: /images/btb4.jpg
   :width: 32%

Our tutorial on :ref:`dynamics` describes how to define scenarios
with dynamic agents that move or take other actions over time.
We also have a tutorial on :ref:`composition`: defining scenarios in a modular, reusable way and combining them to build up more complex scenarios.

For a comprehensive overview of Scenic's syntax, including details on all specifiers,
operators, distributions, statements, and built-in classes, see the
:ref:`syntax_details`. Our :ref:`syntax_guide` summarizes all of these language
constructs in convenient tables with links to the detailed documentation.

.. rubric:: Footnotes

.. [#f1] Although collisions can be allowed on a per-object basis: see the :prop:`allowCollisions` property of `Object`.

.. [#f2] In fact, since :scenic:`ego` is a variable and can be reassigned, we can set :scenic:`ego` to
   one object, build a part of the scene around it, then reassign :scenic:`ego` and build
   another part of the scene.

.. [#f3] On the other hand, Scenic may have to work hard to satisfy difficult
   constraints. Ultimately Scenic falls back on rejection sampling, which in the worst
   case will run forever if the constraints are inconsistent (although we impose a limit
   on the number of iterations: see `Scenario.generate`).

.. rubric:: References

.. [F22] Fremont et al., :t:`Scenic: A Language for Scenario Specification and Data Generation`, Machine Learning, 2022. `[Online] <https://doi.org/10.1007/s10994-021-06120-5>`_

.. [F19] Fremont et al., :t:`Scenic: A Language for Scenario Specification and Scene Generation`, PLDI 2019.
