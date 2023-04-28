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

You can rotate and move the camera of the visualizer around using the mouse. The only `Object` currently present is the one we created using the ``new`` command
(rendered as a green box). Since we assigned this object to the ``ego`` name, it has special significance to Scenic, as we'll see later. For now it only has the effect of highlighting the
object green in Scenic's visualizer. Pressing :kbd:`w` will render all objects as wireframes, which will allow you to see the coordinate axes in the center of 
the ego object (at the origin).

Since we didn't provide any additional information to Scenic about this object, it created it with all of a Scenic `Object`'s default values (more info available in the :ref:`Classes Reference <objects_and_classes>`).
Scenic provides a flexible, natural language, way for setting properties of objects, **specifiers**. Scenic has many specifiers, too many to explore in this tutorial, but a quick reference can be found
in the :ref:`syntax_guide` and a more in depth reference in the :ref:`specifiers`.

Now let's look at a slightly more complicated program:

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

The first object we create, the ego, has a cone shape. Scenic provides several built in shapes like
this (see the bottom of the :ref:`syntax_guide` for more examples). We then set the object's dimensions
using the :specifier:`with` specifier, which can set any property (even properties unknown to Scenic). Finally,
we set the object's global orientation using the :specifier:`facing` specifier. The tuple after :specifier:`facing`
contains the Euler angles of the desired orientation (yaw, pitch, roll).

The second object we create is first placed at a specific point in space using the :specifier:`at` specifier.
We then set its shape to one imported from a mesh file, using the `MeshShape` class, applying an initial rotation to align the mesh with
where Scenic expects the front to be. We also set default dimensions of the shape, which the object will then
automatically inherit. If we hadn't set these default dimensions, Scenic would automatically infer the dimensions
from the mesh file.

On line 10 we load a shape from a file, specifically to highlight that since Scenic is built on top of Python,
we can write arbitrary Python expressions in Scenic (with some exceptions).

For our third and final object, we first place it left of ``chair`` (the second object) by 1 unit.
We set its shape and dimensions, similar to before, and then have it face directly toward the ego object,
using the :specifier:`facing directly toward`.

Scenic will automatically reject scenarios where objects are intersecting (unless :prop:`allowCollisions` is turned off).
For an example of this, try changing the code above to have a much larger ego object, to the point where it would intersect
with the plane. While this isn't too important in the scenarios we've seen so far, it becomes very useful when we start constructing
*random* scenarios.

Randomness, Regions, and more Specifiers
----------------------------------------

So far all of our Scenic programs have been static, i.e. every time we generate a scene it will be exactly the same.
This is because so far we haven't introduced any *randomness*. Scenic is a *probabilistic programming language*,
meaning it can have random elements. 

Let's look at a simple Scenic program with some random elements:

.. code-block:: scenic
	:linenos:

	ego = new Object with shape Uniform(BoxShape(), SpheroidShape(), ConeShape()),
			with width Uniform(1,2),
			with length Uniform(1,2),
			with height Uniform(1,3),
			facing (Uniform(0,360) deg, Uniform(0,360) deg, Uniform(0,360) deg)

This will generate an object with a shape that is either a box, a sphere, or a cone (with equal probability of each).
It will have a random width, length, and height (between the bounds specified), and a totally random orientation.

Here are some of the objects this program might generate:

.. image:: /images/simple_random_1.png
   :width: 32%
.. image:: /images/simple_random_2.png
   :width: 32%
.. image:: /images/simple_random_3.png
   :width: 32%

Randomness in Scenic does have its limits however, the most important of which is that control flow can't depend
on random values.

Another key construct in Scenic is a `Region`, which at its core represents a collection of points. Regions can be used
to sample points, check containment of objects, and more. Regions can also be made the `Workspace` of a scene, which
enforces that all objects are contained in the region, allows Scenic to optimize sampling through pruning techniques,
and is displayed in the internal visualizer. A simple example is presented below:

.. code-block:: scenic
	:linenos:

	region = RectangularRegion((0,0,0), 0, 10, 10)
	workspace = Workspace(region)

	new Object in region,
	    with shape SpheroidShape()

	new Object in region,
	    with shape SpheroidShape()

	new Object in region,
	    with shape SpheroidShape()

Which should generate a scene similar to this:

.. figure:: /images/spheres_in_region.png
  :width: 60%
  :figclass: align-center
  :alt: Three spheres in a rectangular region


We first create a `RectangularRegion`, and set it as the scene's workspace. `RectangularRegion` is a 2D region,
meaning it does not have a volume and therefore can't really contain objects. However, since it's the scene's workspace
it must contain all objects. To solve this problem, Scenic will automatically use a 2D region's *footprint* for
containment checks, which extends infinitely in either z direction. We then create 3 objects with sphere shapes,
and place them uniformly at random in the region, using the :specifier:`in` specifier which sets the :prop:`position`
of an object (its center) uniformly at random in the region.

Similarly, the :specifier:`on` specifier is used to place the *base* of an object uniformly at random in a region,
where the base is by default the center of the bottom side if its bounding box. :specifier:`on` is also overloaded
to work on objects, by default extracting the top surface of the object's mesh and placing the object on that.
This can lead to very compact syntax for randomly placing objects on others, as seen in the following example:

.. code-block:: scenic
	:linenos:

	workspace = Workspace(RectangularRegion((0,0,0), 0, 4, 4))
	floor = workspace

	chair = new Object on floor,
	            with shape MeshShape.fromFile(path=localPath("meshes/chair.obj"), type="obj",
	                dimensions=(1,1,1), initial_rotation=(0,90 deg,0))

	ego = new Object on chair,
	            with shape ConeShape(dimensions=(0.25,0.25,0.25))

Which might generate something like this:

.. figure:: /images/on_chair.png
  :width: 80%
  :figclass: align-center
  :alt: A cone on a chair

Notice how in this example the cone is oriented to be tangent with the curved surface of the chair, even though we
never set an orientation. :specifier:`on`, like many other specifiers that specify :prop:`position`, also specifies
:prop:`parentOrientation`. :prop:`parentOrientation` is a property that determines the local coordinate system for a
Scenic object. For the :specifier:`on`, the orientation is taken from a *vector field* attatched to the region. 
`MeshSurfaceRegion`, the class used to represent surfaces of an object, automatically defines an orientation
that for each point on the surface, equal to the normal vector of the face of the mesh containing that point. This allows
us to effortlessly place objects on regions, even if they have an irregular surface. Other specifiers might use
different sources for :prop:`parentOrientation`.

Classes
-------

In the previous example placing spheres in a region, we explicitly wrote out the specifiers for each object we created
even though they were all identical. Thankfully Scenic provides a way to avoid this repetition, Scenic classes. 
Consider an equivalent Scenic program:

.. code-block:: scenic
	:linenos:

	region = RectangularRegion((0,0,0), 0, 10, 10)
	workspace = Workspace(region)

	class SphereObject():
		position: Point in region
		shape: SpheroidShape()

	new SphereObject
	new SphereObject
	new SphereObject

Scenic classes use the property syntax with ``:`` to define properties. Properties can have random values, and much like 
Python classes they can inherit from other classes.

Models and Simulators
---------------------

For the next part of this tutorial, we'll move beyond the internal Scenic visualizer to an actual simulator, specifically
examples from our case study using Scenic to generate traffic scenes in GTA V to test and train autonomous cars [F22]_, [F19]_.

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
operators use :scenic:`ego` by default when a position is left implicit [#f1]_.

Finally, line 3 creates a second :scenic:`Car`. Compiling this scenario with Scenic, sampling a
scene from it, and importing the scene into GTA V yields an image like this:

.. figure:: /images/simplest2.jpg
  :width: 80%
  :figclass: align-center
  :alt: Simple car scenario image.

  A scene sampled from the simple car scenario, rendered in GTA V.

Note that both the :scenic:`ego` car (where the camera is located) and the second car are both
located on the road and facing along it, despite the fact that the code above does not
specify the position or any other properties of the two cars. This is because they have already
been specified in the :scenic:`Car` definition, which begins:

.. code-block::
	:linenos:

	class Car:
	    position: Point in road
	    yaw: roadDirection at self.position
	    width: self.model.width
	    height: self.model.height
	    model: CarModel.defaultModel()	# a distribution over several car models


Here ``road`` is a region defined in the `gta` model to specify which points in the workspace 
are on a road. Similarly, ``roadDirection`` is a vector field specifying the nominal traffic direction 
at such points. The operator :scenic:`{F} at {X}` simply gets the direction of the field *F* at point *X*, so line 3
sets a :scenic:`Car`'s default yaw to be the road direction at its :prop:`position`. The default
:prop:`position`, in turn, is a :scenic:`Point in road`, which means a uniformly random point on the road. 
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
	car2 = new Car offset by Range(-10, 10) @ Range(20, 40), with viewAngle 30 deg
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
distribution so that the requirement is satisfied [#f2]_. As this example illustrates,
the ability to declaratively impose constraints gives Scenic greater versatility than
purely-generative formalisms. Requirements also improve encapsulation by allowing us to
restrict an existing scenario without altering it. For example::

	import genericTaxiScenario    # import another Scenic scenario
	fifthAvenue = ...             # extract a Region from a map here
	require genericTaxiScenario.taxi on fifthAvenue

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
Some of the specifiers and operators used have not been discussed before in the tutorial,
but information about them can be found in the :ref:`syntax_guide`

We will write a scenario representing a rubble field of rocks and piples with a
bottleneck between the rover and its goal that forces the path planner to consider
climbing over a rock. First, we import a small Scenic library for the Webots robotics
simulator (`scenic.simulators.webots.mars.model`) which defines the (empty) workspace
and several types of objects: the :scenic:`Rover` itself, the :scenic:`Goal` (represented by a flag), and
debris classes :scenic:`Rock`, :scenic:`BigRock`, and :scenic:`Pipe`. :scenic:`Rock` and :scenic:`BigRock` have fixed sizes, and
the rover can climb over them; :scenic:`Pipe` cannot be climbed over, and can represent a pipe of
arbitrary length, controlled by the :prop:`length` property (which corresponds to Scenic's
*y* axis).

.. code-block::
	:linenos:

	from scenic.simulators.webots.mars.model import *

Then we create the rover at a fixed position and the goal at a random position on the
other side of the workspace:

.. code-block::
	:lineno-start: 2

	ego = new Rover at (0, -2)
	goal = new Goal at (Range(-2, 2), Range(2, 2.5))

Next we pick a position for the bottleneck, requiring it to lie roughly on the way from
the robot to its goal, and place a rock there. Here facing takes a scalar arguments,
which will implicitly set the global yaw:

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

	new BigRock beyond bottleneck by Range(-0.5, 0.5) @ Range(0.5, 1)
	new BigRock beyond bottleneck by Range(-0.5, 0.5) @ Range(0.5, 1)
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

Our page on :ref:`dynamics` describes how to define scenarios
with dynamic agents that move or take other actions over time.

For a comprehensive overview of Scenic's syntax, including details on all specifiers,
operators, distributions, statements, and built-in classes, see the
:ref:`syntax_details`. Our :ref:`syntax_guide` summarizes all of these language
constructs in convenient tables with links to the detailed documentation.

.. rubric:: Footnotes

.. [#f1] In fact, since :scenic:`ego` is a variable and can be reassigned, we can set :scenic:`ego` to
   one object, build a part of the scene around it, then reassign :scenic:`ego` and build
   another part of the scene.

.. [#f2] On the other hand, Scenic may have to work hard to satisfy difficult
   constraints. Ultimately Scenic falls back on rejection sampling, which in the worst
   case will run forever if the constraints are inconsistent (although we impose a limit
   on the number of iterations: see `Scenario.generate`).

.. rubric:: References

.. [F22] Fremont et al., :t:`Scenic: A Language for Scenario Specification and Data Generation`, Machine Learning, 2022. `[Online] <https://doi.org/10.1007/s10994-021-06120-5>`_

.. [F19] Fremont et al., :t:`Scenic: A Language for Scenario Specification and Scene Generation`, PLDI 2019.
