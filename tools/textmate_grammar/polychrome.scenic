"""Example file to help test the Scenic syntax highlighter."""

# A comment on its own line.

import collections.abc as abc
from collections import defaultdict

model scenic.simulators.webots.mars.model
simulator NoSuchClass()

param map = 3   # name of param should NOT be formatted as built-in function
param 'quoted name' = str(globalParameters.map)

variable = sin(3) + cos(1) - hypot(4, 1) * min(5, 9) ** max(2, 6)
CONSTANT = 30
workspace = Workspace(everywhere)
field = VectorField('myfield', lambda pos: 30 deg / pos.x)
region = nowhere.union(PolygonalRegion([[0, 0], [1, 0], [1, 1], [0, 1]]))
stuff = filter(lambda thing={}: bool(thing["foo"]),
               ({ "foo": 12.2, },))

print(f"this is a test from {localPath(__file__)}")
verbosePrint("zounds!", level=1)

Object
ego = Object
spot = OrientedPoint with position (10, 10), facing 30 deg relative to field
Point at (1, 2) # comment
Object offset by (3, 4), facing 30 deg
other = Object offset along 45 deg by (5, 6),
    facing toward (7, 8)
Object left of spot by 5, facing toward (0, 0),
    with behavior None, with name 'string'
OrientedPoint behind ego,
    with property 42,
    apparently facing -30 deg from spot
Object beyond ego by (0, 5) from spot,  # comment
    with prop """multiline
                string""",
    facing (30 +
        12) deg,    # implicit line continuation
    with prop2 'explicit line ' \
               'continuation'
Object at spot, \
    facing 30 deg   # specifier on same logical line as previous
Point visible from ego
pt = Point not visible
Object on visible workspace
Object in not visible region
Object following field from spot for 12

require (relative heading of other from ego) > 1 as RequirementName
require[0.5] (apparent heading of spot) > 0 as 'quoted name'
require always 3 <= (distance to other) <= 10
require eventually (angle from other to front of spot) < 1
terminate when ego can see back right of other
terminate when (other in visible region) as MyTerminationCondition
terminate after (field at spot) seconds
terminate after CONSTANT steps
record (other.heading relative to field) as myval
record initial (ego offset by 5@2) as 'quoted name'
record final other offset along field by (3, 2)

class MyClass:
    property: self.foo + 7     # name of property should NOT be formatted as built-in function
    foo: 12
class OtherClass(MyClass):
    foo: 21
    def method(self, arg, *args, darg=5, **kwargs):
        """Docstring."""
        return self.foo + darg
    @property
    def myProp(self):
        return self.method(5, womp='foo')
    class_attr = 42
    @classmethod
    def clsmethod(cls):
        raise Exception(cls.class_attr)
    def __str__(self):
        return super().__str__()
    __hash__ = None
class MultipleBases(MyClass, defaultdict):
    pass

behavior B1(arg, arg2):
    precondition: simulation().currentTime > arg
    invariant: (distance to other) < arg2
    while True:
        if arg > 0:
            break
        elif arg < -5:
            continue
        else:
            wait
        take MyAction()
        take OtherAction(3), OtherAction(4)
    try:
        do B2()
        terminate
    interrupt when ego.heading > 0:
        do B2() until ego.heading < 0
        do B2(4) for 3 seconds
        abort
    except GuardViolation as e:
        pass
behavior B2(darg=5):
    do choose B1(1, 2), B1(3, 4), B1(5, 6)
    do shuffle {B1(1,2): 1, B1(3,4): 2}
monitor Mon:
    wait

scenario Main():
    precondition: True
    invariant: ... is ...
    setup:
        terminate after 50 steps
    compose:
        do shuffle Simple(), Simple(3)
scenario Simple(arg=5):
    if initial scenario:
        ego = Object
    else:
        override ego with behavior B1(0, 1), with foo 19

# Check case of object created with no newline following
Object