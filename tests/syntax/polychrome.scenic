"""Example file to help test the Scenic syntax highlighter."""

# A comment on its own line.

import collections.abc as abc
from collections import defaultdict as dd, foo as bar, \
    baz as qux

model scenic.simulators.webots.mars.model
simulator NoSuchClass()

param map = 3   # name of param should NOT be formatted as built-in function
param 'quoted name' = str(globalParameters.map)
param foo = f(x, y=3), bar = (1, 2, 3), \
    baz = 'zoggle', qux = [     # comment in list
        1.4, 9.16]

variable = sin(3) + cos(1) - hypot(4, 1) * min(5, 9) ** max(2, 6)
CONSTANT = 30
workspace = Workspace(everywhere)
field = VectorField('myfield', lambda pos: 30 deg / pos.x)
region = nowhere.union(PolygonalRegion([[0, 0], [1, 0], [1, 1], [0, 1]]))
stuff = filter(lambda thing={}: bool(thing["foo"]),
               ({ "foo": 12.2, },))
lambda x, *args, \
    **kwargs: NotImplemented
lots   ( of_  ,  whitespace    =  12 )
next (it)           # not a temporal operator
thing = newObject   # not an object definition
visible = True      # ill-advised but legal variable name
distance = visible  # ditto

print(f"this \n is a test from {localPath(__file__)}")
verbosePrint("zounds!", level=1)
match = re.match(r'(?x) (a|b)* [^abc#\n]+')
re.search(rf'(.*)\1 {foo} {{')

Uniform
new Object
ego = new Object
blah = new Point    # comment after no specifiers
spot = new OrientedPoint with position (10, 10), facing 30 deg relative to field
new Point at (Range(1, 3), 2) # comment
new Object offset by (3, 4), facing 30 deg
other = new Object offset along 45 deg by [5, 6],
    facing toward (7, 8)
new Object left of spot by 5, facing toward (0, 0),
    with behavior None, with name 'string', with foo {"k1": 3, Point: ego}
new OrientedPoint behind ego,
    with property 42,
    with items (1, (2, 3), [4, (5, 6)]), with thing f(1, b=2),
    apparently facing -30 deg from spot
new Object beyond ego by (0, 5) from spot,  # comment
    with prop """multiline
                string""",
    facing (30 +
        12) deg,    # implicit line continuation
    with prop2 'explicit line ' \
               'continuation'
new Object at spot, \
    facing 30 deg   # specifier on same logical line as previous
new Point visible from ego,
    with children [new Other for _ in range(60)],
    with friends (new Other at 1@2 for _ in range(60)),
    with cousins (new Thing1, new Thing2)
pt = new Point not visible
new Object on visible workspace
new Object in not visible region
new Object following field from spot for 12
new Car
new thingy
new thingy facing 30 deg

require (relative heading of other from ego) > 1 as RequirementName
require[0.5] (apparent heading of spot) > 0 as 'quoted name'
require always 3 <= (distance to other) <= 10
require eventually (angle from other to front of spot) < 1
require eventually X or (always Y)
require X until Y
require always (X < 5 implies (next x > 5))
terminate when ego can see back right of other
terminate when (other in visible region) as MyTerminationCondition
terminate after (field at spot) seconds
terminate after CONSTANT steps
record (other.heading relative to field) as myval
record initial (ego offset by 5@2) as 'quoted name'
record final other offset along field by (3, 2)

class MyClass:
    property: self.foo + 7     # name of property should NOT be formatted as built-in function
    requireVisible: False
    model: (Point, OrientedPoint , Object)
    foo[additive]: new Point on road
    bar[additive, dynamic]: 'blah'
@mod.decorator(1, 2, foo=3)     # comment
@other_decorator
class OtherClass(MyClass):
    foo: 21
    def method(self, arg:Union[list,types.Wuggle]=[1,2], *args, distance=5,    # comment
        blah, \
        **kwargs: anno
    ):
        """Docstring."""
        thing = args[0] if len(args) > 0 else 6
        return self.foo + distance
    @property
    def myProp(
        self,
        *,
        zug=4
    ) -> Point:
        if abs(self.foo) != 4: x = 3; y = 12    # if-else with simple statements
        else: x = 4
        def nested(foo, /):
            yield from foo
        return self.method(5, womp='foo')
    class_attr = 42
    @classmethod
    def clsmethod(cls, angle : float, /,
                  thingy:types.Wuggle
                  ):
        try:
            pass
        except Exception as e:
            raise angle
        except:
            raise cls
        else:
            pass
        finally:
            pass

        try: x = 0; y = 1
        except Exception as e: raise
        except: raise
        else: pass
        finally: pass

        try: yield x
        finally: yield from x

        raise Exception(cls.class_attr)

        case = match
        match thingy:
            case 42 | 149:
                return thingy
            case Point(x=0, y=y) if y < 5:
                return 0
            case _:
                return 'default'
    def __str__(self):
        return super().__str__()
    __hash__ = None
class MultipleBases(MyClass, defaultdict):
    __slots__ = ('foo', 'bar')

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
monitor Mon(x, y=12):
    wait
require monitor Mon(x=4)

scenario Main():
    precondition: True
    invariant: ... is ...
    setup:
        terminate after 50 steps
    compose:
        do shuffle Simple(), Simple(3)
scenario Simple(arg=5):
    if initial scenario:
        ego = new Object
    else:
        override ego with behavior B1(0, 1), with foo 19

# Check case of object created with no newline following
new Object