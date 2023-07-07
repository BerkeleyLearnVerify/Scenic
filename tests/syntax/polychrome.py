"""Example file to help test the Python syntax highlighter."""

# A comment on its own line.

import collections.abc as abc
from collections import defaultdict as dd, foo as bar, \
    baz as qux

baz = 'zoggle'; qux = [     # comment in list
    1.4, 9.16]

variable = sin(3) + cos(1) - hypot(4, 1) * min(5, 9) ** max(2, 6)
CONSTANT = 30
workspace = Workspace(everywhere)
field = VectorField('myfield', lambda pos: 30 / pos.x)
region = nowhere.union(PolygonalRegion([[0, 0], [1, 0], [1, 1], [0, 1]]))
stuff = filter(lambda thing={}: bool(thing["foo"]),
               ({ "foo": 12.2, },))
lots   ( of  ,  whitespace    =  12 )
next (it)
lambda x, *args, \
    **kwargs: NotImplemented

print(f"this \n is a test from {localPath(__file__)}")
verbosePrint("zounds!", level=1)
match = re.match(r'(?x) (a|b)* [^abc#\n]+')
case = re.search(rf'(.*)\1 {foo} {{')

thing("""multiline
      string""",
      (30 +
      12),    # implicit line continuation
      'explicit line ' \
      'continuation')

class MyClass:
    model : str = 'hello'
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
