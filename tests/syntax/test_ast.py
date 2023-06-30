import ast
import sys

from scenic.syntax.compiler import LocalFinder
from tests.utils import compileScenic, sampleEgoFrom, sampleParamPFrom


def test_method_of_expr():
    ego = sampleEgoFrom("ego = new Object at (12@3).distanceTo(13@3) @ 0")
    assert ego.x == 1


def test_local_finder():
    def localsOf(block):
        module = ast.parse(block)
        return LocalFinder.findIn(module.body)

    assert localsOf("x = y") == {"x"}
    assert localsOf("x, y = z") == {"x", "y"}
    assert localsOf("global x; x = y") == set()
    assert localsOf("nonlocal x; x = y") == set()
    assert localsOf("import x, y, z as w") == {"x", "y", "w"}
    assert localsOf("from m import x, y as z") == {"x", "z"}
    assert localsOf("del x") == {"x"}
    assert localsOf("x = (y := z)") == {"x", "y"}
    assert localsOf("global y; x = (y := z)") == {"x"}
    assert localsOf("@dec(a=(c:=b))\ndef f(x, y=(z:=w)) -> (s:=r): q = 4") == {
        "c",
        "f",
        "z",
        "s",
    }
    assert localsOf("class C(x:=y): z = w") == {"C", "x"}
    assert localsOf("for x in (y := z): w = q") == {"x", "y", "w"}
    assert localsOf("with (x := y) as z: w = q") == {"x", "z", "w"}
    assert localsOf("try: x = y\nexcept (q := r) as e: z = w") == {"x", "q", "e", "z"}
