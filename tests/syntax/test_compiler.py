from ast import *
from scenic.syntax.ast import *

from scenic.syntax.compiler import compileScenicAST


class TestCompiler:
    # Special Case
    def test_ego_assign(self):
        node, _ = compileScenicAST(EgoAssign(Constant(1)))
        match node:
            case Expr(Call(func=Name(id=fn_name), args=args)):
                assert fn_name == "ego"
                assert args[0].value == 1
            case _:
                assert False

    # Instance & Specifiers
    def test_new_no_specifiers(self):
        node, _ = compileScenicAST(New("Object", []))
        match node:
            case Call(func=Name(id=className)):
                assert className == "Object"
            case _:
                assert False
