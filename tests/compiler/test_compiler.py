import ast

import scenic.syntax.ast as s
from scenic.syntax.compiler import compileScenicAST

class TestCompiler:
    # Special Case
    def test_ego_assign(self):
        node, _ = compileScenicAST(s.EgoAssign(ast.Constant(1)))
        match node:
            case ast.Expr(ast.Call(func=ast.Name(id=fn_name), args=args)):
                assert fn_name == "ego"
                assert args[0].value == 1
            case _:
                assert False

    # Instance & Specifiers
    def test_new_no_specifiers(self):
        node, _ = compileScenicAST(s.New("Object", []))
        match node:
            case ast.Call(func=ast.Name(id=className)):
                assert className == "Object"
            case _:
                assert False
