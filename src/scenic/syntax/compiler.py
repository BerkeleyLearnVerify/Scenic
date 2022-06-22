import ast
from typing import Tuple, List
import scenic.syntax.ast as s

# exposed functions


def compileScenicAST(scenicAST: ast.AST) -> Tuple[ast.AST, List[ast.AST]]:
    """Compiles Scenic AST to Python AST"""
    compiler = ScenicToPythonTransformer()
    node = ast.fix_missing_locations(compiler.visit(scenicAST))
    return node, compiler.requirements


# shorthands for convenience

loadCtx = ast.Load()

# transformer


class ScenicToPythonTransformer(ast.NodeTransformer):
    def __init__(self) -> None:
        super().__init__()
        self.requirements = []

    def generic_visit(self, node):
        if isinstance(node, s.AST):
            raise Exception(
                f'Scenic AST node "{node.__class__.__name__}" needs visitor in compiler'
            )
        return super().generic_visit(node)

    # Special Case

    def visit_EgoAssign(self, node: s.EgoAssign):
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id="ego", ctx=loadCtx),
                args=[self.visit(node.value)],
                keywords=[],
            )
        )

    # Instance & Specifier

    def visit_New(self, node: s.New):
        return ast.Call(
            func=ast.Name(id=node.className, ctx=loadCtx),
            args=[],
            keywords=[],
        )

    def visit_WithSpecifier(self, node: s.WithSpecifier):
        return ast.Call(
            func=ast.Name(id="With", ctx=loadCtx),
            args=[
                ast.Constant(value=node.prop),
                self.visit(node.value),
            ],
            keywords=[],
        )

    def visit_AtSpecifier(self, node: s.AtSpecifier):
        return ast.Call(
            func=ast.Name(id="At", ctx=loadCtx),
            args=[node.position],
            keywords=[],
        )

    def visit_OffsetBySpecifier(self, node: s.OffsetBySpecifier):
        return ast.Call(
            func=ast.Name(id="OffsetBy", ctx=loadCtx),
            args=[
                self.visit(node.offset),
            ],
            keywords=[],
        )

    def visit_OffsetAlongSpecifier(self, node: s.OffsetAlongSpecifier):
        return ast.Call(
            func=ast.Name(id="OffsetAlongSpec", ctx=loadCtx),
            args=[
                self.visit(node.direction),
                self.visit(node.offset),
            ],
            keywords=[],
        )

    def visit_PositionSpecifier(self, node: s.PositionSpecifier):
        if isinstance(node.direction, s.LeftOf):
            fn = "LeftSpec"
        elif isinstance(node.direction, s.RightOf):
            fn = "RightSpec"
        elif isinstance(node.direction, s.AheadOf):
            fn = "Ahead"
        elif isinstance(node.direction, s.Behind):
            fn = "Behind"
        else:
            raise TypeError(
                f'"{node.direction.__class__.__name__}" cannot be used as a direction'
            )
        return ast.Call(
            func=ast.Name(id=fn, ctx=loadCtx),
            args=[
                self.visit(node.position),
            ],
            keywords=(
                []
                if node.distance is None
                else [ast.keyword(arg="dist", value=self.visit(node.distance))]
            ),
        )

    def visit_VisibleSpecifier(self, node: s.VisibleSpecifier):
        if node.base is not None:
            return ast.Call(
                func=ast.Name(id="VisibleFrom", ctx=loadCtx),
                args=[self.visit(node.base)],
                keywords=[],
            )
        return ast.Call(
            func=ast.Name(id="VisibleSpec", ctx=loadCtx),
            args=[],
            keywords=[],
        )
