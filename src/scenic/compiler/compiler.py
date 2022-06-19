import ast
from typing import Tuple, List
import scenic.ast as s

# exposed functions

def compileScenicAST(scenicAST: s.AST) -> Tuple[ast.AST, List[ast.AST]]:
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
    
    # Special Case

    def visit_EgoAssign(self, node: s.EgoAssign):
        return ast.Expr(value=ast.Call(
            func=ast.Name(id="ego", ctx=loadCtx),
            args=[self.visit(node.value)],
            keywords=[],
        ))

    # Instance & Specifier

    def visit_New(self, node: s.New):
        return ast.Call(
            func=ast.Name(id=node.className, ctx=loadCtx),
            args=[],
            keywords=[],
        )
