import ast
from typing import Tuple, List

import scenic.syntax.ast as s

# exposed functions


def compileScenicAST(scenicAST: ast.AST) -> Tuple[ast.AST, List[ast.AST]]:
    """Compiles Scenic AST to Python AST"""
    compiler = ScenicToPythonTransformer()
    node = ast.fix_missing_locations(compiler.visit(scenicAST))
    return node, compiler.requirements


# constants

createDefault = "PropertyDefault"
builtInConstructors = {"Object", "OrientedPoint", "Point"}

# shorthands for convenience

loadCtx = ast.Load()
selfArg = ast.arguments(
    posonlyargs=[],
    args=[ast.arg(arg="self", annotation=None)],
    vararg=None,
    kwonlyargs=[],
    kw_defaults=[],
    kwarg=None,
    defaults=[],
)

# helpers


class AttributeFinder(ast.NodeVisitor):
    """Utility class for finding all referenced attributes of a given name."""

    @staticmethod
    def find(target, node):
        af = AttributeFinder(target)
        af.visit(node)
        return af.attributes

    def __init__(self, target):
        super().__init__()
        self.target = target
        self.attributes = set()

    def visit_Attribute(self, node):
        val = node.value
        if isinstance(val, ast.Name) and val.id == self.target:
            self.attributes.add(node.attr)
        self.visit(val)


# transformer


class ScenicToPythonTransformer(ast.NodeTransformer):
    def __init__(self) -> None:
        super().__init__()
        self.requirements = []
        self.constructors = set(builtInConstructors)

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

    def visit_ClassDef(self, node: ast.ClassDef) -> any:
        # if no base class is specified, use Object
        if not node.bases:
            node.bases = [ast.Name(id="Object", ctx=ast.Load())]

        # create a set of names of base classes
        baseNames = set(
            y
            for base in node.bases
            # omit if the base class is given not as a name
            for y in ([base.id] if isinstance(base, ast.Name) else [])
        )

        # if node base classes contain a Scenic class
        if baseNames & self.constructors:
            # add this class to the set of Scenic classes
            self.constructors.add(node.name)
            return self.transformScenicClass(node)
        else:
            # TODO(shun): Is any assertion necessary here?
            return self.generic_visit(node)

    def transformScenicClass(self, node):
        """Process property defaults for Scenic classes."""
        newBody = []
        for child in node.body:
            child = self.visit(child)
            if isinstance(child, ast.AnnAssign):  # default value for property
                origValue = child.annotation
                target = child.target
                # extract any attributes for this property
                metaAttrs = []
                if isinstance(target, ast.Subscript):
                    sl = target.slice
                    # needed for compatibility with Python 3.8 and earlier
                    if isinstance(sl, ast.Index):
                        sl = sl.value
                    if isinstance(sl, ast.Name):
                        metaAttrs.append(sl.id)
                    elif isinstance(sl, Tuple):
                        for elt in sl.elts:
                            if not isinstance(elt, ast.Name):
                                raise SyntaxError(
                                    "malformed attributes for property default"
                                )
                            metaAttrs.append(elt.id)
                    else:
                        raise SyntaxError("malformed attributes for property default")
                    newTarget = ast.Name(target.value.id, ast.Store())
                    ast.copy_location(newTarget, target)
                    target = newTarget
                # find dependencies of the default value
                properties = AttributeFinder.find("self", origValue)
                # create default value object
                args = [
                    ast.Set([ast.Str(prop) for prop in properties]),
                    ast.Set([ast.Str(attr) for attr in metaAttrs]),
                    ast.Lambda(selfArg, origValue),
                ]
                value = ast.Call(ast.Name(createDefault, ast.Load()), args, [])
                ast.copy_location(value, origValue)
                newChild = ast.AnnAssign(
                    target=target, annotation=value, value=None, simple=True
                )
                child = ast.copy_location(newChild, child)
            newBody.append(child)
        node.body = newBody
        return node

    def visit_Param(self, node: s.Param):
        d = dict()
        for parameter in node.elts:
            if parameter.identifier in d:
                raise SyntaxError(f'Duplicated param "{parameter.identifier}"')
            d[parameter.identifier] = self.visit(parameter.value)
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id="param", ctx=loadCtx),
                args=[
                    ast.Dict(
                        [ast.Constant(k) for k in d.keys()],
                        list(d.values()),
                    )
                ],
                keywords=[],
            )
        )

    # Instance & Specifier

    def visit_New(self, node: s.New):
        return ast.Call(
            func=ast.Name(id=node.className, ctx=loadCtx),
            args=[self.visit(s) for s in node.specifiers],
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
            args=[self.visit(node.position)],
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

    def visit_DirectionOfSpecifier(self, node: s.DirectionOfSpecifier):
        if isinstance(node.direction, s.LeftOf):
            fn = "LeftSpec"
        elif isinstance(node.direction, s.RightOf):
            fn = "RightSpec"
        elif isinstance(node.direction, s.AheadOf):
            fn = "Ahead"
        elif isinstance(node.direction, s.Behind):
            fn = "Behind"
        else:
            assert False, f"impossible direction {node.direction} in PositionSpecifier"
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

    def visit_BeyondSpecifier(self, node: s.BeyondSpecifier):
        return ast.Call(
            func=ast.Name(id="Beyond", ctx=loadCtx),
            args=[self.visit(node.position), self.visit(node.offset)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
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

    def visit_InSpecifier(self, node: s.InSpecifier):
        return ast.Call(
            func=ast.Name(id="In", ctx=loadCtx),
            args=[self.visit(node.region)],
            keywords=[],
        )

    def visit_FollowingSpecifier(self, node: s.FollowingSpecifier):
        return ast.Call(
            func=ast.Name(id="Following", ctx=loadCtx),
            args=[self.visit(node.field), self.visit(node.distance)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )

    def visit_FacingSpecifier(self, node: s.FacingSpecifier):
        return ast.Call(
            func=ast.Name(id="Facing", ctx=loadCtx),
            args=[self.visit(node.heading)],
            keywords=[],
        )

    def visit_FacingTowardSpecifier(self, node: s.FacingTowardSpecifier):
        return ast.Call(
            func=ast.Name(id="FacingToward", ctx=loadCtx),
            args=[self.visit(node.position)],
            keywords=[],
        )

    def visit_ApparentlyFacingSpecifier(self, node: s.ApparentlyFacingSpecifier):
        return ast.Call(
            func=ast.Name(id="ApparentlyFacing", ctx=loadCtx),
            args=[self.visit(node.heading)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )
