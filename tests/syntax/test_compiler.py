from ast import *

import pytest

from scenic.syntax.ast import *
from scenic.syntax.compiler import compileScenicAST


class TestCompiler:
    # Special Case
    def test_ego_assign(self):
        node, _ = compileScenicAST(EgoAssign(Constant(1)))
        match node:
            case Expr(Call(Name("ego"), [Constant(1)])):
                assert True
            case _:
                assert False

    def test_scenic_class(self):
        # Because Scenic's class definition is syntactically the same as Python,
        # can use the Python parser here
        node, _ = compileScenicAST(
            parse(
                """
class MyObject(Object):
    property: type
"""
            )
        )
        match node:
            case Module(
                [
                    ClassDef(
                        name="MyObject",
                        bases=[Name("Object")],
                        body=[
                            AnnAssign(
                                target=Name("property"),
                                annotation=Call(
                                    func=Name("PropertyDefault"),
                                    # just checking types here, advanced checks to be done in integration tests
                                    args=[Set(), Set(), Lambda()],
                                ),
                            )
                        ],
                    )
                ]
            ):
                assert True
            case _:
                assert False

    def test_scenic_class_inheritance(self):
        # Because `ScenicClass` is a Scenic class, `ChildScenicClass` is also a Scenic class
        node, _ = compileScenicAST(
            parse(
                """
class ScenicClass(Object): pass
class ChildScenicClass(ScenicClass):
    property: type
        """
            )
        )
        match node.body[1]:
            case ClassDef(
                name="ChildScenicClass",
                bases=[Name("ScenicClass")],
                body=[
                    AnnAssign(
                        target=Name("property"),
                        annotation=Call(
                            func=Name("PropertyDefault"),
                            args=[Set(), Set(), Lambda()],
                        ),
                    )
                ],
            ):
                assert True
            case _:
                assert False

    def test_scenic_class_inheritance_2(self):
        # ChildScenicClass is a Scenic class even if its bases contains a Python class
        node, _ = compileScenicAST(
            parse(
                """
class ScenicClass(Object): pass
class ChildScenicClass(ScenicClass, object):
    property: type
        """
            )
        )
        match node.body[1]:
            case ClassDef(
                name="ChildScenicClass",
                bases=[Name("ScenicClass"), Name("object")],
                body=[
                    AnnAssign(
                        target=Name("property"),
                        annotation=Call(
                            func=Name("PropertyDefault"),
                            args=[Set(), Set(), Lambda()],
                        ),
                    )
                ],
            ):
                assert True
            case _:
                assert False

    def test_scenic_class_default_base(self):
        node, _ = compileScenicAST(parse("class ScenicClass: pass"))
        match node.body[0]:
            case ClassDef(name="ScenicClass", bases=[Name("Object")]):
                assert True
            case _:
                assert False

    def test_python_class(self):
        # Python class should be left as is
        node, _ = compileScenicAST(
            parse(
                """
class PythonClass(object):
    property: type
        """
            )
        )
        match node.body[0]:
            case ClassDef(
                body=[
                    AnnAssign(
                        target=Name("property"),
                        annotation=Name("type"),
                    )
                ],
            ):
                assert True
            case _:
                assert False

    # Simple Statement
    def test_param_basic(self):
        node, _ = compileScenicAST(Param([parameter("p1", Name("v1"))]))
        match node:
            case Expr(Call(Name("param"), [Dict([Constant("p1")], [Name("v1")])])):
                assert True
            case _:
                assert False

    def test_param_multiple(self):
        node, _ = compileScenicAST(
            Param([parameter("p1", Name("v1")), parameter("p2", Constant(1))])
        )
        match node:
            case Expr(
                Call(
                    Name("param"),
                    [Dict([Constant("p1"), Constant("p2")], [Name("v1"), Constant(1)])],
                )
            ):
                assert True
            case _:
                assert False

    def test_param_duplicate(self):
        with pytest.raises(SyntaxError):
            compileScenicAST(
                Param([parameter("p1", Name("v1")), parameter("p1", Constant(1))])
            )

    # Instance & Specifiers
    def test_new_no_specifiers(self):
        node, _ = compileScenicAST(New("Object", []))
        match node:
            case Call(Name("Object")):
                assert True
            case _:
                assert False

    def test_new_one_specifier(self):
        node, _ = compileScenicAST(New("Object", [WithSpecifier("foo", Constant(1))]))
        match node:
            case Call(
                Name("Object"), [Call(Name("With"), [Constant("foo"), Constant(1)])]
            ):
                assert True
            case _:
                assert False

    def test_new_multiple_specifiers(self):
        node, _ = compileScenicAST(
            New(
                "Object",
                [WithSpecifier("foo", Constant(1)), AtSpecifier(Name("position"))],
            )
        )
        match node:
            case Call(
                Name("Object"),
                [
                    Call(Name("With"), [Constant("foo"), Constant(1)]),
                    Call(Name("At"), [Name("position")]),
                ],
            ):
                assert True
            case _:
                assert False

    def test_with_specifier(self):
        node, _ = compileScenicAST(WithSpecifier("foo", Constant(1)))
        match node:
            case Call(Name("With"), [Constant(prop), Constant(value)]):
                assert prop == "foo"
                assert value == 1
            case _:
                assert False

    def test_at_specifier(self):
        node, _ = compileScenicAST(AtSpecifier(Name("x")))
        match node:
            case Call(Name("At"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_offset_by_specifier(self):
        node, _ = compileScenicAST(OffsetBySpecifier(Name("x")))
        match node:
            case Call(Name("OffsetBy"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_offset_along_specifier(self):
        node, _ = compileScenicAST(
            OffsetAlongSpecifier(Name("direction"), Name("offset"))
        )
        match node:
            case Call(Name("OffsetAlongSpec"), [Name("direction"), Name("offset")]):
                assert True
            case _:
                assert False

    def test_position_specifier_left(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(LeftOf(), Name("x"), None))
        match node:
            case Call(Name("LeftSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_right(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(RightOf(), Name("x"), None))
        match node:
            case Call(Name("RightSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_ahead(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(AheadOf(), Name("x"), None))
        match node:
            case Call(Name("Ahead"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_behind(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(Behind(), Name("x"), None))
        match node:
            case Call(Name("Behind"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_distance(self):
        node, _ = compileScenicAST(
            DirectionOfSpecifier(Behind(), Name("x"), Constant(10))
        )
        match node:
            case Call(Name("Behind"), [Name("x")], [keyword("dist", Constant(10))]):
                assert True
            case _:
                assert False

    def test_position_specifier_unknown_direction(self):
        with pytest.raises(AssertionError):
            compileScenicAST(DirectionOfSpecifier(str(), Name("x"), None))

    def test_beyond_specifier(self):
        node, _ = compileScenicAST(BeyondSpecifier(Name("x"), Name("y")))
        match node:
            case Call(Name("Beyond"), [Name("x"), Name("y")]):
                assert True
            case _:
                assert False

    def test_beyond_specifier_with_base(self):
        node, _ = compileScenicAST(BeyondSpecifier(Name("x"), Name("y"), Name("z")))
        match node:
            case Call(
                Name("Beyond"), [Name("x"), Name("y")], [keyword("fromPt", Name("z"))]
            ):
                assert True
            case _:
                assert False

    def test_visible_specifier(self):
        node, _ = compileScenicAST(VisibleSpecifier())
        match node:
            case Call(Name("VisibleSpec")):
                assert True
            case _:
                assert False

    def test_visible_specifier_with_base(self):
        node, _ = compileScenicAST(VisibleSpecifier(Name("x")))
        match node:
            case Call(Name("VisibleFrom"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_in_specifier(self):
        node, _ = compileScenicAST(InSpecifier(Name("region")))
        match node:
            case Call(Name("In"), [Name("region")]):
                assert True
            case _:
                assert False

    def test_following_specifier(self):
        node, _ = compileScenicAST(FollowingSpecifier(Name("field"), Name("distance")))
        match node:
            case Call(Name("Following"), [Name("field"), Name("distance")]):
                assert True
            case _:
                assert False

    def test_following_specifier_from(self):
        node, _ = compileScenicAST(
            FollowingSpecifier(Name("field"), Name("distance"), Name("base"))
        )
        match node:
            case Call(
                Name("Following"),
                [Name("field"), Name("distance")],
                [keyword("fromPt", Name("base"))],
            ):
                assert True
            case _:
                assert False

    def test_facing_specifier(self):
        node, _ = compileScenicAST(FacingSpecifier(Name("heading")))
        match node:
            case Call(Name("Facing"), [Name("heading")]):
                assert True
            case _:
                assert False

    def test_facing_toward_specifier(self):
        node, _ = compileScenicAST(FacingTowardSpecifier(Name("position")))
        match node:
            case Call(Name("FacingToward"), [Name("position")]):
                assert True
            case _:
                assert False

    def test_apparently_facing_specifier(self):
        node, _ = compileScenicAST(ApparentlyFacingSpecifier(Name("heading")))
        match node:
            case Call(Name("ApparentlyFacing"), [Name("heading")]):
                assert True
            case _:
                assert False

    def test_apparently_facing_specifier_from(self):
        node, _ = compileScenicAST(
            ApparentlyFacingSpecifier(Name("heading"), Name("base"))
        )
        match node:
            case Call(
                Name("ApparentlyFacing"),
                [Name("heading")],
                [keyword("fromPt", Name("base"))],
            ):
                assert True
            case _:
                assert False
