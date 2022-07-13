from ast import *
from typing import Any
from textwrap import dedent

import pytest

from scenic.syntax.ast import *
from scenic.syntax.parser import parse_string


def parse_string_helper(source: str) -> Any:
    "Parse string and return Scenic AST"
    return parse_string(dedent(source), "exec")


class TestTrackedNames:
    def test_ego_assign(self):
        mod = parse_string_helper("ego = 10")
        stmt = mod.body[0]
        match stmt:
            case TrackedAssign(Ego(), Constant(10)):
                assert True
            case _:
                assert False

    def test_ego_assign_with_new(self):
        mod = parse_string_helper("ego = new Object")
        stmt = mod.body[0]
        match stmt:
            case TrackedAssign(Ego(), New("Object")):
                assert True
            case _:
                assert False

    def test_workspace_assign(self):
        mod = parse_string_helper("workspace = Workspace()")
        stmt = mod.body[0]
        match stmt:
            case TrackedAssign(Workspace(), Call(Name("Workspace"))):
                assert True
            case _:
                assert False


class TestClass:
    def test_basic(self):
        mod = parse_string_helper(
            """
            class C:
                pass
            """
        )
        stmt = mod.body[0]
        match stmt:
            case ClassDef(name="C", bases=[], keywords=[]):
                assert True
            case _:
                assert False

    def test_property_def(self):
        mod = parse_string_helper(
            """
            class C:
                property: value
            """
        )
        stmt = mod.body[0]
        match stmt:
            case ClassDef(
                name="C",
                bases=[],
                keywords=[],
                body=[PropertyDef("property", [], Name("value", Load()))],
            ):
                assert True
            case _:
                assert False

    def test_property_def_with_attr(self):
        mod = parse_string_helper(
            """
            class C:
                property[attribute]: value
            """
        )
        stmt = mod.body[0]
        match stmt:
            case ClassDef(
                name="C",
                bases=[],
                keywords=[],
                body=[PropertyDef("property", ["attribute"], Name("value", Load()))],
            ):
                assert True
            case _:
                assert False

    def test_property_def_with_multiple(self):
        mod = parse_string_helper(
            """
            class C:
                property[attribute1, attribute2]: value
            """
        )
        stmt = mod.body[0]
        match stmt:
            case ClassDef(
                name="C",
                bases=[],
                keywords=[],
                body=[
                    PropertyDef(
                        "property",
                        ["attribute1", "attribute2"],
                        Name("value", Load()),
                    )
                ],
            ):
                assert True
            case _:
                assert False

    def test_property_def_nested(self):
        # property definition is allowed only on the top level
        mod = parse_string_helper(
            """
            class C:
                if True:
                    property: value
            """
        )
        stmt = mod.body[0]
        match stmt:
            case ClassDef(
                name="C",
                bases=[],
                keywords=[],
                body=[
                    If(
                        Constant(True),
                        [
                            AnnAssign(
                                Name("property", Store()), Name("value", Load()), None
                            )
                        ],
                    )
                ],
            ):
                assert True
            case _:
                assert False


class TestModel:
    def test_basic(self):
        mod = parse_string_helper("model some_model")
        stmt = mod.body[0]
        match stmt:
            case Model("some_model"):
                assert True
            case _:
                assert False

    def test_dotted(self):
        mod = parse_string_helper("model scenic.simulators.carla.model")
        stmt = mod.body[0]
        match stmt:
            case Model("scenic.simulators.carla.model"):
                assert True
            case _:
                assert False


class TestMutate:
    def test_basic(self):
        mod = parse_string_helper("mutate x")
        stmt = mod.body[0]
        match stmt:
            case Mutate([Name("x", Load())]):
                assert True
            case _:
                assert False

    def test_multiple(self):
        mod = parse_string_helper("mutate x, y, z")
        stmt = mod.body[0]
        match stmt:
            case Mutate([Name("x", Load()), Name("y", Load()), Name("z", Load())]):
                assert True
            case _:
                assert False

    def test_ego(self):
        mod = parse_string_helper("mutate ego")
        stmt = mod.body[0]
        match stmt:
            case Mutate([Name("ego", Load())]):
                assert True
            case _:
                assert False

    def test_empty(self):
        mod = parse_string_helper("mutate")
        stmt = mod.body[0]
        match stmt:
            case Mutate([]):
                assert True
            case _:
                assert False


class TestParam:
    def test_basic(self):
        mod = parse_string_helper("param i = v")
        stmt = mod.body[0]
        match stmt:
            case Param([parameter("i", Name("v"))]):
                assert True
            case _:
                assert False

    def test_quoted(self):
        mod = parse_string_helper("param 'i' = v")
        stmt = mod.body[0]
        match stmt:
            case Param([parameter("i", Name("v"))]):
                assert True
            case _:
                assert False

    def test_multiple(self):
        mod = parse_string_helper('param p1=v1, "p2"=v2, "p1"=v3')
        stmt = mod.body[0]
        match stmt:
            case Param(
                [
                    parameter("p1", Name("v1")),
                    parameter("p2", Name("v2")),
                    parameter("p1", Name("v3")),
                ]
            ):
                assert True
            case _:
                assert False

    def test_empty(self):
        with pytest.raises(SyntaxError):
            parse_string_helper("param")


class TestRequire:
    def test_basic(self):
        mod = parse_string_helper("require X")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), None, None):
                assert True
            case _:
                assert False

    def test_prob(self):
        mod = parse_string_helper("require[0.2] X")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), 0.2, None):
                assert True
            case _:
                assert False

    def test_name(self):
        mod = parse_string_helper("require X as name")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), None, "name"):
                assert True
            case _:
                assert False

    def test_prob_name(self):
        mod = parse_string_helper("require[0.3] X as name")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), 0.3, "name"):
                assert True
            case _:
                assert False

    def test_name_quoted(self):
        mod = parse_string_helper("require X as 'requirement name'")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), None, "requirement name"):
                assert True
            case _:
                assert False

    def test_name_number(self):
        mod = parse_string_helper("require X as 123")
        stmt = mod.body[0]
        match stmt:
            case Require(Name("X"), None, "123"):
                assert True
            case _:
                assert False


class TestNew:
    def test_basic(self):
        mod = parse_string_helper("new Object")
        stmt = mod.body[0]
        match stmt:
            case Expr(New("Object")):
                assert True
            case _:
                assert False

    def test_specifier_single(self):
        mod = parse_string_helper("new Object with foo 1")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [WithSpecifier("foo", Constant(1))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_multiple(self):
        mod = parse_string_helper("new Object with foo 1, with bar 2, with baz 3")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [
                        WithSpecifier("foo", Constant(1)),
                        WithSpecifier("bar", Constant(2)),
                        WithSpecifier("baz", Constant(3)),
                    ],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_at(self):
        mod = parse_string_helper("new Object at x")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [AtSpecifier(Name("x"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_offset_by(self):
        mod = parse_string_helper("new Object offset by x")
        stmt = mod.body[0]
        match stmt:
            case Expr(New("Object", [OffsetBySpecifier(Name("x"))])):
                assert True
            case _:
                assert False

    def test_specifier_offset_along(self):
        mod = parse_string_helper("new Object offset along x by y")
        stmt = mod.body[0]
        match stmt:
            case Expr(New("Object", [OffsetAlongSpecifier(Name("x"), Name("y"))])):
                assert True
            case _:
                assert False

    def test_specifier_position_left(self):
        mod = parse_string_helper("new Object left of left")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [DirectionOfSpecifier(LeftOf(), Name("left"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_right(self):
        mod = parse_string_helper("new Object right of right")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [DirectionOfSpecifier(RightOf(), Name("right"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_ahead(self):
        mod = parse_string_helper("new Object ahead of ahead")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [DirectionOfSpecifier(AheadOf(), Name("ahead"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_behind(self):
        mod = parse_string_helper("new Object behind behind")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [DirectionOfSpecifier(Behind(), Name("behind"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_by(self):
        mod = parse_string_helper("new Object left of left by distance")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [DirectionOfSpecifier(LeftOf(), Name("left"), Name("distance"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_beyond(self):
        mod = parse_string_helper("new Object beyond position by distance")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [BeyondSpecifier(Name("position"), Name("distance"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_beyond_from(self):
        mod = parse_string_helper("new Object beyond position by distance from base")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [BeyondSpecifier(Name("position"), Name("distance"), Name("base"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_visible(self):
        mod = parse_string_helper("new Object visible")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [VisibleSpecifier(None)],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_visible_from(self):
        mod = parse_string_helper("new Object visible from base")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [VisibleSpecifier(Name("base"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_in(self):
        mod = parse_string_helper("new Object in region")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [InSpecifier(Name("region"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_on(self):
        mod = parse_string_helper("new Object on region")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [InSpecifier(Name("region"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_following(self):
        mod = parse_string_helper("new Object following field for distance")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [FollowingSpecifier(Name("field"), Name("distance"), None)],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_following_from(self):
        mod = parse_string_helper("new Object following field from base for distance")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [FollowingSpecifier(Name("field"), Name("distance"), Name("base"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_facing(self):
        mod = parse_string_helper("new Object facing heading")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [FacingSpecifier(Name("heading"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_facing_toward(self):
        mod = parse_string_helper("new Object facing toward position")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [FacingTowardSpecifier(Name("position"))],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_apparently_facing(self):
        mod = parse_string_helper("new Object apparently facing heading")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [ApparentlyFacingSpecifier(Name("heading"), None)],
                )
            ):
                assert True
            case _:
                assert False

    def test_specifier_apparently_facing_from(self):
        mod = parse_string_helper("new Object apparently facing heading from base")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New(
                    "Object",
                    [ApparentlyFacingSpecifier(Name("heading"), Name("base"))],
                )
            ):
                assert True
            case _:
                assert False
