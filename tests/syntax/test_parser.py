import pytest
from typing import Any

from ast import *
from scenic.syntax.ast import *

from scenic.syntax.parser import parse_string


def parse_string_helper(source: str) -> Any:
    "Parse string and return Scenic AST"
    return parse_string(source, "exec")


class TestEgoAssign:
    def test_basic(self):
        mod = parse_string_helper("ego = 10")
        stmt = mod.body[0]
        match stmt:
            case EgoAssign(Constant(10)):
                assert True
            case _:
                assert False

    def test_with_new(self):
        mod = parse_string_helper("ego = new Object")
        stmt = mod.body[0]
        match stmt:
            case EgoAssign(New("Object")):
                assert True
            case _:
                assert False

    def test_ego_keyword(self):
        """Ego is a hard keyword and cannot be used except for ego assignment"""
        with pytest.raises(SyntaxError):
            parse_string_helper("ego, x = 10, 20")


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
            case Expr(New("Object", [PositionSpecifier(LeftOf(), Name("left"), None)])):
                assert True
            case _:
                assert False

    def test_specifier_position_right(self):
        mod = parse_string_helper("new Object right of right")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [PositionSpecifier(RightOf(), Name("right"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_ahead(self):
        mod = parse_string_helper("new Object ahead of ahead")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [PositionSpecifier(AheadOf(), Name("ahead"), None)])
            ):
                assert True
            case _:
                assert False

    def test_specifier_position_behind(self):
        mod = parse_string_helper("new Object behind behind")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                New("Object", [PositionSpecifier(Behind(), Name("behind"), None)])
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
                    [PositionSpecifier(LeftOf(), Name("left"), Name("distance"))],
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
