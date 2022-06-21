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
            case EgoAssign(value=Constant(value=v)):
                assert v == 10
            case _:
                assert False

    def test_with_new(self):
        mod = parse_string_helper("ego = new Object")
        stmt = mod.body[0]
        match stmt:
            case EgoAssign(value=New(className=c)):
                assert c == "Object"
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
            case Expr(value=New(className=c)):
                assert c == "Object"
            case _:
                assert False

    def test_specifier_single(self):
        mod = parse_string_helper("new Object with foo 1")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                value=New(
                    className=c,
                    specifiers=[WithSpecifier(prop=prop, value=Constant(value=value))],
                )
            ):
                assert c == "Object"
                assert prop == "foo"
                assert value == 1
            case _:
                assert False

    def test_specifier_multiple(self):
        mod = parse_string_helper("new Object with foo 1, with bar 2, with baz 3")
        stmt = mod.body[0]
        match stmt:
            case Expr(
                value=New(
                    className="Object",
                    specifiers=[
                        WithSpecifier(prop="foo", value=Constant(value=1)),
                        WithSpecifier(prop="bar", value=Constant(value=2)),
                        WithSpecifier(prop="baz", value=Constant(value=3)),
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
                    className="Object",
                    specifiers=[AtSpecifier(position=Name(position))],
                )
            ):
                assert position == "x"
            case _:
                assert False
