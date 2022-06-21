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

    # TODO(shun): Add tests for specifiers
