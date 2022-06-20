import ast
import pytest

from .helper import parse_string_helper
import scenic.syntax.ast as s

class TestEgoAssign:
    def test_basic(self):
        mod = parse_string_helper("ego = 10")
        stmt = mod.body[0]
        match stmt:
            case s.EgoAssign(value=ast.Constant(value=v)):
                assert v == 10
            case _:
                assert False
    
    def test_with_new(self):
        mod = parse_string_helper("ego = new Object")
        stmt = mod.body[0]
        match stmt:
            case s.EgoAssign(value=s.New(className=c)):
                assert c == "Object"
            case _:
                assert False

    def test_ego_keyword(self):
        """Ego is a hard keyword and cannot be used except for ego assignment"""
        with pytest.raises(SyntaxError):
            parse_string_helper("ego, x = 10, 20")
