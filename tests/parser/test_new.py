import ast
import pytest

from helper import parse_string_helper
import scenic.ast as s


@pytest.mark.match
class TestNew:
    def test_basic(self):
        mod = parse_string_helper("new Object")
        stmt = mod.body[0]
        match stmt:
            case ast.Expr(value=s.New(className=c)):
                assert c == "Object"

    # TODO(shun): Add tests for specifiers
