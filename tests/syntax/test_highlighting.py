"""Tests for the Scenic and Python syntax highlighters.

The highlighters will get a thorough workout in :file:`test_docs.py` when
they're applied to the entire Scenic codebase. But building the documentation
is slow, so we provide quicker tests here. The :file:`polychrome.scenic` file
also is convenient for seeing all the highlighting possibilities in one place,
and includes a few oddball Python syntaxes that don't occur anywhere in Scenic
(at the time of writing).
"""

import ast
import os.path
import sys

import pytest

pygments = pytest.importorskip("pygments")
import pygments.formatters
import pygments.lexers

import scenic.syntax.parser as parser
from scenic.syntax.pygment import BetterPythonLexer, ScenicLexer

testFiles = (
    "polychrome.py",
    "polychrome.scenic",
)


@pytest.mark.parametrize("name", testFiles)
@pytest.mark.skipif(sys.version_info < (3, 10), reason="need match statement")
def test_sanity(request, name):
    """Make sure our test files are actually syntactically valid."""
    base = os.path.dirname(request.fspath)
    path = os.path.join(base, name)
    with open(path, "r") as f:
        source = f.read()
    if name.endswith(".py"):
        ast.parse(source)
    elif name.endswith(".scenic"):
        parser.parse_string(source, "exec", filename=path)
    else:
        pytest.fail("unexpected test file extension")


@pytest.mark.parametrize("name", testFiles)
@pytest.mark.skipif(sys.version_info < (3, 10), reason="need match statement")
def test_lexer(request, name):
    base = os.path.dirname(request.fspath)
    path = os.path.join(base, name)
    with open(path, "r") as f:
        source = f.read()
    if name.endswith(".py"):
        lexer = BetterPythonLexer()
    elif name.endswith(".scenic"):
        lexer = ScenicLexer()
    else:
        pytest.fail("unexpected test file extension")
    lexer.add_filter("raiseonerror")
    tokens = list(pygments.lex(source, lexer))
    # Check formatting too in case a weird token type somehow crashes the formatters
    for alias in ("html", "latex"):
        formatter = pygments.formatters.get_formatter_by_name(alias)
        pygments.format(tokens, formatter)


def test_lexer_plugin_name():
    lexer = pygments.lexers.get_lexer_by_name("scenic")
    assert isinstance(lexer, ScenicLexer)


def test_lexer_plugin_filename():
    lexer = pygments.lexers.get_lexer_for_filename("foo.scenic")
    assert isinstance(lexer, ScenicLexer)
