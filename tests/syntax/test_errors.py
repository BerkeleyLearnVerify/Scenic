
import sys
import os.path

import pytest

import scenic
from scenic import scenarioFromString as compileScenic
from scenic.syntax.translator import (ParseError, TokenParseError,
    PythonParseError, ASTParseError, InterpreterParseError)

### File errors

def test_missing_file():
    with pytest.raises(FileNotFoundError):
        scenic.scenarioFromFile('____baloney-file-2342905_.sc')

def test_bad_extension(tmpdir):
    path = os.path.join(tmpdir, 'blah.py')
    with pytest.raises(RuntimeError), open(path, 'w'):
        scenic.scenarioFromFile(path)

### Parse errors during token translation

## Operators used in Python but not Scenic

def test_illegal_operators():
    # Unary operators
    with pytest.raises(TokenParseError):
        compileScenic('x = ~2')
    # Binary operators
    binops = ('<<', '>>', '|', '&', '^', '//')
    for op in binops:
        try:
            compileScenic(f'x = 3 {op} 2')
            pytest.fail(f'Illegal operator {op} did not raise ParseError')
        except TokenParseError:
            pass
    # Augmented assignments
    for op in binops:
        try:
            compileScenic(f'x {op}= 4')
            pytest.fail(f'Illegal augmented assigment {op}= did not raise ParseError')
        except TokenParseError:
            pass

## Constructor definitions

badNames = ('', '3', '+', 'Range')

def test_illegal_constructor_name():
    for name in badNames:
        with pytest.raises(TokenParseError):
            compileScenic(f'constructor {name}:\n' '    pass')

def test_illegal_constructor_superclass():
    for name in badNames:
        with pytest.raises(TokenParseError):
            compileScenic(f'constructor Foo({name}):\n' '    pass')

def test_constructor_python_superclass():
    with pytest.raises(TokenParseError):
        compileScenic('constructor Foo(object):\n' '    pass')

def test_constructor_undefined_superclass():
    with pytest.raises(TokenParseError):
        compileScenic('constructor Foo(Bar):\n' '    pass')

def test_malformed_constructor():
    with pytest.raises(TokenParseError):
        compileScenic('constructor Foo\n' '    pass')
    with pytest.raises(TokenParseError):
        compileScenic('constructor Foo(Bar:\n' '    pass')

## Soft requirements

def test_malformed_soft_requirement():
    with pytest.raises(TokenParseError):
        compileScenic('require[x] 3 == 3')
    with pytest.raises(TokenParseError):
        compileScenic('require[1+x] 3 == 3')
    with pytest.raises(TokenParseError):
        compileScenic('require[] 3 == 3')

## Specifiers

def test_undefined_specifier():
    with pytest.raises(TokenParseError):
        compileScenic('Object cattywampus')
    with pytest.raises(TokenParseError):
        compileScenic('Object athwart 3')

## Illegal usages of keywords

def test_reserved_functions():
    with pytest.raises(TokenParseError):
        compileScenic('Range(4, 6)')
    with pytest.raises(TokenParseError):
        compileScenic('PropertyDefault()')

## Unmatched parentheses and multiline strings

def test_unmatched_parentheses():
    with pytest.raises(TokenParseError):
        compileScenic('(')
    with pytest.raises(TokenParseError):
        compileScenic('x = (3 + 4')
    with pytest.raises(TokenParseError):
        compileScenic(')')
    with pytest.raises(TokenParseError):
        compileScenic('x = (4 - 2))')

def test_incomplete_multiline_string():
    with pytest.raises(TokenParseError):
        compileScenic('"""foobar')
    with pytest.raises(TokenParseError):
        compileScenic('x = """foobar\n' 'wog\n')

### Line numbering

def programsWithBug(bug):
    """Some example programs with a hole to place a buggy statement.
    The hole should be reached during execution, and not be inside a loop."""
    yield bug, 1
    yield 'y = 12\n' + bug, 2
    yield '\n' 'import time\n' + bug, 3
    yield f"""\
from time import time

def qux(foo=None,
        bar=3):
    x = 1 + 2 \\
        + 3
    {bug}

qux()
""", 7

def checkBugs(bugs, checkTraceback=True):
    for bug in bugs:
        for program, line in programsWithBug(bug):
            try:
                compileScenic(program)
                print(f'FAILING PROGRAM:\n{program}')
                pytest.fail(f'Program with buggy statement "{bug}" did not raise error')
            except Exception as e:
                if isinstance(e, (ParseError, SyntaxError)):
                    assert e.lineno == line, program
                # in Python 3.7+, when we can modify tracebacks, check that the
                # last frame of the traceback has the correct line number
                if checkTraceback and sys.version_info >= (3, 7):
                    tb = sys.exc_info()[2]
                    while tb.tb_next is not None:
                        tb = tb.tb_next
                    assert tb.tb_lineno == line, f'\n{program}'

def test_line_numbering_early():
    """Line numbering for parse errors too early to have a full traceback."""
    bugs = (
        'x = 3 << 2',       # caught during token translation
        '4 = 2',            # caught during Python parsing
        'require',          # caught during AST surgery
        'break',            # caught during Python compilation
    )
    checkBugs(bugs, checkTraceback=False)

def test_line_numbering_late():
    """Line numbering for parse errors and exceptions with a full traceback."""
    bugs = (
        'mutate 4',         # caught during Python execution (Scenic parse error)
        'x = _flub__',      # caught during Python execution (Python runtime error)
        'raise Exception',  # caught during Python execution (program exception)
    )
    checkBugs(bugs)

def test_line_numbering_double():
    """Line numbering for errors arising in reused syntax elements."""
    bugs = (
        'x = float(0@0)\n' 'y = 1@2',
        'x = float((0, 10))\n' 'y = (0, 10)',
    )
    checkBugs(bugs)
