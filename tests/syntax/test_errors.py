
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

@pytest.mark.parametrize('op', ('~',))
def test_illegal_unary_operators(op):
    with pytest.raises(TokenParseError):
        compileScenic(f'x = {op}2')

@pytest.mark.parametrize('op', ('<<', '>>', '|', '&', '^', '//'))
def test_illegal_binary_operators(op):
    # Basic usage
    with pytest.raises(TokenParseError):
        compileScenic(f'x = 3 {op} 2')
    # Augmented assignments
    with pytest.raises(TokenParseError):
        compileScenic(f'x {op}= 4')

## Constructor definitions

badNames = ('', '3', '+', 'Range')

@pytest.mark.parametrize('name', badNames)
def test_illegal_constructor_name(name):
    with pytest.raises(TokenParseError):
        compileScenic(f'constructor {name}:\n' '    pass')

@pytest.mark.parametrize('name', badNames)
def test_illegal_constructor_superclass(name):
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

### Parse errors during Python parsing

def test_incomplete_infix_operator():
    """Binary infix operator with too few arguments."""
    with pytest.raises(PythonParseError):
        compileScenic('x = 3 @')
    with pytest.raises(PythonParseError):
        compileScenic('x = 3 at')

### Parse errors during parse tree surgery

## Infix operators

def test_incomplete_infix_package():
    """Packaged (3+-ary) infix operator with too few arguments."""
    with pytest.raises(ASTParseError):
        compileScenic('x = 4 offset along 12')

def test_extra_infix_package():
    """Packaged (3+-ary) infix operator with too many arguments."""
    with pytest.raises(ASTParseError):
        compileScenic('x = 4 at 12 by 17')
    with pytest.raises(ASTParseError):
        compileScenic('x = 4 offset along 12 by 17 by 19')

## Ranges

def test_malformed_range():
    with pytest.raises(ASTParseError):
        compileScenic('x = (4,)')
    with pytest.raises(ASTParseError):
        compileScenic('x = (4, 5, 6)')

## Requirements

def test_multiple_requirements():
    with pytest.raises(ASTParseError):
        compileScenic('require True, True')
    with pytest.raises(ASTParseError):
        compileScenic('require True, True, True')

### Line numbering

# Some example programs with a hole to place a buggy statement.
#
# The hole should be reached during execution, and not be inside a loop.
# If the hole is replaced by an empty string, the program should compile.
# Each entry in the list is a pair, with the first element being the line
# number the error should be reported on.
templates = [
(1,     # on first line
'''{bug}
ego = Object'''
),
(2,     # after ordinary statement
'''ego = Object
{bug}'''
),
(3,     # after import
'''ego = Object
import time
{bug}'''
),
(8,     # after explicit line continuation (and in a function)
'''from time import time
ego = Object

def qux(foo=None,
        bar=3):
    x = 1 + 2 \\
        + 3
    {bug}

qux()
'''
),
(4,     # after automatic line continuation by parentheses
'''ego = Object
x = (1 + 2
     + 3)
{bug}
'''
),
]

indentedSpecTemplate = '''\
ego = Object facing 1, {continuation}
             at 10@10
{{bug}}
'''
for continuation in ('', '\\', '# comment'):
    templates.append((3, indentedSpecTemplate.format(continuation=continuation)))

@pytest.mark.parametrize('template', templates)
def test_bug_template_sanity(template, tmpdir):
    """Check that the program templates above have the correct form."""
    line, program = template
    program = program.format(bug='')
    # Check that they compile when not injecting a bug
    print(f'TRYING PROGRAM:\n{program}')
    compileScenic(program)

def checkBug(bug, template, tmpdir, checkTraceback=True, generate=False):
    path = os.path.join(tmpdir, 'test.sc')
    line, program = template
    program = program.format(bug=bug)
    print(f'TRYING PROGRAM:\n{program}')
    try:
        # write program to file so we can check SyntaxError line correction
        with open(path, 'w') as f:
            f.write(program)
        scenario = scenic.scenarioFromFile(path)
        if generate:
            scenario.generate(maxIterations=1)
        pytest.fail(f'Program with buggy statement "{bug}" did not raise error')
    except Exception as e:
        if isinstance(e, (ParseError, SyntaxError)):
            assert e.lineno == line, program
            if isinstance(e, SyntaxError):
                assert e.text.strip() == bug
                assert e.offset <= len(e.text)
        # in Python 3.7+, when we can modify tracebacks, check that the
        # last frame of the traceback has the correct line number
        if checkTraceback and sys.version_info >= (3, 7):
            tb = sys.exc_info()[2]
            while tb.tb_next is not None:
                tb = tb.tb_next
            assert tb.tb_lineno == line

@pytest.mark.parametrize('bug', (
    'x = 3 << 2',       # caught during token translation
    '4 = 2',            # caught during Python parsing
    'require mutate',   # caught during Python parsing
    'Point at x y',     # caught during Python parsing (with offset past end of original line)
    'require',          # caught during AST surgery
    'break',            # caught during Python compilation
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_early(bug, template, tmpdir):
    """Line numbering for parse errors too early to have a full traceback."""
    checkBug(bug, template, tmpdir, checkTraceback=False)

@pytest.mark.parametrize('bug', (
    'mutate 4',         # caught during Python execution (Scenic parse error)
    'x = _flub__',      # caught during Python execution (Python runtime error)
    'raise Exception',  # caught during Python execution (program exception)
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_late(bug, template, tmpdir):
    """Line numbering for parse errors and exceptions with a full traceback."""
    checkBug(bug, template, tmpdir)

@pytest.mark.parametrize('bug', (
    'require _flub__',      # Python runtime error
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_generation(bug, template, tmpdir):
    """Line numbering for errors and exceptions occuring scene generation."""
    checkBug(bug, template, tmpdir, generate=True)

@pytest.mark.parametrize('bug', (
    'x = float(0@0)\n' 'y = 1@2',
    'x = float((0, 10))\n' 'y = (0, 10)',
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_double(bug, template, tmpdir):
    """Line numbering for errors arising in reused syntax elements."""
    checkBug(bug, template, tmpdir)
