
import sys
import os.path
import subprocess

import pytest

import scenic
from scenic.core.errors import (ScenicSyntaxError, TokenParseError,
    PythonParseError, ASTParseError, RuntimeParseError)

from tests.utils import compileScenic, sampleActionsFromScene, sampleActions

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

badNames = ('', '3', '+', 'Behavior')

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
    with pytest.raises(TypeError):
        compileScenic('x = Range(4,)')
    with pytest.raises(TypeError):
        compileScenic('x = Range(4, 5, 6)')

## Requirements

def test_multiple_requirements():
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

dynamicTemplates = [
(3,
'''behavior foo():
    try:
        {bug}
        take 5
    interrupt when ego.position.x > 4:
        take 10
ego = Object with behavior foo
'''
),
(5,
'''behavior foo():
    try:
        take 5
    interrupt when simulation().currentTime > 0:
        {bug}
        take 10
ego = Object with behavior foo
'''
),
]

@pytest.mark.parametrize('template', templates + dynamicTemplates)
def test_bug_template_sanity(template, tmpdir):
    """Check that the program templates above have the correct form."""
    line, program = template
    program = program.format(bug='')
    # Check that they compile, sample, and simulate when not injecting a bug
    print(f'TRYING PROGRAM:\n{program}')
    scenario = compileScenic(program, removeIndentation=False)
    scene, _ = scenario.generate(maxIterations=1)
    sampleActionsFromScene(scene, maxSteps=2)

def checkBug(bug, template, tmpdir, pytestconfig):
    path = os.path.join(tmpdir, 'test.sc')
    line, program = template
    program = program.format(bug=bug)
    print(f'TRYING PROGRAM:\n{program}')
    # write program to file so we can check SyntaxError line correction
    with open(path, 'w') as f:
        f.write(program)
    try:
        runFile(path)
        pytest.fail(f'Program with buggy statement "{bug}" did not raise error')
    except Exception as e:
        syntaxErrorLike = (isinstance(e, (ScenicSyntaxError, SyntaxError))
                           and not isinstance(e, RuntimeParseError))
        if syntaxErrorLike:
            assert e.lineno == line, program
            assert e.text.strip() == bug
            assert e.offset <= len(e.text)

        # allow exception to propagate to top level in a subprocess so we can
        # test the formatting of the resulting backtrace
        fast = pytestconfig.getoption('--fast', False)
        if fast:
            pytest.skip('slow traceback check skipped by --fast')

        command = (
            'from tests.syntax.test_errors import runFile;'
            f'runFile("{path}")'
        )
        args = [sys.executable, '-c', command]
        result = subprocess.run(args, capture_output=True, text=True)
        print('RESULTING STDERR:\n', result.stderr)
        assert result.returncode == 1
        lines = result.stderr.splitlines()
        assert not any(line.startswith('Error in sys.excepthook') for line in lines)
        loc = -4 if syntaxErrorLike else -3
        assert len(lines) >= -loc
        lastFrame = lines[loc]
        prefix = f'  File "{path}", line {line}'
        if syntaxErrorLike:
            assert lastFrame == prefix
        else:
            assert lastFrame.startswith(prefix + ',')

def runFile(path):
    scenario = scenic.scenarioFromFile(path)
    scene, _ = scenario.generate(maxIterations=1)
    sampleActionsFromScene(scene, maxSteps=2)

@pytest.mark.parametrize('bug', (
    # BUGGY CODE                    ERROR CAUGHT DURING:
    'x = 3 << 2',                   # token translation
    '4 = 2',                        # Python parsing
    '3 relative to',                # Python parsing
    'Point at x y',                 # Python parsing (with offset past end of original line)
    'require',                      # AST surgery
    'terminate 4',                  # AST surgery (handled differently inside behaviors)
    'break',                        # Python compilation
    '4 at 0',                       # Python execution (Scenic parse error)
    'x = _flub__',                  # Python execution (Python runtime error)
    'raise Exception',              # Python execution (program exception)
    'Object at Uniform(0@0, 4)'     # sampling
    'require 4 at 0',               # requirement evaluation (Scenic parse error)
    'require _flub__',              # requirement evaluation (Python runtime error)
))
@pytest.mark.parametrize('template', templates + dynamicTemplates)
def test_line_numbering(bug, template, tmpdir, pytestconfig):
    """Line numbering for parse errors."""
    checkBug(bug, template, tmpdir, pytestconfig)

@pytest.mark.parametrize('bug', (
    'mutate',           # caught during AST surgery
))
@pytest.mark.parametrize('template', dynamicTemplates)
def test_line_numbering_dynamic(bug, template, tmpdir, pytestconfig):
    """Line numbering for parse errors only occurring in dynamic behaviors."""
    checkBug(bug, template, tmpdir, pytestconfig)

@pytest.mark.parametrize('bug', (
    'x = float(0@0)\n' 'y = 1@2',
    'x = float(Range(0, 10))\n' 'y = Range(0, 10)',
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_double(bug, template, tmpdir, pytestconfig):
    """Line numbering for errors arising in reused syntax elements."""
    checkBug(bug, template, tmpdir, pytestconfig)
