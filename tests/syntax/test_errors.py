import os.path
import subprocess
import sys
from tokenize import TokenError

import pytest

import scenic
from scenic.core.errors import ASTParseError, ParseCompileError, ScenicSyntaxError
from tests.utils import compileScenic, sampleActions, sampleActionsFromScene

### File errors


def test_missing_file():
    with pytest.raises(FileNotFoundError):
        scenic.scenarioFromFile("____baloney-file-2342905_.sc")


def test_bad_extension(tmpdir):
    path = os.path.join(tmpdir, "blah.py")
    with pytest.raises(RuntimeError), open(path, "w"):
        scenic.scenarioFromFile(path)


### Parse errors


## Reserved names
def test_reserved_type_names():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("float = 3")

    with pytest.raises(ScenicSyntaxError):
        compileScenic("int = 3")

    with pytest.raises(ScenicSyntaxError):
        compileScenic("str = 3")


## Constructor definitions


def test_illegal_constructor_name():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            f"""
            class 3:
                pass
            """
        )

    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            f"""
            class +:
                pass
            """
        )


def test_illegal_constructor_superclass():
    with pytest.raises(TypeError):
        compileScenic(
            f"""
            class Foo(3):
                pass
            """
        )

    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            f"""
            class Foo(+):
                pass
            """
        )


def test_malformed_constructor():
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            class Foo
                pass
            """
        )
    with pytest.raises(ScenicSyntaxError):
        compileScenic(
            """
            class Foo(Bar:
                pass
            """
        )


def test_new_python_class():
    with pytest.raises(TypeError):
        compileScenic(
            """
            class PyCls(object):
                pass
            new PyCls
            """
        )


## Soft requirements


def test_malformed_soft_requirement():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("require[x] 3 == 3")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("require[1+x] 3 == 3")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("require[] 3 == 3")


## Specifiers


def test_undefined_specifier():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("new Object cattywampus")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("new Object athwart 3")


## Unmatched parentheses and multiline strings


def test_unmatched_parentheses():
    with pytest.raises(TokenError):
        compileScenic("(")
    with pytest.raises(TokenError):
        compileScenic("x = (3 + 4")
    with pytest.raises(ScenicSyntaxError):
        compileScenic(")")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = (4 - 2))")


def test_incomplete_multiline_string():
    with pytest.raises(TokenError):
        compileScenic('"""foobar')
    with pytest.raises(TokenError):
        compileScenic(
            '''
            x = """foobar
            wog
            '''
        )


def test_incomplete_infix_operator():
    """Binary infix operator with too few arguments."""
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = 3 @")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = 3 at")


## Infix operators


def test_incomplete_ternary_operator():
    """3+-ary infix operator with too few arguments."""
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = 4 offset along 12")


def test_extra_ternary_operator():
    """3+-ary infix operator with too many arguments."""
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = 4 at 12 by 17")
    with pytest.raises(ScenicSyntaxError):
        compileScenic("x = 4 offset along 12 by 17 by 19")


def test_invalid_temporal_operator_use():
    """Temporal operators can only be used inside requirements"""
    with pytest.raises(ScenicSyntaxError):
        compileScenic("req = always x")


def test_extra_temporal_operands():
    """Temporal infix operators should only take two arguments"""
    with pytest.raises(ScenicSyntaxError):
        compileScenic("require a or b until c or d until e")


## Ranges


def test_malformed_range():
    with pytest.raises(TypeError):
        compileScenic("x = Range(4,)")
    with pytest.raises(TypeError):
        compileScenic("x = Range(4, 5, 6)")


## Requirements


def test_multiple_requirements():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("require True, True, True")


### Line numbering

# Some example programs with a hole to place a buggy statement.
#
# The hole should be reached during execution, and not be inside a loop.
# If the hole is replaced by an empty string, the program should compile.
# Each entry in the list is a pair, with the first element being the line
# number the error should be reported on.

# fmt: off
templates = [
(1,     # on first line
'''{bug}
ego = new Object'''
),
(2,     # after ordinary statement
'''ego = new Object
{bug}'''
),
(3,     # after import
'''ego = new Object
import time
{bug}'''
),
(8,     # after explicit line continuation (and in a function)
'''from time import time
ego = new Object

def qux(foo=None,
        bar=3):
    x = 1 + 2 \\
        + 3
    {bug}

qux()
'''
),
(4,     # after automatic line continuation by parentheses
'''ego = new Object
x = (1 + 2
     + 3)
{bug}
'''
),
]

indentedSpecTemplate = '''\
ego = new Object facing 1, {continuation}
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
ego = new Object with behavior foo
'''
),
(5,
'''behavior foo():
    try:
        take 5
    interrupt when simulation().currentTime > 0:
        {bug}
        take 10
ego = new Object with behavior foo
'''
),
]
# fmt: on


@pytest.mark.parametrize("template", templates + dynamicTemplates)
def test_bug_template_sanity(template, tmpdir):
    """Check that the program templates above have the correct form."""
    line, program = template
    program = program.format(bug="")
    # Check that they compile, sample, and simulate when not injecting a bug
    print(f"TRYING PROGRAM:\n{program}")
    scenario = compileScenic(program, removeIndentation=False)
    scene, _ = scenario.generate(maxIterations=1)
    sampleActionsFromScene(scene, maxSteps=2)


def checkBug(bug, template, tmpdir, pytestconfig):
    path = os.path.join(tmpdir, "test.scenic")
    line, program = template
    if bug is not None:
        program = program.format(bug=bug)
    print(f"TRYING PROGRAM:\n{program}")
    # write program to file so we can check SyntaxError line correction
    with open(path, "w") as f:
        f.write(program)
    try:
        runFile(path)
        pytest.fail(f'Program with buggy statement "{bug}" did not raise error')
    except Exception as e:
        fast = pytestconfig.getoption("--fast", False)
        if fast:
            lines = None
        else:
            # allow exception to propagate to top level in a subprocess so we can
            # test the formatting of the resulting backtrace
            command = (
                "from tests.syntax.test_errors import runFile;" f'runFile(r"{path}")'
            )
            args = [sys.executable, "-c", command]
            result = subprocess.run(args, capture_output=True, text=True)
            print("RESULTING STDERR:\n", result.stderr)
            assert result.returncode == 1
            lines = result.stderr.splitlines()
            # Filter out any lines that only contain " ", "^", or "~", which indicate location info.
            lines = [l for l in lines if not set(l).issubset({" ", "^", "~"})]
        checkException(e, line, program, bug, path, lines)
        if fast:
            # Mark the test as skipped, since we didn't do all of it
            pytest.skip("slow traceback check skipped by --fast")


def checkException(e, lines, program, bug, path, output, topLevel=True):
    if isinstance(lines, int):
        eLine = lines
        remainingLines = []
    else:
        eLine = lines[0]
        remainingLines = lines[1:]

    # For SyntaxError-like exceptions, check metadata.
    syntaxErrorLike = isinstance(e, (ScenicSyntaxError, SyntaxError))
    if syntaxErrorLike:
        assert e.lineno == eLine, program
        assert e.text.strip() == bug
        # if text does not end with NEWLINE, it is okay to point to the next character after the last
        assert e.offset <= len(e.text) if e.text[-1] == "\n" else len(e.text) + 1

    # If we skipped generating a textual backtrace in a subprocess, stop here.
    if output is None:
        return

    # Check that the general form of the backtrace looks OK.
    assert not any(line.startswith("Error in sys.excepthook") for line in output)
    ty = type(e)
    name = "ScenicSyntaxError" if issubclass(ty, ScenicSyntaxError) else ty.__name__
    assert output[-1].startswith(name)

    # Check that the backtrace lists the correct file and line.
    loc = -3
    assert len(output) >= -loc
    lastFrame = output[loc]
    prefix = f'  File "{path}", line {eLine}'
    if syntaxErrorLike:
        assert lastFrame == prefix
    else:
        assert lastFrame.startswith(prefix + ",")

    # Recurse on chained exceptions, if any.
    chained = bool(e.__cause__ or (e.__context__ and not e.__suppress_context__))
    assert bool(remainingLines) == chained
    if remainingLines:
        if topLevel:
            mid = loc - 6 if sys.version_info >= (3, 13) else loc - 5
        else:
            mid = loc - 2
        assert len(output) >= -(mid - 1)
    if e.__cause__:
        assert (
            output[mid]
            == "The above exception was the direct cause of the following exception:"
        )
        nextE = e.__cause__
    elif e.__context__ and not e.__suppress_context__:
        assert (
            output[mid]
            == "During handling of the above exception, another exception occurred:"
        )
        nextE = e.__context__
    if chained:
        checkException(
            nextE, remainingLines, program, bug, path, output[:mid], topLevel=False
        )


def runFile(path):
    scenario = scenic.scenarioFromFile(path)
    scene, _ = scenario.generate(maxIterations=1)
    sampleActionsFromScene(scene, maxSteps=2)


# fmt: off
@pytest.mark.parametrize('bug', (
    # BUGGY CODE                    ERROR CAUGHT DURING:
    '4 = 2',                        # Scenic parsing
    '3 relative to',                # Scenic parsing
    'new Point at x y',             # Scenic parsing
    'require',                      # Scenic parsing
    'terminate 4',                  # Scenic compilation (handled differently inside behaviors)
    'break',                        # Python compilation
    '4 at 0',                       # Python execution (Scenic parse error)
    'x = _flub__',                  # Python execution (Python runtime error)
    'raise Exception',              # Python execution (program exception)
    'new Object at Uniform("a", 4)',# sampling
    'require 4 at 0',               # requirement evaluation (Scenic parse error)
    'require _flub__',              # requirement evaluation (Python runtime error)
))
# fmt: on
@pytest.mark.parametrize('template', templates + dynamicTemplates)
def test_line_numbering(bug, template, tmpdir, pytestconfig):
    """Line numbering for parse errors."""
    checkBug(bug, template, tmpdir, pytestconfig)

# fmt: off
@pytest.mark.parametrize('bug', (
    'mutate',           # caught during AST surgery
))
# fmt: on
@pytest.mark.parametrize('template', dynamicTemplates)
def test_line_numbering_dynamic(bug, template, tmpdir, pytestconfig):
    """Line numbering for parse errors only occurring in dynamic behaviors."""
    checkBug(bug, template, tmpdir, pytestconfig)

@pytest.mark.parametrize('bug', (
    'x = float(0@0)\n' 'y = 1@2',
    'x = range(Range(0, 10))\n' 'y = Range(0, 10)',
))
@pytest.mark.parametrize('template', templates)
def test_line_numbering_double(bug, template, tmpdir, pytestconfig):
    """Line numbering for errors arising in reused syntax elements."""
    checkBug(bug, template, tmpdir, pytestconfig)

# fmt: off
chainedTemplates = [
((5, 3),
'''ego = new Object
try:
    raise TypeError
except Exception as e:
    {bug}
'''
),
((8, 6, 3),
'''ego = new Object
try:
    raise TypeError
except Exception as e:
    try:
        {bug}
    except Exception:
        raise Exception
'''
),
]
# fmt: on


@pytest.mark.parametrize(
    "bug",
    (
        "raise ValueError",
        "raise RuntimeError from e",
    ),
)
@pytest.mark.parametrize("template", chainedTemplates)
def test_line_numbering_chained(bug, template, tmpdir, pytestconfig):
    """Line numbering for chained exceptions."""
    checkBug(bug, template, tmpdir, pytestconfig)
