"""Common exceptions and error handling."""

import importlib
import itertools
import pathlib
import traceback
import types
import sys
import warnings

import scenic
import scenic.core
import scenic.syntax

## Configuration

#: Whether or not to elide Scenic's innards from backtraces.
#:
#: Set to True by default so that any errors during import of the scenic module
#: will get full backtraces; the :mod:`scenic` module's *__init__.py* sets it to False.
showInternalBacktrace = True

#: Whether or not to do post-mortem debugging of uncaught exceptions.
postMortemDebugging = False

#: Folders elided from backtraces when :obj:`showInternalBacktrace` is false.
hiddenFolders = [
    pathlib.Path(scenic.core.__file__).parent,      # scenic.core submodules
    pathlib.Path(scenic.syntax.__file__).parent,    # scenic.syntax submodules
    '<frozen importlib._bootstrap>',                # parts of importlib used internally
    pathlib.Path(importlib.__file__).parent,
]

## Exceptions

class ScenicError(Exception):
    """An error produced during Scenic compilation, scene generation, or simulation."""
    pass

class ScenicSyntaxError(ScenicError):
    """An error produced by attempting to parse an invalid Scenic program.

    This is intentionally not a subclass of SyntaxError so that pdb can be used
    for post-mortem debugging of the parser.
    """
    pass

class TokenParseError(ScenicSyntaxError):
    """Parse error occurring during token translation."""
    def __init__(self, tokenOrLine, filename, message):
        self.filename = filename
        self.msg = message
        if hasattr(tokenOrLine, 'start'):
            self.lineno, self.offset = tokenOrLine.start
            self.offset += 1
            self.text = tokenOrLine.line
        else:
            self.lineno = tokenOrLine
            self.text, self.offset = getText(filename, tokenOrLine)
        super().__init__(message)

class PythonParseError(ScenicSyntaxError):
    """Parse error occurring during Python parsing or compilation."""
    def __init__(self, exc):
        self.msg = exc.args[0]
        self.filename, self.lineno = exc.filename, exc.lineno
        self.text, self.offset = getText(self.filename, self.lineno, exc.text, exc.offset)
        super().__init__(self.msg)
        self.with_traceback(exc.__traceback__)

class ASTParseError(ScenicSyntaxError):
    """Parse error occuring during modification of the Python AST."""
    def __init__(self, node, message, filename):
        self.msg = message
        self.lineno = node.lineno
        self.filename = filename
        self.text, self.offset = getText(filename, node.lineno, offset=node.col_offset)
        super().__init__(message)

class RuntimeParseError(ScenicSyntaxError):
    """A Scenic parse error generated during execution of the translated Python."""
    def __init__(self, msg, loc=None):
        super().__init__(msg)
        self.loc = loc

class InvalidScenarioError(ScenicError):
    """Error raised for syntactically-valid but otherwise problematic Scenic programs."""
    pass

class InconsistentScenarioError(InvalidScenarioError):
    """Error for scenarios with inconsistent requirements."""
    def __init__(self, line, message):
        self.lineno = line
        super().__init__('Inconsistent requirement on line ' + str(line) + ': ' + message)

## Scenic backtraces

def excepthook(ty, value, tb):
    if showInternalBacktrace:
        strings = ['Traceback (most recent call last):\n']
    else:
        strings = ['Traceback (most recent call last; use -b to show Scenic internals):\n']

    # Work out how to present the exception type
    pseudoSyntaxError = (issubclass(ty, ScenicSyntaxError)
                         and not issubclass(ty, RuntimeParseError))
    if issubclass(ty, ScenicError):
        if issubclass(ty, ScenicSyntaxError) and not showInternalBacktrace:
            name = 'ScenicSyntaxError'
        else:
            name = ty.__name__
        if pseudoSyntaxError:
            # hack to get format_exception_only to format this like a bona fide SyntaxError
            bases = (SyntaxError,)
        else:
            bases = ty.__bases__
        formatTy = type(name, bases, {})
        if not showInternalBacktrace:
            formatTy.__module__ = '__main__'    # hide qualified name of the exception
    else:
        formatTy = ty

    if issubclass(ty, SyntaxError) or (pseudoSyntaxError and not showInternalBacktrace):
        pass    # no backtrace for these types of errors
    elif issubclass(ty, RuntimeParseError) and value.loc and not showInternalBacktrace:
        strings.extend(traceback.format_list([value.loc]))
    else:
        summary = traceback.extract_tb(tb)
        if showInternalBacktrace:
            filtered = summary
        else:
            filtered = []
            skip = False
            for frame in summary:
                if skip:
                    skip = False
                    continue
                if frame.name == 'callBeginningScenicTrace':
                    filtered = []
                    skip = True
                elif includeFrame(frame):
                    filtered.append(frame)
        strings.extend(traceback.format_list(filtered))
    strings.extend(traceback.format_exception_only(formatTy, value))
    message = ''.join(strings)
    print(message, end='', file=sys.stderr)

    if postMortemDebugging:
        print('Uncaught exception. Entering post-mortem debugging')
        import pdb
        pdb.post_mortem(tb)

def includeFrame(frame):
    if frame.filename in hiddenFolders:
        return False
    parents = pathlib.Path(frame.filename).parents
    return not any(folder in parents for folder in hiddenFolders)

if sys.excepthook is not sys.__excepthook__:
    warnings.warn('unable to install sys.excepthook to format Scenic backtraces')
else:
    sys.excepthook = excepthook

def callBeginningScenicTrace(func):
    """Call the given function, starting the Scenic backtrace at that point.

    This function is just a convenience to make Scenic backtraces cleaner when
    running Scenic programs from the command line.
    """
    return func()

def saveErrorLocation():
    stack = traceback.extract_stack()
    for frame in reversed(stack):
        if includeFrame(frame):
            return frame
    return None

## Utilities

def getText(filename, lineno, line='', offset=0):
    try:    # attempt to recover line from original file
        with open(filename, 'r') as f:
            line = list(itertools.islice(f, lineno-1, lineno))
        line = line[0] if line else ''
        offset = min(offset, len(line))     # TODO improve?
    except FileNotFoundError:
        pass
    return line, offset
