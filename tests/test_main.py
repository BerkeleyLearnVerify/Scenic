"""Tests for the 'scenic' command-line tool."""

import inspect
import os
import re
import subprocess

import pytest

# Mark all tests in this file as slow, since they require spawning a subprocess
pytestmark = pytest.mark.slow

## Utilities

paramPattern = re.compile(r'\s*Parameter "p": (.*)$')
recordPattern = re.compile(r'\s*Record "r": (.*)$')


@pytest.fixture
def runAndGetP(tmpdir):
    path = os.path.join(tmpdir, "test.sc")
    return lambda *args, **kwargs: helper(path, *args, **kwargs)


@pytest.fixture
def runAndGetRecordR(tmpdir):
    path = os.path.join(tmpdir, "test.sc")
    return lambda *args, **kwargs: simHelper(path, *args, **kwargs)


def helper(path, program, options=[], addEgo=True):
    footer = "ego = new Object" if addEgo else ""
    opts = ["--show-params", "--gather-stats", "1"] + options
    lines = run(path, program, opts, addEgo, footer=footer)
    return extractValue(lines, paramPattern)


def simHelper(path, program, options=[]):
    header = "import scenic\nsimulator scenic.core.simulators.DummySimulator()"
    opts = ["-S", "--count", "1", "--show-records"] + options
    lines = run(path, program, opts, header=header)
    return extractValue(lines, recordPattern)


def run(path, program, options, header="", footer=""):
    program = inspect.cleandoc(program)
    program = f"{header}\n{program}\n{footer}"
    with open(path, "w") as f:
        f.write(program)
    args = ["scenic", path] + options
    result = subprocess.run(args, capture_output=True, text=True)
    assert result.returncode == 0
    lines = result.stdout.splitlines()
    return lines


def extractValue(lines, pattern):
    value = None
    for line in lines:
        match = pattern.match(line)
        if match:
            assert value is None
            value = match.group(1)
    assert value is not None
    return value


## Tests for command-line options


def test_param(runAndGetP):
    p = runAndGetP('param p = "foo"', options=["--param", "p", "bar"])
    assert p == "bar"


def test_param_int(runAndGetP):
    p = runAndGetP("param p = 42", options=["--param", "p", "+123"])
    assert p == "123"


def test_param_float(runAndGetP):
    p = runAndGetP("param p = 42", options=["--param", "p", "123e1"])
    assert p == "1230.0"


def test_seed(runAndGetP):
    p1 = runAndGetP("param p = Range(0, 1)", options=["--seed", "12345"])
    p2 = runAndGetP("param p = Range(0, 1)", options=["--seed", "12345"])
    assert p1 == p2
    p3 = runAndGetP("param p = Range(0, 1)", options=["--seed", "54321"])
    assert p1 != p3


def test_time(runAndGetRecordR):
    r = runAndGetRecordR(
        """
        i = 0
        behavior Foo():
            global i
            while True:
                i += 2
                wait
        ego = new Object with behavior Foo
        record final i as r
    """,
        options=["--time", "5"],
    )
    assert r == "10"
