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

@pytest.fixture
def runAndGetP(tmpdir):
    path = os.path.join(tmpdir, 'test.sc')
    return lambda *args, **kwargs: helper(path, *args, **kwargs)

def helper(path, program, options=[], addEgo=True):
    program = inspect.cleandoc(program)
    if addEgo:
        program += '\nego = Object'
    with open(path, 'w') as f:
        f.write(program)
    args = ['scenic', '--show-params', '--gather-stats', '1', path] + options
    result = subprocess.run(args, capture_output=True, text=True)
    assert result.returncode == 0
    lines = result.stdout.splitlines()
    value = None
    for line in lines:
        match = paramPattern.match(line)
        if match:
            assert value is None
            value = match.group(1)
    assert value is not None
    return value

## Tests for command-line options

def test_param(runAndGetP):
    p = runAndGetP('param p = "foo"',
                   options=['--param', 'p', 'bar'])
    assert p == 'bar'

def test_param_int(runAndGetP):
    p = runAndGetP('param p = 42',
                   options=['--param', 'p', '+123'])
    assert p == '123'

def test_param_float(runAndGetP):
    p = runAndGetP('param p = 42',
                   options=['--param', 'p', '123e1'])
    assert p == '1230.0'

def test_seed(runAndGetP):
    p1 = runAndGetP('param p = Range(0, 1)',
                    options=['--seed', '12345'])
    p2 = runAndGetP('param p = Range(0, 1)',
                    options=['--seed', '12345'])
    assert p1 == p2
    p3 = runAndGetP('param p = Range(0, 1)',
                    options=['--seed', '54321'])
    assert p1 != p3
