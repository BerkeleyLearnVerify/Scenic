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

def runAndGetP(tmpdir, program, options=[], addEgo=True):
    program = inspect.cleandoc(program)
    if addEgo:
        program += '\nego = Object'
    path = os.path.join(tmpdir, 'test.sc')
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

def test_param(tmpdir):
    p = runAndGetP(tmpdir, 'param p = "foo"',
                   options=['--param', 'p', 'bar'])
    assert p == 'bar'

def test_param_int(tmpdir):
    p = runAndGetP(tmpdir, 'param p = 42',
                   options=['--param', 'p', '+123'])
    assert p == '123'

def test_param_float(tmpdir):
    p = runAndGetP(tmpdir, 'param p = 42',
                   options=['--param', 'p', '123e1'])
    assert p == '1230.0'
