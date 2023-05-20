import warnings
from pathlib import Path

import pytest

import scenic

## Utility Functions ##
def helper_test_example(path, additional_args, deps):
    for dep in deps:
        pytest.importorskip(dep)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        scenario = scenic.scenarioFromFile(path=path, **additional_args)
        scenario.generate(maxIterations=float('inf'))

def get_examples(start_path, top=True):
    # Check if a test.py already covers this directory and its descendents
    if not top and len(set(start_path.glob("test*.py"))) > 0:
        return []

    # Get tests in this directory
    tests = []
    tests += (start_path).glob("*.scenic")

    # Recurse on all sub directories
    for directory in [p for p in start_path.glob("*") if p.is_dir()]:
        tests += get_examples(directory,top=False)

    return tests

# Top Level Test Imports

EXAMPLES = get_examples(Path())
PARAMS = {}
DEPS = []

@pytest.mark.parametrize(
    "path,additional_args,deps",
    [(e,PARAMS,DEPS) for e in EXAMPLES],
)
def test_examples(path, additional_args, deps):
    helper_test_example(path, additional_args, deps)
