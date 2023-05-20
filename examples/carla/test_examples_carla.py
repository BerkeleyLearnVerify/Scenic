import warnings
from pathlib import Path

import pytest

import scenic

from examples.test_examples import helper_test_example, get_examples

# Test Imports
EXAMPLES = get_examples(Path(scenic.syntax.veneer.localPath(".")))
PARAMS = {"mode2D": True, "params": {"render": 0}}
DEPS = []

@pytest.mark.parametrize(
    "path,additional_args,deps",
    [(e,PARAMS,DEPS) for e in EXAMPLES],
)
def test_examples(path, additional_args, deps):
	helper_test_example(path, additional_args, deps)
