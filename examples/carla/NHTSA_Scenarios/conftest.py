import pytest


@pytest.fixture
def options(options):
    pytest.importorskip("verifai")
    return options
