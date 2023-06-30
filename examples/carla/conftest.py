import pytest


@pytest.fixture
def options():
    return dict(mode2D=True, params={"render": 0})
