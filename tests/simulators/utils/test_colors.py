import pytest

from scenic.simulators.utils.colors import Color, NoisyColorDistribution
from tests.utils import sampleEgoFrom


def test_color_class():
    program = """
        from scenic.simulators.utils.colors import Color
        ego = new Object with color Color.defaultCarColor()
        """
    ego = sampleEgoFrom(program)
    assert isinstance(ego.color, Color)


def test_add_noise():
    ant = NoisyColorDistribution.addNoiseTo
    for r in (0, 1, 0.9999999999999999):
        color = (r, 1, 1)
        assert ant(color, 0, 0, 0) == pytest.approx(color)
