import pytest

from scenic.simulators.utils.colors import NoisyColorDistribution


def test_add_noise():
    ant = NoisyColorDistribution.addNoiseTo
    for r in (0, 1, 0.9999999999999999):
        color = (r, 1, 1)
        assert ant(color, 0, 0, 0) == pytest.approx(color)
