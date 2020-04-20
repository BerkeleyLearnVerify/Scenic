
import pytest

from scenic import scenarioFromString as compileScenic
from tests.utils import checkVeneerIsInactive

def test_veneer_activation():
    checkVeneerIsInactive()
    compileScenic('ego = Object')
    checkVeneerIsInactive()
    with pytest.raises(Exception):
        compileScenic('raise Exception')
    checkVeneerIsInactive()
