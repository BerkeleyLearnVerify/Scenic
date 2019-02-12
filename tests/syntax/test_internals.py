
import pytest

from scenic import scenarioFromString as compileScenic
import scenic.syntax.veneer as veneer

def checkVeneerIsInactive():
    assert veneer.activity == 0
    assert len(veneer.allObjects) == 0
    assert veneer.egoObject is None
    assert len(veneer.globalParameters) == 0
    assert len(veneer.pendingRequirements) == 0
    assert len(veneer.inheritedReqs) == 0

def test_veneer_activation():
    checkVeneerIsInactive()
    compileScenic('ego = Object')
    checkVeneerIsInactive()
    with pytest.raises(Exception):
        compileScenic('raise Exception')
    checkVeneerIsInactive()
