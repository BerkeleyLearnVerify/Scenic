import pkgutil
import sys

import pytest

from scenic import scenarioFromString as compileScenic
from tests.utils import checkVeneerIsInactive


def test_veneer_activation():
    checkVeneerIsInactive()
    compileScenic("ego = new Object")
    checkVeneerIsInactive()
    with pytest.raises(Exception):
        compileScenic("raise Exception")
    checkVeneerIsInactive()


def test_module_inspection():
    import scenic.syntax.translator as translator

    try:
        translator.buildingDocs = True
        import tests.syntax.helper3

        loader = tests.syntax.helper3.__spec__.loader
        loader.get_code("tests.syntax.helper3")
        loader.get_source("tests.syntax.helper3")
        assert not loader.is_package("tests.syntax.helper3")
    finally:
        translator.buildingDocs = False
        sys.modules.pop("tests.syntax.helper3", None)


def test_iter_modules(request, monkeypatch):
    monkeypatch.chdir(request.path.parent)
    modules = set(info.name for info in pkgutil.iter_modules([""]))
    assert "helper" in modules
    assert "helper2" in modules
