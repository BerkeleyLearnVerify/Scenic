
from tests.utils import compileScenic, sampleEgoFrom, sampleParamPFrom

def test_method_of_expr():
    ego = sampleEgoFrom('ego = Object at (12@3).distanceTo(13@3) @ 0')
    assert ego.x == 1
