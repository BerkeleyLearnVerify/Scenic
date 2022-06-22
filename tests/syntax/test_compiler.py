from ast import *

import pytest
from scenic.syntax.ast import *

from scenic.syntax.compiler import compileScenicAST


class TestCompiler:
    # Special Case
    def test_ego_assign(self):
        node, _ = compileScenicAST(EgoAssign(Constant(1)))
        match node:
            case Expr(Call(Name("ego"), [Constant(1)])):
                assert True
            case _:
                assert False

    # Instance & Specifiers
    def test_new_no_specifiers(self):
        node, _ = compileScenicAST(New("Object", []))
        match node:
            case Call(Name("Object")):
                assert True
            case _:
                assert False

    def test_with_specifier(self):
        node, _ = compileScenicAST(WithSpecifier("foo", Constant(1)))
        match node:
            case Call(Name("With"), [Constant(prop), Constant(value)]):
                assert prop == "foo"
                assert value == 1
            case _:
                assert False

    def test_at_specifier(self):
        node, _ = compileScenicAST(AtSpecifier(Name("x")))
        match node:
            case Call(Name("At"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_offset_by_specifier(self):
        node, _ = compileScenicAST(OffsetBySpecifier(Name("x")))
        match node:
            case Call(Name("OffsetBy"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_offset_along_specifier(self):
        node, _ = compileScenicAST(
            OffsetAlongSpecifier(Name("direction"), Name("offset"))
        )
        match node:
            case Call(Name("OffsetAlongSpec"), [Name("direction"), Name("offset")]):
                assert True
            case _:
                assert False

    def test_position_specifier_left(self):
        node, _ = compileScenicAST(PositionSpecifier(LeftOf(), Name("x"), None))
        match node:
            case Call(Name("LeftSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_right(self):
        node, _ = compileScenicAST(PositionSpecifier(RightOf(), Name("x"), None))
        match node:
            case Call(Name("RightSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_ahead(self):
        node, _ = compileScenicAST(PositionSpecifier(AheadOf(), Name("x"), None))
        match node:
            case Call(Name("Ahead"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_behind(self):
        node, _ = compileScenicAST(PositionSpecifier(Behind(), Name("x"), None))
        match node:
            case Call(Name("Behind"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_distance(self):
        node, _ = compileScenicAST(PositionSpecifier(Behind(), Name("x"), Constant(10)))
        match node:
            case Call(Name("Behind"), [Name("x")], [keyword("dist", Constant(10))]):
                assert True
            case _:
                assert False

    def test_position_specifier_unknown_direction(self):
        with pytest.raises(TypeError) as excinfo:
            compileScenicAST(PositionSpecifier(str(), Name("x"), None))
        assert '"str" cannot be used as a direction' in str(excinfo.value)

    def test_visible_specifier(self):
        node, _ = compileScenicAST(VisibleSpecifier())
        match node:
            case Call(Name("VisibleSpec")):
                assert True
            case _:
                assert False

    def test_visible_specifier_with_base(self):
        node, _ = compileScenicAST(VisibleSpecifier(Name("x")))
        match node:
            case Call(Name("VisibleFrom"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_in_specifier(self):
        node, _ = compileScenicAST(InSpecifier(Name("region")))
        match node:
            case Call(Name("In"), [Name("region")]):
                assert True
            case _:
                assert False

    def test_following_specifier(self):
        node, _ = compileScenicAST(FollowingSpecifier(Name("field"), Name("distance")))
        match node:
            case Call(Name("Following"), [Name("field"), Name("distance")]):
                assert True
            case _:
                assert False

    def test_following_specifier_from(self):
        node, _ = compileScenicAST(
            FollowingSpecifier(Name("field"), Name("distance"), Name("base"))
        )
        match node:
            case Call(
                Name("Following"),
                [Name("field"), Name("distance")],
                [keyword("fromPt", Name("base"))],
            ):
                assert True
            case _:
                assert False

    def test_facing_specifier(self):
        node, _ = compileScenicAST(FacingSpecifier(Name("heading")))
        match node:
            case Call(Name("Facing"), [Name("heading")]):
                assert True
            case _:
                assert False

    def test_facing_toward_specifier(self):
        node, _ = compileScenicAST(FacingTowardSpecifier(Name("position")))
        match node:
            case Call(Name("FacingToward"), [Name("position")]):
                assert True
            case _:
                assert False

    def test_apparently_facing_specifier(self):
        node, _ = compileScenicAST(ApparentlyFacingSpecifier(Name("heading")))
        match node:
            case Call(Name("ApparentlyFacing"), [Name("heading")]):
                assert True
            case _:
                assert False

    def test_apparently_facing_specifier_from(self):
        node, _ = compileScenicAST(
            ApparentlyFacingSpecifier(Name("heading"), Name("base"))
        )
        match node:
            case Call(
                Name("ApparentlyFacing"),
                [Name("heading")],
                [keyword("fromPt", Name("base"))],
            ):
                assert True
            case _:
                assert False
