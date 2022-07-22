from ast import *

import pytest

from scenic.syntax.ast import *
from scenic.syntax.compiler import compileScenicAST


class TestCompiler:
    # Special Case
    def test_ego_assign(self):
        node, _ = compileScenicAST(TrackedAssign(Ego(), Constant(1)))
        match node:
            case Expr(Call(Name("ego"), [Constant(1)])):
                assert True
            case _:
                assert False

    def test_workspace_assign(self):
        node, _ = compileScenicAST(TrackedAssign(Workspace(), Constant(1)))
        match node:
            case Expr(Call(Name("workspace"), [Constant(1)])):
                assert True
            case _:
                assert False

    def test_ego_reference(self):
        node, _ = compileScenicAST(Name("ego", Load()))
        match node:
            case Call(Name("ego")):
                assert True
            case _:
                assert False

    def test_workspace_reference(self):
        node, _ = compileScenicAST(Name("workspace", Load()))
        match node:
            case Call(Name("workspace")):
                assert True
            case _:
                assert False

    def test_builtin_name(self):
        with pytest.raises(SyntaxError):
            compileScenicAST(Assign([Name("globalParameters", Store())], Constant(1)))

    def test_tracked_name_assign(self):
        # simple assign will be converted to `TrackedAssign` by the parser,
        # but it is still possible to get a `Name` node with one of the tracked names
        # e.g. multiple assignment: ego = myEgo = <some value>
        # if a tracked name is used in the Store context, that is an syntax error
        with pytest.raises(SyntaxError):
            compileScenicAST(Assign([Name("workspace", Store())], Constant(1)))

    def test_classdef(self):
        # Object is specified as a base
        # `_scenic_properties` is initialized at the beginning of class body
        node, _ = compileScenicAST(ClassDef("C", [], [], [], []))
        match node:
            case ClassDef(
                name="C",
                bases=[Name("Object", Load())],
                body=[
                    Assign(
                        targets=[Name("_scenic_properties", Store())],
                        value=Dict([], []),
                    )
                ],
            ):
                assert True
            case _:
                assert False

    def test_classdef_explicit_base(self):
        node, _ = compileScenicAST(ClassDef("C", [Name("MyBase", Load())], [], [], []))
        match node:
            case ClassDef(
                name="C",
                bases=[Name("MyBase", Load())],
            ):
                assert True
            case _:
                assert False

    def test_classdef_properties(self):
        node, _ = compileScenicAST(
            ClassDef(
                "C",
                [],
                [],
                [
                    PropertyDef("property1", [], Constant(1)),
                    PropertyDef("property2", [], Constant(2)),
                    Expr(Constant("hello")),
                ],
                [],
            )
        )
        match node:
            case ClassDef(
                name="C",
                body=[
                    Assign(
                        targets=[Name("_scenic_properties", Store())],
                        value=Dict(
                            [Constant("property1"), Constant("property2")],
                            [
                                Call(
                                    func=Name(id="PropertyDefault", ctx=Load()),
                                    args=[
                                        Set([]),
                                        Set([]),
                                        Lambda(body=Constant(1)),
                                    ],
                                    keywords=[],
                                ),
                                Call(
                                    func=Name(id="PropertyDefault", ctx=Load()),
                                    args=[
                                        Set([]),
                                        Set([]),
                                        Lambda(body=Constant(2)),
                                    ],
                                    keywords=[],
                                ),
                            ],
                        ),
                    ),
                    Expr(Constant("hello")),
                ],
            ):
                assert True
            case _:
                assert False

    def test_classdef_properties2(self):
        node, _ = compileScenicAST(
            ClassDef(
                "C",
                [],
                [],
                [
                    PropertyDef(
                        "property",
                        [Dynamic()],
                        Attribute(
                            value=Name(id="self", ctx=Load()), attr="x", ctx=Load()
                        ),
                    ),
                ],
                [],
            )
        )
        match node:
            case ClassDef(
                name="C",
                body=[
                    Assign(
                        targets=[Name("_scenic_properties", Store())],
                        value=Dict(
                            [Constant("property")],
                            [
                                Call(
                                    func=Name(id="PropertyDefault", ctx=Load()),
                                    args=[
                                        Set([Constant("x")]),  # includes `x`
                                        Set([Constant("dynamic")]),
                                        Lambda(
                                            body=Attribute(
                                                value=Name(id="self", ctx=Load()),
                                                attr="x",
                                                ctx=Load(),
                                            )
                                        ),
                                    ],
                                    keywords=[],
                                ),
                            ],
                        ),
                    ),
                ],
            ):
                assert True
            case _:
                assert False

    def test_classdef_duplicated_property(self):
        with pytest.raises(SyntaxError):
            compileScenicAST(
                ClassDef(
                    "C",
                    [],
                    [],
                    [
                        PropertyDef(
                            "property",
                            [],
                            Constant(1),
                        ),
                        PropertyDef(
                            "property",
                            [],
                            Constant(2),
                        ),
                    ],
                    [],
                )
            )

    # Simple Statement
    def test_model_basic(self):
        node, _ = compileScenicAST(Model("model"))
        match node:
            case Expr(
                Call(
                    Name("model"), [Name("_Scenic_module_namespace"), Constant("model")]
                )
            ):
                assert True
            case _:
                assert False

    def test_mutate_basic(self):
        node, _ = compileScenicAST(Mutate([Name("x", Load())]))
        match node:
            case Expr(Call(Name("mutate"), [Name("x")])):
                assert True
            case _:
                assert False

    def test_mutate_multiple(self):
        node, _ = compileScenicAST(
            Mutate([Name("x", Load()), Name("y", Load()), Name("z", Load())])
        )
        match node:
            case Expr(Call(Name("mutate"), [Name("x"), Name("y"), Name("z")])):
                assert True
            case _:
                assert False

    def test_mutate_ego(self):
        node, _ = compileScenicAST(Mutate([Name("ego", Load())]))
        match node:
            case Expr(Call(Name("mutate"), [Call(Name("ego"))])):
                assert True
            case _:
                assert False

    def test_mutate_empty(self):
        node, _ = compileScenicAST(Mutate([]))
        match node:
            case Expr(Call(Name("mutate"), [])):
                assert True
            case _:
                assert False

    def test_param_basic(self):
        node, _ = compileScenicAST(Param([parameter("p1", Name("v1"))]))
        match node:
            case Expr(Call(Name("param"), [Dict([Constant("p1")], [Name("v1")])])):
                assert True
            case _:
                assert False

    def test_param_multiple(self):
        node, _ = compileScenicAST(
            Param([parameter("p1", Name("v1")), parameter("p2", Constant(1))])
        )
        match node:
            case Expr(
                Call(
                    Name("param"),
                    [Dict([Constant("p1"), Constant("p2")], [Name("v1"), Constant(1)])],
                )
            ):
                assert True
            case _:
                assert False

    def test_param_duplicate(self):
        with pytest.raises(SyntaxError):
            compileScenicAST(
                Param([parameter("p1", Name("v1")), parameter("p1", Constant(1))])
            )

    @pytest.mark.parametrize(
        "options, expected_name, expected_prob",
        [
            # just requirement
            ({}, None, 1.0),
            # with probability
            ({"prob": 0.2}, None, 0.2),
            # with name
            ({"name": "req_name"}, "req_name", 1.0),
            # with probability and name
            (
                {"prob": 0.5, "name": "req_name"},
                "req_name",
                0.5,
            ),
        ],
    )
    def test_require(self, options, expected_name, expected_prob):
        node, requirements = compileScenicAST(Require(Name("C"), lineno=2, **options))
        match node:
            case Expr(
                Call(
                    Name("require"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # requirement
                        Constant(2),  # lineno
                        Constant(name),  # name
                    ],
                    kwargs,  # prob or empty list
                )
            ):
                assert name is None if expected_name is None else name == expected_name
                compiled_prob = (
                    kwargs[0].value if kwargs else 1.0
                )  # if kwargs is empty, use 1.0
                assert compiled_prob == expected_prob
            case _:
                assert False

        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    # Instance & Specifiers
    def test_new_no_specifiers(self):
        node, _ = compileScenicAST(New("Object", []))
        match node:
            case Call(Name("new"), [Name("Object"), List([])]):
                assert True
            case _:
                assert False

    def test_new_one_specifier(self):
        node, _ = compileScenicAST(New("Object", [WithSpecifier("foo", Constant(1))]))
        match node:
            case Call(
                Name("new"),
                [
                    Name("Object"),
                    List([Call(Name("With"), [Constant("foo"), Constant(1)])]),
                ],
            ):
                assert True
            case _:
                assert False

    def test_new_multiple_specifiers(self):
        node, _ = compileScenicAST(
            New(
                "Object",
                [WithSpecifier("foo", Constant(1)), AtSpecifier(Name("position"))],
            )
        )
        match node:
            case Call(
                Name("new"),
                [
                    Name("Object"),
                    List(
                        [
                            Call(Name("With"), [Constant("foo"), Constant(1)]),
                            Call(Name("At"), [Name("position")]),
                        ]
                    ),
                ],
            ):
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
        node, _ = compileScenicAST(DirectionOfSpecifier(LeftOf(), Name("x"), None))
        match node:
            case Call(Name("LeftSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_right(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(RightOf(), Name("x"), None))
        match node:
            case Call(Name("RightSpec"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_ahead(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(AheadOf(), Name("x"), None))
        match node:
            case Call(Name("Ahead"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_behind(self):
        node, _ = compileScenicAST(DirectionOfSpecifier(Behind(), Name("x"), None))
        match node:
            case Call(Name("Behind"), [Name("x")]):
                assert True
            case _:
                assert False

    def test_position_specifier_distance(self):
        node, _ = compileScenicAST(
            DirectionOfSpecifier(Behind(), Name("x"), Constant(10))
        )
        match node:
            case Call(Name("Behind"), [Name("x")], [keyword("dist", Constant(10))]):
                assert True
            case _:
                assert False

    def test_position_specifier_unknown_direction(self):
        with pytest.raises(AssertionError):
            compileScenicAST(DirectionOfSpecifier(str(), Name("x"), None))

    def test_beyond_specifier(self):
        node, _ = compileScenicAST(BeyondSpecifier(Name("x"), Name("y")))
        match node:
            case Call(Name("Beyond"), [Name("x"), Name("y")]):
                assert True
            case _:
                assert False

    def test_beyond_specifier_with_base(self):
        node, _ = compileScenicAST(BeyondSpecifier(Name("x"), Name("y"), Name("z")))
        match node:
            case Call(
                Name("Beyond"), [Name("x"), Name("y")], [keyword("fromPt", Name("z"))]
            ):
                assert True
            case _:
                assert False

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

    # Operators
    def test_relative_position_op(self):
        node, _ = compileScenicAST(RelativePositionOp(Name("X")))
        match node:
            case Call(Name("RelativePosition"), [Name("X")]):
                assert True
            case _:
                assert False

    def test_relative_position_op_base(self):
        node, _ = compileScenicAST(RelativePositionOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("RelativePosition"), [Name("X")], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_relative_heading_op(self):
        node, _ = compileScenicAST(RelativeHeadingOp(Name("X")))
        match node:
            case Call(Name("RelativeHeading"), [Name("X")]):
                assert True
            case _:
                assert False

    def test_relative_heading_op_base(self):
        node, _ = compileScenicAST(RelativeHeadingOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("RelativeHeading"), [Name("X")], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_apparent_heading_op(self):
        node, _ = compileScenicAST(ApparentHeadingOp(Name("X")))
        match node:
            case Call(Name("ApparentHeading"), [Name("X")]):
                assert True
            case _:
                assert False

    def test_apparent_heading_op_base(self):
        node, _ = compileScenicAST(ApparentHeadingOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("ApparentHeading"), [Name("X")], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_distance_to_op(self):
        node, _ = compileScenicAST(DistanceFromOp(Name("X"), None))
        match node:
            case Call(Name("DistanceFrom"), [Name("X")], []):
                assert True
            case _:
                assert False

    def test_distance_to_op_from(self):
        node, _ = compileScenicAST(DistanceFromOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("DistanceFrom"), [Name("X")], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_distance_past_op(self):
        node, _ = compileScenicAST(DistancePastOp(Name("X")))
        match node:
            case Call(Name("DistancePast"), [Name("X")]):
                assert True
            case _:
                assert False

    def test_distance_past_op_of(self):
        node, _ = compileScenicAST(DistancePastOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("DistancePast"), [Name("X")], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_angle_to_op(self):
        node, _ = compileScenicAST(AngleFromOp(Name("Y"), None))
        match node:
            case Call(Name("AngleFrom"), [], [keyword("Y", Name("Y"))]):
                assert True
            case _:
                assert False

    def test_angle_from_op(self):
        node, _ = compileScenicAST(AngleFromOp(None, Name("X")))
        match node:
            case Call(Name("AngleFrom"), [], [keyword("X", Name("X"))]):
                assert True
            case _:
                assert False

    def test_angle_to_from_op(self):
        node, _ = compileScenicAST(AngleFromOp(Name("Y"), Name("X")))
        match node:
            case Call(
                Name("AngleFrom"),
                [],
                [keyword("X", Name("X")), keyword("Y", Name("Y"))],
            ):
                assert True
            case _:
                assert False

    def test_angle_op_invalid(self):
        with pytest.raises(AssertionError):
            # target or base needs to be set
            compileScenicAST(AngleFromOp())

    def test_follow_op(self):
        node, _ = compileScenicAST(FollowOp(Name("X"), Name("Y"), Name("Z")))
        match node:
            case Call(Name("Follow"), [Name("X"), Name("Y"), Name("Z")]):
                assert True
            case _:
                assert False

    def test_visible_op(self):
        node, _ = compileScenicAST(VisibleOp(Name("X")))
        match node:
            case Call(Name("Visible"), [Name("X")]):
                assert True
            case _:
                assert False

    def test_not_visible_op(self):
        node, _ = compileScenicAST(NotVisibleOp(Name("X")))
        match node:
            case Call(Name("NotVisible"), [Name("X")]):
                assert True
            case _:
                assert False

    @pytest.mark.parametrize(
        "node,function_name",
        [
            (Front, "Front"),
            (Back, "Back"),
            (Left, "Left"),
            (Right, "Right"),
            (FrontLeft, "FrontLeft"),
            (FrontRight, "FrontRight"),
            (BackLeft, "BackLeft"),
            (BackRight, "BackRight"),
        ],
    )
    def test_position_of_op(self, node, function_name):
        node, _ = compileScenicAST(PositionOfOp(node(), Name("X")))
        match node:
            case Call(Name(f), [Name("X")]):
                assert f == function_name
            case _:
                assert False

    def test_deg_op(self):
        node, _ = compileScenicAST(DegOp(Name("X")))
        match node:
            case BinOp(Name("X"), Mult(), Constant()):
                assert True
            case _:
                assert False

    def test_vector_op(self):
        node, _ = compileScenicAST(VectorOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("Vector"), [Name("X"), Name("Y")]):
                assert True
            case _:
                assert False

    def test_field_at_op(self):
        node, _ = compileScenicAST(FieldAtOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("FieldAt"), [Name("X"), Name("Y")]):
                assert True
            case _:
                assert False

    def test_relative_to_op(self):
        node, _ = compileScenicAST(RelativeToOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("RelativeTo"), [Name("X"), Name("Y")]):
                assert True
            case _:
                assert False

    def test_offset_along_op(self):
        node, _ = compileScenicAST(OffsetAlongOp(Name("X"), Name("Y"), Name("Z")))
        match node:
            case Call(Name("OffsetAlong"), [Name("X"), Name("Y"), Name("Z")]):
                assert True
            case _:
                assert False

    def test_can_see_op(self):
        node, _ = compileScenicAST(CanSeeOp(Name("X"), Name("Y")))
        match node:
            case Call(Name("CanSee"), [Name("X"), Name("Y")]):
                assert True
            case _:
                assert False
