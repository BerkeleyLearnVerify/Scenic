from ast import *

import pytest

from scenic.syntax.ast import *
from scenic.syntax.compiler import compileScenicAST


class TestCompiler:
    def test_scenic_try(self):
        node, _ = compileScenicAST(
            TryInterrupt(
                body=[Expr(Call(Name("foo", Load()), args=[], keywords=[]))],
                interrupt_when_handlers=[
                    InterruptWhenHandler(
                        Constant(True),
                        [Expr(Call(Name("bar", Load()), args=[], keywords=[]))],
                    )
                ],
                except_handlers=[
                    ExceptHandler(
                        Constant(True),
                        None,
                        [
                            Expr(
                                Call(
                                    Name("handle_except", Load()), args=[], keywords=[]
                                )
                            )
                        ],
                    )
                ],
                orelse=[],
                finalbody=[],
            )
        )
        match node:
            case Try(
                body=[
                    FunctionDef(
                        name="_Scenic_interrupt_body",
                        args=arguments(
                            args=[arg("_Scenic_current_behavior"), arg("self")],
                        ),
                        body=[
                            Expr(Call(Name("foo"))),
                            Return(
                                value=Attribute(
                                    value=Name("BlockConclusion"), attr="FINISHED"
                                )
                            ),
                        ],
                    ),
                    FunctionDef(
                        name="_Scenic_interrupt_handler_0",
                        args=arguments(
                            args=[arg("_Scenic_current_behavior"), arg("self")],
                        ),
                        body=[
                            Expr(value=Call(func=Name("bar"))),
                            Return(
                                value=Attribute(
                                    value=Name("BlockConclusion"),
                                    attr="FINISHED",
                                )
                            ),
                        ],
                    ),
                    Assign(
                        targets=[Name("_Scenic_interrupt_condition_0")],
                        value=Lambda(
                            args=arguments(),
                            body=Constant(True),
                        ),
                    ),
                    Assign(
                        targets=[Name("_Scenic_temporary_name")],
                        value=YieldFrom(
                            value=Call(
                                func=Name("runTryInterrupt"),
                                args=[
                                    Name("_Scenic_current_behavior"),
                                    Name("self"),
                                    Name("_Scenic_interrupt_body"),
                                    Tuple(
                                        elts=[Name("_Scenic_interrupt_condition_0")],
                                    ),
                                    Tuple(
                                        elts=[Name("_Scenic_interrupt_handler_0")],
                                    ),
                                ],
                            )
                        ),
                    ),
                    If(
                        test=Compare(
                            left=Name("_Scenic_temporary_name"),
                            ops=[Is()],
                            comparators=[
                                Attribute(
                                    value=Name("BlockConclusion"),
                                    attr="RETURN",
                                )
                            ],
                        ),
                        body=[
                            Return(
                                value=Attribute(
                                    value=Name("_Scenic_temporary_name"),
                                    attr="return_value",
                                )
                            )
                        ],
                    ),
                ],
                handlers=[
                    ExceptHandler(
                        type=Constant(value=True),
                        body=[Expr(value=Call(func=Name("handle_except")))],
                    )
                ],
                orelse=[],
                finalbody=[],
            ):
                assert True
            case _:
                assert False

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

    def test_unpack_distribution(self):
        node, _ = compileScenicAST(
            Call(Name("fn"), [Starred(Name("dist", lineno=1)), Name("ego", Load())], [])
        )
        match node:
            case Call(
                Name("callWithStarArgs"),
                [
                    Name("fn"),
                    Starred(
                        Call(
                            Name("wrapStarredValue"),
                            [Name("dist"), Constant(1)],
                            [],
                        )
                    ),
                    Call(Name("ego")),
                ],
                [],
            ):
                assert True
            case _:
                assert False

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

    def test_mutate_scale(self):
        node, _ = compileScenicAST(Mutate([Name("x")], Constant(2)))
        match node:
            case Expr(
                Call(Name("mutate"), [Name("x")], [keyword("scale", Constant(2))])
            ):
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
                    kwargs[0].value.value if kwargs else 1.0
                )  # if kwargs is empty, use 1.0
                assert compiled_prob == expected_prob
            case _:
                assert False

        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_behavior(self):
        node, _ = compileScenicAST(
            BehaviorDef(
                name="Foo",
                args=arguments(
                    posonlyargs=[],
                    args=[arg("input")],
                    kwonlyargs=[],
                    kw_defaults=[],
                    defaults=[],
                ),
                docstring="'''DOCSTRING'''",
                header=[
                    Invariant(
                        value=Compare(
                            left=Name(id="localvar", ctx=Load()),
                            ops=[Gt()],
                            comparators=[Name(id="input", ctx=Load())],
                        ),
                        lineno=2,
                    ),
                    Precondition(
                        value=Compare(
                            left=Name(id="localvar", ctx=Load()),
                            ops=[Gt()],
                            comparators=[Name(id="input", ctx=Load())],
                        ),
                        lineno=3,
                    ),
                ],
                body=[
                    Assign(
                        targets=[Name(id="localvar", ctx=Store())],
                        value=Constant(value=1),
                    ),
                    While(test=Constant(value=True), body=[Pass()], orelse=[]),
                ],
            )
        )
        match node:
            case ClassDef(
                name="Foo",
                bases=[Name("Behavior")],
                keywords=[],
                body=[
                    Expr(Constant("DOCSTRING")),  # preserved docstring
                    FunctionDef(
                        name="checkPreconditions",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                            args=[arg("input")],
                        ),
                        body=[
                            If(
                                test=UnaryOp(
                                    op=Not(),
                                    operand=Compare(
                                        left=Attribute(
                                            value=Name("_Scenic_current_behavior"),
                                            attr="localvar",
                                        ),
                                        ops=[Gt()],
                                        comparators=[Name("input")],
                                    ),
                                ),
                                body=[
                                    Raise(
                                        exc=Call(
                                            func=Name("PreconditionViolation"),
                                            args=[
                                                Name("_Scenic_current_behavior"),
                                                Constant(3),
                                            ],
                                        )
                                    )
                                ],
                            )
                        ],
                    ),
                    FunctionDef(
                        name="checkInvariants",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                            args=[arg("input")],
                        ),
                        body=[
                            If(
                                test=UnaryOp(
                                    op=Not(),
                                    operand=Compare(
                                        left=Attribute(
                                            value=Name("_Scenic_current_behavior"),
                                            attr="localvar",
                                        ),
                                        ops=[Gt()],
                                        comparators=[Name("input")],
                                    ),
                                ),
                                body=[
                                    Raise(
                                        exc=Call(
                                            func=Name("InvariantViolation"),
                                            args=[
                                                Name("_Scenic_current_behavior"),
                                                Constant(2),
                                            ],
                                        )
                                    )
                                ],
                            )
                        ],
                    ),
                    Assign(
                        targets=[Name("_locals")],
                        value=Constant(frozenset()),
                    ),
                    FunctionDef(
                        name="makeGenerator",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                            args=[arg("input")],
                        ),
                        body=[
                            Assign(
                                targets=[
                                    Attribute(
                                        value=Name("_Scenic_current_behavior"),
                                        attr="input",
                                    )
                                ],
                                value=Name("input"),
                            ),
                            Assign(
                                targets=[
                                    Attribute(
                                        value=Name("_Scenic_current_behavior"),
                                        attr="localvar",
                                    )
                                ],
                                value=Constant(1),
                            ),
                            While(test=Constant(True), body=[Pass()]),
                        ],
                    ),
                ],
            ):
                assert True
            case _:
                assert False

    def test_monitor(self):
        node, _ = compileScenicAST(
            MonitorDef(
                name="M",
                docstring="'DOCSTRING'",
                body=[
                    Assign(
                        targets=[Name(id="localvar", ctx=Store())],
                        value=Constant(value=1),
                    ),
                    While(test=Constant(value=True), body=[Pass()], orelse=[]),
                ],
            )
        )
        match node:
            case ClassDef(
                name="M",
                bases=[Name("Monitor")],
                body=[
                    Expr(Constant("DOCSTRING")),
                    FunctionDef(
                        name="checkPreconditions",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                        ),
                        body=[Pass()],
                    ),
                    FunctionDef(
                        name="checkInvariants",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                        ),
                        body=[Pass()],
                    ),
                    Assign(
                        targets=[Name("_locals")],
                        value=Constant(value=frozenset()),
                    ),
                    FunctionDef(
                        name="makeGenerator",
                        args=arguments(
                            posonlyargs=[
                                arg("_Scenic_current_behavior"),
                                arg("self"),
                            ],
                        ),
                        body=[
                            Assign(
                                targets=[
                                    Attribute(
                                        value=Name("_Scenic_current_behavior"),
                                        attr="localvar",
                                        ctx=Store(),
                                    )
                                ],
                                value=Constant(value=1),
                            ),
                            While(
                                test=Constant(value=True),
                                body=[Pass()],
                            ),
                        ],
                    ),
                ],
            ):
                assert True
            case _:
                assert False

    def test_abort(self):
        node, _ = compileScenicAST(Abort())
        match node:
            case Return(Attribute(Name("BlockConclusion"), "ABORT")):
                assert True
            case _:
                assert False

    def test_take(self):
        node, _ = compileScenicAST(Take([Constant(1), Constant(2), Constant(3)]))
        match node:
            case [
                Expr(value=Yield(value=Tuple([Constant(1), Constant(2), Constant(3)]))),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_wait(self):
        node, _ = compileScenicAST(Wait())
        match node:
            case [
                Expr(value=Yield(value=Constant(value=()))),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_terminate(self):
        node, _ = compileScenicAST(Terminate(lineno=1))
        match node:
            case [
                Expr(
                    value=Yield(
                        value=Call(
                            func=Name(id="makeTerminationAction", ctx=Load()),
                            args=[Constant(value=1)],
                            keywords=[],
                        )
                    )
                ),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_do(self):
        node, _ = compileScenicAST(Do([Constant(1)]))
        match node:
            case [
                Expr(
                    value=YieldFrom(
                        value=Call(
                            func=Attribute(
                                value=Name(id="_Scenic_current_behavior", ctx=Load()),
                                attr="_invokeSubBehavior",
                                ctx=Load(),
                            ),
                            args=[
                                Name(id="self", ctx=Load()),
                                Tuple(
                                    elts=[Constant(value=1)],
                                    ctx=Load(),
                                ),
                            ],
                            keywords=[],
                        )
                    )
                ),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_do_for_seconds(self):
        node, _ = compileScenicAST(DoFor([Constant("foo")], Seconds(Constant(1))))
        match node:
            case [
                Expr(
                    value=YieldFrom(
                        value=Call(
                            func=Attribute(
                                value=Name(id="_Scenic_current_behavior", ctx=Load()),
                                attr="_invokeSubBehavior",
                                ctx=Load(),
                            ),
                            args=[
                                Name(id="self", ctx=Load()),
                                Tuple(
                                    elts=[Constant("foo")],
                                    ctx=Load(),
                                ),
                                ast.Call(
                                    func=ast.Name("Modifier"),
                                    args=[
                                        ast.Constant("for"),
                                        Constant(1),
                                        ast.Constant("seconds"),
                                    ],
                                    keywords=[],
                                ),
                            ],
                            keywords=[],
                        )
                    )
                ),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_do_for_steps(self):
        node, _ = compileScenicAST(DoFor([Constant("foo")], Steps(Constant(3))))
        match node:
            case [
                Expr(
                    value=YieldFrom(
                        value=Call(
                            func=Attribute(
                                value=Name(id="_Scenic_current_behavior", ctx=Load()),
                                attr="_invokeSubBehavior",
                                ctx=Load(),
                            ),
                            args=[
                                Name(id="self", ctx=Load()),
                                Tuple(
                                    elts=[Constant("foo")],
                                    ctx=Load(),
                                ),
                                ast.Call(
                                    func=ast.Name("Modifier"),
                                    args=[
                                        ast.Constant("for"),
                                        Constant(3),
                                        ast.Constant("steps"),
                                    ],
                                    keywords=[],
                                ),
                            ],
                            keywords=[],
                        )
                    )
                ),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def test_do_until(self):
        node, _ = compileScenicAST(DoUntil([Constant("foo")], Name("condition")))
        match node:
            case [
                Expr(
                    value=YieldFrom(
                        value=Call(
                            func=Attribute(
                                value=Name(id="_Scenic_current_behavior", ctx=Load()),
                                attr="_invokeSubBehavior",
                                ctx=Load(),
                            ),
                            args=[
                                Name(id="self", ctx=Load()),
                                Tuple(
                                    elts=[Constant("foo")],
                                    ctx=Load(),
                                ),
                                ast.Call(
                                    func=ast.Name("Modifier"),
                                    args=[
                                        ast.Constant("until"),
                                        Lambda(body=Name("condition")),
                                    ],
                                    keywords=[],
                                ),
                            ],
                            keywords=[],
                        )
                    )
                ),
                checkInvariants,
            ]:
                self.assert_invocation_check_invariants(checkInvariants)
            case _:
                assert False

    def assert_invocation_check_invariants(self, node):
        match node:
            case Expr(
                value=Call(
                    func=Attribute(
                        value=Name(
                            id="_Scenic_current_behavior",
                            ctx=Load(),
                        ),
                        attr="checkInvariants",
                        ctx=Load(),
                    ),
                    args=[
                        Name(id="self", ctx=Load()),
                        Starred(
                            value=Attribute(
                                value=Name(
                                    id="_Scenic_current_behavior",
                                    ctx=Load(),
                                ),
                                attr="_args",
                                ctx=Load(),
                            ),
                            ctx=Load(),
                        ),
                    ],
                    keywords=[
                        keyword(
                            value=Attribute(
                                value=Name(
                                    id="_Scenic_current_behavior",
                                    ctx=Load(),
                                ),
                                attr="_kwargs",
                                ctx=Load(),
                            )
                        )
                    ],
                )
            ):
                assert True
            case _:
                assert False

    def test_require_always(self):
        node, requirements = compileScenicAST(RequireAlways(Name("C"), lineno=2))
        match node:
            case Expr(
                Call(
                    Name("require_always"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # requirement
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False
        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_require_eventually(self):
        node, requirements = compileScenicAST(RequireEventually(Name("C"), lineno=2))
        match node:
            case Expr(
                Call(
                    Name("require_eventually"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # requirement
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False
        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_record(self):
        node, requirements = compileScenicAST(Record(Name("C"), lineno=2))
        match node:
            case Expr(
                Call(
                    Name("record"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # record value
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False
        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_record_initial(self):
        node, requirements = compileScenicAST(RecordInitial(Name("C"), lineno=2))
        match node:
            case Expr(
                Call(
                    Name("record_initial"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # record value
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False
        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_record_final(self):
        node, requirements = compileScenicAST(RecordFinal(Name("C"), lineno=2))
        match node:
            case Expr(
                Call(
                    Name("record_final"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # record value
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False
        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_terminate_when(self):
        node, requirements = compileScenicAST(TerminateWhen(Name("C"), lineno=2))

        match node:
            case Expr(
                Call(
                    Name("terminate_when"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # requirement
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False

        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_terminate_simulation_when(self):
        node, requirements = compileScenicAST(
            TerminateSimulationWhen(Name("C"), lineno=2)
        )

        match node:
            case Expr(
                Call(
                    Name("terminate_simulation_when"),
                    [
                        Constant(0),  # reqId
                        Lambda(body=Name("C")),  # requirement
                        Constant(2),  # lineno
                        Constant(None),  # name
                    ],
                )
            ):
                assert True
            case _:
                assert False

        match requirements:
            case [Name("C")]:
                assert True
            case _:
                assert False

    def test_terminate_after_seconds(self):
        node, _ = compileScenicAST(TerminateAfter(Seconds(Constant(10))))
        match node:
            case Expr(Call(Name("terminate_after"), [Constant(10), Constant("seconds")], [])):
                assert True
            case _:
                assert False

    def test_terminate_after_steps(self):
        node, _ = compileScenicAST(TerminateAfter(Steps(Constant(20))))
        match node:
            case Expr(Call(Name("terminate_after"), [Constant(20), Constant("steps")], [])):
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

    def test_not_visible_specifier(self):
        node, _ = compileScenicAST(NotVisibleSpecifier())
        match node:
            case Call(Name("NotVisibleSpec")):
                assert True
            case _:
                assert False

    def test_not_visible_specifier_with_base(self):
        node, _ = compileScenicAST(NotVisibleSpecifier(Name("x")))
        match node:
            case Call(Name("NotVisibleFrom"), [Name("x")]):
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
