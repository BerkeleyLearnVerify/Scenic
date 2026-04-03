from pathlib import Path
import sys

import pytest

from tools.scenic_composition_analysis_helpers import (
    Container,
    CompositionGraph,
    CompositionStatement,
    ExecutionStructure,
    GraphEdge,
    GraphNode,
    InvocationSpec,
    SXONode,
    SXOStructure,
    SxOEdge,
    choose_invocation,
    extract_from_parser,
    invocation_result,
    load_source,
    operator_semantics,
    ordered_compositions,
    parse_weight,
    sample_composition,
    sort_node_ids,
    target_name,
)


EXAMPLE_CASES = [
    (
        "examples/driving/OAS_Scenarios/oas_scenario_03.scenic",
        ["<initial>", "CollisionAvoidance", "FollowLeadCarBehavior"],
        2,
    ),
    (
        "examples/driving/OAS_Scenarios/oas_scenario_04.scenic",
        ["<initial>", "LeadCarBehavior", "CollisionAvoidance", "FollowLeadCarBehavior"],
        3,
    ),
    (
        "examples/driving/OAS_Scenarios/oas_scenario_28.scenic",
        ["<initial>", "EgoBehavior", "OtherCarBehavior"],
        2,
    ),
    (
        "examples/driving/OAS_Scenarios/oas_scenario_29.scenic",
        ["<initial>", "EgoBehavior"],
        1,
    ),
    (
        "examples/driving/OAS_Scenarios/oas_scenario_30.scenic",
        ["<initial>", "SafeBehavior", "EgoBehavior"],
        2,
    ),
    (
        "examples/driving/OAS_Scenarios/oas_scenario_32.scenic",
        ["<initial>", "FollowTrafficBehavior", "SafeBehavior"],
        2,
    ),
    (
        "examples/driving/Carla_Challenge/carlaChallenge2.scenic",
        ["<initial>", "EgoBehavior", "LeadingCarBehavior"],
        2,
    ),
    (
        "examples/driving/Carla_Challenge/carlaChallenge3.scenic",
        ["<initial>", "EgoBehavior"],
        1,
    ),
]


def test_load_source_from_string():
    text, source_path, source_kind = load_source("behavior Foo():\n    do Bar()")

    assert text == "behavior Foo():\n    do Bar()"
    assert source_path is None
    assert source_kind == "string"


def test_load_source_from_path():
    path = Path("examples/driving/OAS_Scenarios/oas_scenario_03.scenic")

    text, source_path, source_kind = load_source(path)

    assert "behavior FollowLeadCarBehavior" in text
    assert source_path == path.resolve()
    assert source_kind == "path"


@pytest.mark.parametrize(
    "path_str, expected_containers, expected_statement_count", EXAMPLE_CASES
)
def test_extract_from_parser_matches_real_examples(
    path_str, expected_containers, expected_statement_count
):
    text = Path(path_str).read_text()

    containers, statements = extract_from_parser(text)

    assert [container.name for container in containers] == expected_containers
    assert all(isinstance(container, Container) for container in containers)
    assert len(statements) == expected_statement_count
    assert all(isinstance(statement, CompositionStatement) for statement in statements)
    assert all(statement.operator == "parallel" for statement in statements)


def test_extract_from_parser_captures_try_interrupt_structure():
    text = Path("examples/driving/OAS_Scenarios/oas_scenario_04.scenic").read_text()

    containers, statements = extract_from_parser(text)

    assert [container.name for container in containers] == [
        "<initial>",
        "LeadCarBehavior",
        "CollisionAvoidance",
        "FollowLeadCarBehavior",
    ]
    assert [statement.container_name for statement in statements] == [
        "LeadCarBehavior",
        "FollowLeadCarBehavior",
        "FollowLeadCarBehavior",
    ]
    assert statements[0].invocations[0].target == "FollowLaneBehavior"
    assert statements[0].nesting == ("try",)
    assert statements[1].invocations[0].text == "FollowLaneBehavior()"
    assert statements[1].invocations[0].target == "FollowLaneBehavior"
    assert statements[1].nesting == ("try",)
    assert statements[2].nesting == (
        "try",
        "interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE)",
    )
    assert statements[2].invocations[0].target == "CollisionAvoidance"


def test_target_name_and_parse_weight_helpers():
    assert target_name("FollowLaneBehavior(speed)") == "FollowLaneBehavior"
    assert target_name("CollisionAvoidance()") == "CollisionAvoidance"
    assert target_name("vehicle + 1") is None

    assert parse_weight("3") == 3.0
    assert parse_weight("0.25") == 0.25
    assert parse_weight("not-a-number") is None


def test_operator_semantics_for_all_supported_operators():
    assert operator_semantics("parallel", "behavior") == {
        "container_kind": "behavior",
        "execution": "parallel-all",
        "randomized": False,
        "weighted": False,
    }
    assert operator_semantics("choose", "scenario") == {
        "container_kind": "scenario",
        "execution": "single-enabled-choice",
        "randomized": True,
        "weighted": True,
    }
    assert operator_semantics("shuffle", "scenario") == {
        "container_kind": "scenario",
        "execution": "random-permutation-of-enabled-choices",
        "randomized": True,
        "weighted": True,
    }


def test_dataclasses_round_trip_with_as_dict():
    graph = CompositionGraph(
        source_path="example.scenic",
        source_kind="path",
        container_names=("Main",),
        statements=(
            CompositionStatement(
                node_id="Main:1",
                container_name="Main",
                operator="parallel",
                semantics={"container_kind": "scenario"},
                invocations=(
                    InvocationSpec(
                        text="Sub()",
                        target="Sub",
                        weight=None,
                        is_weighted=False,
                        line=3,
                    ),
                ),
                line=3,
            ),
        ),
        nodes=(
            GraphNode(id="scenario:Main", kind="scenario", label="Main"),
            GraphNode(id="Main:1", kind="composition", label="parallel"),
        ),
        edges=(GraphEdge(source="scenario:Main", target="Main:1", kind="contains"),),
    )

    as_dict = graph.as_dict()

    assert as_dict["container_names"] == ("Main",)
    assert as_dict["statements"][0]["container_name"] == "Main"
    assert as_dict["nodes"][0]["id"] == "scenario:Main"
    assert as_dict["edges"][0]["kind"] == "contains"


def test_execution_and_sxo_dataclasses_round_trip_with_as_dict():
    node = GraphNode(id="behavior:Foo", kind="behavior", label="Foo")
    execution = ExecutionStructure(
        container_ids=("behavior:Foo",),
        containers={
            "behavior:Foo": {
                "node": node,
                "composition_ids": ["Foo:1"],
                "start_ids": ["Foo:1"],
            }
        },
        compositions={
            "Foo:1": {
                "node": GraphNode(id="Foo:1", kind="composition", label="parallel"),
                "invocation_ids": ["Foo:1:invocation:0"],
                "next_ids": [],
            }
        },
        invocations={
            "Foo:1:invocation:0": GraphNode(
                id="Foo:1:invocation:0", kind="invocation", label="Bar"
            )
        },
        node_by_id={"behavior:Foo": node},
    )
    sxo = SXOStructure(
        nodes=(SXONode(id="S:behavior:Foo", kind="S", label="Foo", attributes={}),),
        edges=(SxOEdge(source="S:behavior:Foo", target="X:Foo:1", kind="S_to_X", attributes={}),),
    )

    assert execution.as_dict()["container_ids"] == ("behavior:Foo",)
    assert execution.as_dict()["containers"]["behavior:Foo"]["node"]["label"] == "Foo"
    assert sxo.as_dict()["nodes"][0]["kind"] == "S"
    assert sxo.as_dict()["edges"][0]["kind"] == "S_to_X"


def test_sort_node_ids_orders_by_line_then_id():
    node_by_id = {
        "b": GraphNode(id="b", kind="composition", label="parallel", attributes={"line": 3}),
        "a": GraphNode(id="a", kind="composition", label="parallel", attributes={"line": 3}),
        "c": GraphNode(id="c", kind="composition", label="parallel", attributes={"line": 1}),
    }

    assert sort_node_ids(["b", "a", "c"], node_by_id) == ["c", "a", "b"]


def test_ordered_compositions_follows_start_and_next_then_remaining():
    execution = ExecutionStructure(
        container_ids=("behavior:Foo",),
        containers={
            "behavior:Foo": {
                "node": GraphNode(id="behavior:Foo", kind="behavior", label="Foo"),
                "composition_ids": ["Foo:1", "Foo:2", "Foo:3"],
                "start_ids": ["Foo:1"],
            }
        },
        compositions={
            "Foo:1": {
                "node": GraphNode(id="Foo:1", kind="composition", label="parallel"),
                "invocation_ids": [],
                "next_ids": ["Foo:2"],
            },
            "Foo:2": {
                "node": GraphNode(id="Foo:2", kind="composition", label="parallel"),
                "invocation_ids": [],
                "next_ids": [],
            },
            "Foo:3": {
                "node": GraphNode(id="Foo:3", kind="composition", label="parallel"),
                "invocation_ids": [],
                "next_ids": [],
            },
        },
        invocations={},
        node_by_id={},
    )

    ordered = ordered_compositions(execution.containers["behavior:Foo"], execution)

    assert ordered == ["Foo:1", "Foo:2", "Foo:3"]


def test_choose_invocation_prefers_weighted_choice(monkeypatch):
    invocation_nodes = {
        "a": GraphNode(id="a", kind="invocation", label="A", attributes={"weight": 1.0}),
        "b": GraphNode(id="b", kind="invocation", label="B", attributes={"weight": 5.0}),
    }

    def fake_choices(population, weights, k):
        assert population == ["a", "b"]
        assert weights == [1.0, 5.0]
        assert k == 1
        return ["b"]

    monkeypatch.setattr("tools.scenic_composition_analysis_helpers.random.choices", fake_choices)

    assert choose_invocation(["a", "b"], invocation_nodes) == "b"


def test_choose_invocation_uses_uniform_choice_without_weights(monkeypatch):
    invocation_nodes = {
        "a": GraphNode(id="a", kind="invocation", label="A", attributes={}),
        "b": GraphNode(id="b", kind="invocation", label="B", attributes={}),
    }

    monkeypatch.setattr("tools.scenic_composition_analysis_helpers.random.choice", lambda population: population[0])

    assert choose_invocation(["a", "b"], invocation_nodes) == "a"


def test_invocation_result_prefers_target_over_text():
    with_target = GraphNode(
        id="a",
        kind="invocation",
        label="A",
        attributes={"target": "CollisionAvoidance", "text": "CollisionAvoidance()"},
    )
    without_target = GraphNode(
        id="b",
        kind="invocation",
        label="B",
        attributes={"target": None, "text": "new Object"},
    )

    assert invocation_result(with_target) == "CollisionAvoidance"
    assert invocation_result(without_target) == "new Object"


def test_sample_composition_parallel_returns_all_targets():
    composition = {
        "node": GraphNode(id="Foo:1", kind="composition", label="parallel"),
        "invocation_ids": ["a", "b"],
    }
    invocations = {
        "a": GraphNode(id="a", kind="invocation", label="A", attributes={"target": "A"}),
        "b": GraphNode(id="b", kind="invocation", label="B", attributes={"target": "B"}),
    }

    assert sample_composition(composition, invocations) == ["A", "B"]


def test_sample_composition_choose_and_shuffle(monkeypatch):
    choose_composition = {
        "node": GraphNode(id="Foo:1", kind="composition", label="choose"),
        "invocation_ids": ["a", "b"],
    }
    shuffle_composition = {
        "node": GraphNode(id="Foo:2", kind="composition", label="shuffle"),
        "invocation_ids": ["a", "b"],
    }
    invocations = {
        "a": GraphNode(id="a", kind="invocation", label="A", attributes={"target": "A"}),
        "b": GraphNode(id="b", kind="invocation", label="B", attributes={"target": "B"}),
    }

    monkeypatch.setattr(
        "tools.scenic_composition_analysis_helpers.choose_invocation",
        lambda invocation_ids, invocation_nodes: "b",
    )
    monkeypatch.setattr(
        "tools.scenic_composition_analysis_helpers.random.shuffle",
        lambda items: items.reverse(),
    )

    assert sample_composition(choose_composition, invocations) == ["B"]
    assert sample_composition(shuffle_composition, invocations) == ["B", "A"]
