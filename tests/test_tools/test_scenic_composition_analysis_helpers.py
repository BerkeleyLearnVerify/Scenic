from pathlib import Path
import sys

import pytest

from tools.scenic_composition_analysis_helpers import (
    Container,
    CompositionGraph,
    CompositionStatement,
    GraphEdge,
    GraphNode,
    InvocationSpec,
    extract_from_parser,
    load_source,
    operator_semantics,
    parse_weight,
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
