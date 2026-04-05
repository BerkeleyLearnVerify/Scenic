from pathlib import Path

import pytest

from tools.scenic_composition_analysis import (
    analyze_scenic_composition,
    build_compact_graph_dict,
    find_composition_statements,
)


GRAPH_CASES = [
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_03.scenic",
        "containers": ("<initial>", "CollisionAvoidance", "FollowLeadCarBehavior"),
        "statement_containers": ("FollowLeadCarBehavior", "FollowLeadCarBehavior"),
        "statement_node_ids": ("FollowLeadCarBehavior:1", "FollowLeadCarBehavior:2"),
        "invocation_texts": ("FollowLaneBehavior()", "CollisionAvoidance()"),
        "invocation_targets": ("FollowLaneBehavior", "CollisionAvoidance"),
        "nestings": (
            ("try",),
            ("try", "interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE)"),
        ),
        "node_counts": {"initial": 1, "behavior": 2, "composition": 2, "invocation": 2},
        "edge_counts": {"contains": 2, "invokes": 2, "next": 1},
        "next_within": ("FollowLeadCarBehavior",),
        "non_composition_containers": ("<initial>", "CollisionAvoidance"),
    },
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_04.scenic",
        "containers": (
            "<initial>",
            "LeadCarBehavior",
            "CollisionAvoidance",
            "FollowLeadCarBehavior",
        ),
        "statement_containers": (
            "LeadCarBehavior",
            "FollowLeadCarBehavior",
            "FollowLeadCarBehavior",
        ),
        "statement_node_ids": (
            "LeadCarBehavior:1",
            "FollowLeadCarBehavior:1",
            "FollowLeadCarBehavior:2",
        ),
        "invocation_texts": (
            "FollowLaneBehavior()",
            "FollowLaneBehavior()",
            "CollisionAvoidance()",
        ),
        "invocation_targets": (
            "FollowLaneBehavior",
            "FollowLaneBehavior",
            "CollisionAvoidance",
        ),
        "nestings": (
            ("try",),
            ("try",),
            ("try", "interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE)"),
        ),
        "node_counts": {"initial": 1, "behavior": 3, "composition": 3, "invocation": 3},
        "edge_counts": {"contains": 3, "invokes": 3, "next": 1},
        "next_within": ("FollowLeadCarBehavior",),
        "non_composition_containers": ("<initial>", "CollisionAvoidance"),
    },
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_28.scenic",
        "containers": ("<initial>", "EgoBehavior", "OtherCarBehavior"),
        "statement_containers": ("EgoBehavior", "OtherCarBehavior"),
        "statement_node_ids": ("EgoBehavior:1", "OtherCarBehavior:1"),
        "invocation_texts": (
            "FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)",
            "FollowTrajectoryBehavior(target_speed=8, trajectory=trajectory)",
        ),
        "invocation_targets": ("FollowTrajectoryBehavior", "FollowTrajectoryBehavior"),
        "nestings": (("try",), ()),
        "node_counts": {"initial": 1, "behavior": 2, "composition": 2, "invocation": 2},
        "edge_counts": {"contains": 2, "invokes": 2, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_29.scenic",
        "containers": ("<initial>", "EgoBehavior"),
        "statement_containers": ("EgoBehavior",),
        "statement_node_ids": ("EgoBehavior:1",),
        "invocation_texts": (
            "FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)",
        ),
        "invocation_targets": ("FollowTrajectoryBehavior",),
        "nestings": (("try",),),
        "node_counts": {"initial": 1, "behavior": 1, "composition": 1, "invocation": 1},
        "edge_counts": {"contains": 1, "invokes": 1, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_30.scenic",
        "containers": ("<initial>", "SafeBehavior", "EgoBehavior"),
        "statement_containers": ("SafeBehavior", "EgoBehavior"),
        "statement_node_ids": ("SafeBehavior:1", "EgoBehavior:1"),
        "invocation_texts": (
            "FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)",
            "FollowTrajectoryBehavior(target_speed, trajectory)",
        ),
        "invocation_targets": ("FollowTrajectoryBehavior", "FollowTrajectoryBehavior"),
        "nestings": (("try",), ()),
        "node_counts": {"initial": 1, "behavior": 2, "composition": 2, "invocation": 2},
        "edge_counts": {"contains": 2, "invokes": 2, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
    {
        "path": "examples/driving/OAS_Scenarios/oas_scenario_32.scenic",
        "containers": ("<initial>", "FollowTrafficBehavior", "SafeBehavior"),
        "statement_containers": ("FollowTrafficBehavior", "SafeBehavior"),
        "statement_node_ids": ("FollowTrafficBehavior:1", "SafeBehavior:1"),
        "invocation_texts": (
            "FollowTrajectoryBehavior(target_speed, trajectory)",
            "FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)",
        ),
        "invocation_targets": ("FollowTrajectoryBehavior", "FollowTrajectoryBehavior"),
        "nestings": ((), ("try",)),
        "node_counts": {"initial": 1, "behavior": 2, "composition": 2, "invocation": 2},
        "edge_counts": {"contains": 2, "invokes": 2, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
    {
        "path": "examples/driving/Carla_Challenge/carlaChallenge2.scenic",
        "containers": ("<initial>", "EgoBehavior", "LeadingCarBehavior"),
        "statement_containers": ("EgoBehavior", "LeadingCarBehavior"),
        "statement_node_ids": ("EgoBehavior:1", "LeadingCarBehavior:1"),
        "invocation_texts": ("FollowLaneBehavior(speed)", "FollowLaneBehavior(speed)"),
        "invocation_targets": ("FollowLaneBehavior", "FollowLaneBehavior"),
        "nestings": (("try",), ("try",)),
        "node_counts": {"initial": 1, "behavior": 2, "composition": 2, "invocation": 2},
        "edge_counts": {"contains": 2, "invokes": 2, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
    {
        "path": "examples/driving/Carla_Challenge/carlaChallenge3.scenic",
        "containers": ("<initial>", "EgoBehavior"),
        "statement_containers": ("EgoBehavior",),
        "statement_node_ids": ("EgoBehavior:1",),
        "invocation_texts": ("FollowLaneBehavior(speed)",),
        "invocation_targets": ("FollowLaneBehavior",),
        "nestings": (("try",),),
        "node_counts": {"initial": 1, "behavior": 1, "composition": 1, "invocation": 1},
        "edge_counts": {"contains": 1, "invokes": 1, "next": 0},
        "next_within": (),
        "non_composition_containers": ("<initial>",),
    },
]


def count_by_kind(items):
    counts = {}
    for item in items:
        counts[item.kind] = counts.get(item.kind, 0) + 1
    return counts


@pytest.mark.parametrize(
    "case", GRAPH_CASES, ids=[Path(case["path"]).stem for case in GRAPH_CASES]
)
def test_analyze_scenic_composition_builds_expected_graph_structure(case):
    graph = analyze_scenic_composition(case["path"])

    assert graph.source_kind == "path"
    assert graph.source_path == str(Path(case["path"]).resolve())
    assert graph.container_names == case["containers"]

    assert [statement.container_name for statement in graph.statements] == list(
        case["statement_containers"]
    )
    assert [statement.node_id for statement in graph.statements] == list(
        case["statement_node_ids"]
    )
    assert [statement.operator for statement in graph.statements] == ["parallel"] * len(
        case["statement_containers"]
    )
    assert [statement.invocations[0].text for statement in graph.statements] == list(
        case["invocation_texts"]
    )
    assert [statement.invocations[0].target for statement in graph.statements] == list(
        case["invocation_targets"]
    )
    assert [statement.nesting for statement in graph.statements] == list(
        case["nestings"]
    )

    node_counts = count_by_kind(graph.nodes)
    for kind, expected in case["node_counts"].items():
        assert node_counts.get(kind, 0) == expected

    edge_counts = count_by_kind(graph.edges)
    for kind, expected in case["edge_counts"].items():
        assert edge_counts.get(kind, 0) == expected

    next_edges = [edge for edge in graph.edges if edge.kind == "next"]
    assert (
        tuple(edge.attributes["within"] for edge in next_edges) == case["next_within"]
    )

    composition_node_ids = {
        node.id for node in graph.nodes if node.kind == "composition"
    }
    invocation_node_ids = {node.id for node in graph.nodes if node.kind == "invocation"}
    assert composition_node_ids == set(case["statement_node_ids"])
    assert invocation_node_ids == {
        f"{statement_id}:invocation:0" for statement_id in case["statement_node_ids"]
    }

    contains_edges = [edge for edge in graph.edges if edge.kind == "contains"]
    assert {(edge.source, edge.target) for edge in contains_edges} == {
        (f"behavior:{statement.container_name}", statement.node_id)
        for statement in graph.statements
    }

    invokes_edges = [edge for edge in graph.edges if edge.kind == "invokes"]
    assert {(edge.source, edge.target) for edge in invokes_edges} == {
        (statement.node_id, f"{statement.node_id}:invocation:0")
        for statement in graph.statements
    }

    containers_with_statements = {
        statement.container_name for statement in graph.statements
    }
    assert all(
        container_name not in containers_with_statements
        for container_name in case["non_composition_containers"]
    )

    node_ids = {node.id for node in graph.nodes}
    assert all(
        edge.source in node_ids and edge.target in node_ids for edge in graph.edges
    )


def test_oas03_graph_preserves_behavior_order_and_nesting():
    graph = analyze_scenic_composition(
        "examples/driving/OAS_Scenarios/oas_scenario_03.scenic"
    )

    assert len(graph.statements) == 2
    assert graph.statements[0].container_name == "FollowLeadCarBehavior"
    assert graph.statements[0].invocations[0].text == "FollowLaneBehavior()"
    assert graph.statements[0].nesting == ("try",)

    assert graph.statements[1].container_name == "FollowLeadCarBehavior"
    assert graph.statements[1].invocations[0].text == "CollisionAvoidance()"
    assert graph.statements[1].nesting == (
        "try",
        "interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE)",
    )


def test_oas04_graph_has_one_next_edge_only_within_follow_lead_behavior():
    graph = analyze_scenic_composition(
        "examples/driving/OAS_Scenarios/oas_scenario_04.scenic"
    )

    next_edges = [edge for edge in graph.edges if edge.kind == "next"]

    assert len(next_edges) == 1
    assert next_edges[0].source == "FollowLeadCarBehavior:1"
    assert next_edges[0].target == "FollowLeadCarBehavior:2"
    assert next_edges[0].attributes["within"] == "FollowLeadCarBehavior"


def test_find_composition_statements_matches_analyze_output_for_all_examples():
    for case in GRAPH_CASES:
        path = case["path"]
        graph = analyze_scenic_composition(path)
        statements = find_composition_statements(path)

        assert statements == graph.statements


@pytest.mark.parametrize(
    "case",
    GRAPH_CASES,
    ids=[f"{Path(case['path']).stem}-json" for case in GRAPH_CASES],
)
def test_graph_json_shape_matches_analysis_output_for_all_examples(case):
    graph = analyze_scenic_composition(case["path"])

    as_dict = graph.as_dict()

    assert as_dict["source_path"] == str(Path(case["path"]).resolve())
    assert as_dict["container_names"] == case["containers"]
    assert (
        tuple(stmt["container_name"] for stmt in as_dict["statements"])
        == case["statement_containers"]
    )
    assert (
        tuple(stmt["node_id"] for stmt in as_dict["statements"])
        == case["statement_node_ids"]
    )
    assert (
        tuple(stmt["invocations"][0]["text"] for stmt in as_dict["statements"])
        == case["invocation_texts"]
    )
    assert (
        tuple(stmt["invocations"][0]["target"] for stmt in as_dict["statements"])
        == case["invocation_targets"]
    )
    assert (
        tuple(node["kind"] for node in as_dict["nodes"]).count("composition")
        == case["node_counts"]["composition"]
    )
    assert (
        tuple(edge["kind"] for edge in as_dict["edges"]).count("contains")
        == case["edge_counts"]["contains"]
    )


def test_graph_json_shape_matches_analysis_output():
    graph = analyze_scenic_composition(
        "examples/driving/OAS_Scenarios/oas_scenario_29.scenic"
    )

    as_dict = graph.as_dict()

    assert as_dict["container_names"] == ("<initial>", "EgoBehavior")
    assert as_dict["statements"][0]["container_name"] == "EgoBehavior"
    assert as_dict["nodes"][0]["kind"] == "initial"
    assert {edge["kind"] for edge in as_dict["edges"]} == {"contains", "invokes"}


def test_build_compact_graph_dict_has_simple_sxo_shape():
    graph = analyze_scenic_composition(
        "examples/driving/OAS_Scenarios/oas_scenario_03.scenic"
    )

    compact = build_compact_graph_dict(graph)

    assert set(compact.keys()) == {"nodes", "edges"}
    assert compact["nodes"]["S:behavior:FollowLeadCarBehavior"] == {
        "type": "S",
        "name": "FollowLeadCarBehavior",
        "container_kind": "behavior",
    }
    assert compact["nodes"]["X:FollowLeadCarBehavior:1"] == {
        "type": "X",
        "op": "parallel",
        "container": "FollowLeadCarBehavior",
    }
    assert compact["nodes"]["O:FollowLeadCarBehavior:2:invocation:0"] == {
        "type": "O",
        "target": "CollisionAvoidance",
    }
    assert {"source", "target", "type"} <= set(compact["edges"][0].keys())
    assert {
        (edge["source"], edge["target"], edge["type"]) for edge in compact["edges"]
    } == {
        (
            "S:behavior:FollowLeadCarBehavior",
            "X:FollowLeadCarBehavior:1",
            "S_to_X",
        ),
        (
            "X:FollowLeadCarBehavior:1",
            "O:FollowLeadCarBehavior:1:invocation:0",
            "X_to_O",
        ),
        (
            "S:behavior:FollowLeadCarBehavior",
            "X:FollowLeadCarBehavior:2",
            "S_to_X",
        ),
        (
            "X:FollowLeadCarBehavior:2",
            "O:FollowLeadCarBehavior:2:invocation:0",
            "X_to_O",
        ),
        (
            "X:FollowLeadCarBehavior:1",
            "X:FollowLeadCarBehavior:2",
            "X_to_X",
        ),
        (
            "O:FollowLeadCarBehavior:2:invocation:0",
            "S:behavior:CollisionAvoidance",
            "O_targets_S",
        ),
    }


def test_build_compact_graph_dict_adds_uniform_probabilities_for_choose():
    graph = analyze_scenic_composition(
        "behavior MainBehavior():\n"
        "    do choose Foo(), Bar()\n"
    )

    compact = build_compact_graph_dict(graph)
    choose_edges = [
        edge
        for edge in compact["edges"]
        if edge["type"] == "X_to_O"
    ]

    assert len(choose_edges) == 2
    assert {edge["probability"] for edge in choose_edges} == {0.5}
