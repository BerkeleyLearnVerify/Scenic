from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union


if __package__ in (None, ""):
    if str(Path(__file__).resolve().parent) not in sys.path:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
    from scenic_composition_analysis_helpers import (
        CompositionGraph,
        CompositionStatement,
        GraphEdge,
        GraphNode,
        Container,
        ExecutionStructure,
        SXOStructure,
        SXONode,
        SxOEdge,
        extract_from_parser,
        load_source,
        sort_node_ids,
        ordered_compositions,
        sample_composition,
    )
else:
    from .scenic_composition_analysis_helpers import (
        CompositionGraph,
        CompositionStatement,
        GraphEdge,
        GraphNode,
        Container,
        ExecutionStructure,
        SXOStructure,
        SXONode,
        SxOEdge,
        extract_from_parser,
        load_source,
        sort_node_ids,
        ordered_compositions,
        sample_composition,
    )


def build_graph(
    source_path: Optional[Path],
    source_kind: str,
    containers: Sequence[Container],
    statements: Sequence[CompositionStatement],
) -> CompositionGraph:
    """Build a composition graph from the extracted containers and composition statements.

    Args:
        source_path (Optional[Path]): The path to the source file.
        source_kind (str): The kind of the source.
        containers (Sequence[Container]): The list of containers.
        statements (Sequence[CompositionStatement]): The list of composition statements.

    Returns:
        CompositionGraph: The built composition graph.
    """
    nodes: List[GraphNode] = []
    edges: List[GraphEdge] = []
    container_ids = {}

    for container in containers:
        node_id = f"{container.kind}:{container.name}"
        container_ids[container.name] = node_id
        nodes.append(
            GraphNode(
                id=node_id,
                kind=container.kind,
                label=container.name,
                attributes={"name": container.name, "kind": container.kind},
            )
        )

    byContainer: Dict[str, List[CompositionStatement]] = {}
    for statement in statements:
        byContainer.setdefault(statement.container_name, []).append(statement)
        nodes.append(
            GraphNode(
                id=statement.node_id,
                kind="composition",
                label=statement.operator,
                attributes={
                    "container": statement.container_name,
                    "line": statement.line,
                    "nesting": list(statement.nesting),
                    **statement.semantics,
                },
            )
        )
        edges.append(
            GraphEdge(
                source=container_ids[statement.container_name],
                target=statement.node_id,
                kind="contains",
                attributes={"line": statement.line},
            )
        )
        for index, invocation in enumerate(statement.invocations):
            inv_id = f"{statement.node_id}:invocation:{index}"
            nodes.append(
                GraphNode(
                    id=inv_id,
                    kind="invocation",
                    label=invocation.target or invocation.text,
                    attributes=asdict(invocation),
                )
            )
            edges.append(
                GraphEdge(
                    source=statement.node_id,
                    target=inv_id,
                    kind="invokes",
                    attributes={"weight": invocation.weight},
                )
            )

    for container_name, container_statements in byContainer.items():
        ordered = sorted(container_statements, key=lambda stmt: stmt.line or -1)
        for left, right in zip(ordered, ordered[1:]):
            edges.append(
                GraphEdge(
                    source=left.node_id,
                    target=right.node_id,
                    kind="next",
                    attributes={"within": container_name},
                )
            )

    return CompositionGraph(
        source_path=str(source_path) if source_path else None,
        source_kind=source_kind,
        container_names=tuple(container.name for container in containers),
        statements=tuple(statements),
        nodes=tuple(nodes),
        edges=tuple(edges),
    )


def analyze_scenic_composition(source: Union[str, Path]) -> CompositionGraph:
    """Analyze the composition structure of a Scenic source.

    Args:
        source (Union[str, Path]): The Scenic source to analyze.

    Returns:
        CompositionGraph: The analyzed composition graph.
    """

    text, source_path, source_kind = load_source(source)
    containers, statements = extract_from_parser(text)

    return build_graph(
        source_path=source_path,
        source_kind=source_kind,
        containers=containers,
        statements=statements,
    )


def find_composition_statements(
    source: Union[str, Path],
) -> Tuple[CompositionStatement, ...]:
    """Find all composition statements in a Scenic source.

    Args:
        source (Union[str, Path]): The Scenic source to analyze.

    Returns:
        Tuple[CompositionStatement, ...]: The list of composition statements.
    """
    return analyze_scenic_composition(source).statements


def build_execution_structure(graph: CompositionGraph) -> ExecutionStructure:
    """Build an execution structure from the composition graph, organizing containers, compositions, and invocations in a way that reflects the execution semantics of the composition graph.

    Args:
        graph (CompositionGraph): The composition graph to convert.

    Returns:
        ExecutionStructure: The execution structure.
    """

    node_by_id = {node.id: node for node in graph.nodes}
    contains_by_container: Dict[str, List[str]] = {}
    invokes_by_composition: Dict[str, List[str]] = {}
    next_by_composition: Dict[str, List[str]] = {}
    incoming_next: Dict[str, int] = {}

    for edge in graph.edges:
        if edge.kind == "contains":
            contains_by_container.setdefault(edge.source, []).append(edge.target)
        elif edge.kind == "invokes":
            invokes_by_composition.setdefault(edge.source, []).append(edge.target)
        elif edge.kind == "next":
            next_by_composition.setdefault(edge.source, []).append(edge.target)
            incoming_next[edge.target] = incoming_next.get(edge.target, 0) + 1

    containers: Dict[str, Dict[str, Any]] = {}
    container_ids: List[str] = []
    for node in graph.nodes:
        if node.kind not in {"initial", "scenario", "behavior"}:
            continue
        container_ids.append(node.id)
        composition_ids = sort_node_ids(
            contains_by_container.get(node.id, []), node_by_id
        )
        start_ids = sort_node_ids(
            [
                node_id
                for node_id in composition_ids
                if incoming_next.get(node_id, 0) == 0
            ],
            node_by_id,
        )
        containers[node.id] = {
            "node": node,
            "composition_ids": composition_ids,
            "start_ids": start_ids,
        }

    compositions: Dict[str, Dict[str, Any]] = {}
    for node in graph.nodes:
        if node.kind != "composition":
            continue
        compositions[node.id] = {
            "node": node,
            "invocation_ids": sort_node_ids(
                invokes_by_composition.get(node.id, []), node_by_id
            ),
            "next_ids": sort_node_ids(next_by_composition.get(node.id, []), node_by_id),
        }

    invocations = {node.id: node for node in graph.nodes if node.kind == "invocation"}

    return ExecutionStructure(
        container_ids=tuple(container_ids),
        containers=containers,
        compositions=compositions,
        invocations=invocations,
        node_by_id=node_by_id,
    )


def build_sxo_structure(graph: CompositionGraph) -> SXOStructure:
    """Build an SxO structure from the composition graph, where scenarios and behaviors are represented as S nodes, compositions as X nodes, and invocations as O nodes. Edges represent the relationships between these nodes based on the "contains", "invokes", and "next" edges in the composition graph, as well as additional edges from invocations to their target containers.

    Args:
        graph (CompositionGraph): The composition graph to convert.

    Returns:
        SXOStructure: The simplified composition graph structure.
    """

    sxo_nodes: List[SXONode] = []
    sxo_edges: List[SxOEdge] = []
    container_name_to_s_id: Dict[str, str] = {}

    for node in graph.nodes:
        if node.kind in {"initial", "scenario", "behavior"}:
            s_id = f"S:{node.id}"
            container_name_to_s_id[node.label] = s_id
            sxo_nodes.append(
                SXONode(
                    id=s_id,
                    kind="S",
                    label=node.label,
                    attributes={"container_kind": node.kind, **node.attributes},
                )
            )
        elif node.kind == "composition":
            sxo_nodes.append(
                SXONode(
                    id=f"X:{node.id}",
                    kind="X",
                    label=node.label,
                    attributes=dict(node.attributes),
                )
            )
        elif node.kind == "invocation":
            sxo_nodes.append(
                SXONode(
                    id=f"O:{node.id}",
                    kind="O",
                    label=node.label,
                    attributes=dict(node.attributes),
                )
            )

    for edge in graph.edges:
        if edge.kind == "contains":
            sxo_edges.append(
                SxOEdge(
                    source=f"S:{edge.source}",
                    target=f"X:{edge.target}",
                    kind="S_to_X",
                    attributes=dict(edge.attributes),
                )
            )
        elif edge.kind == "invokes":
            sxo_edges.append(
                SxOEdge(
                    source=f"X:{edge.source}",
                    target=f"O:{edge.target}",
                    kind="X_to_O",
                    attributes=dict(edge.attributes),
                )
            )
        elif edge.kind == "next":
            sxo_edges.append(
                SxOEdge(
                    source=f"X:{edge.source}",
                    target=f"X:{edge.target}",
                    kind="X_to_X",
                    attributes=dict(edge.attributes),
                )
            )

    for node in graph.nodes:
        if node.kind != "invocation":
            continue
        target = node.attributes.get("target")
        if target in container_name_to_s_id:
            sxo_edges.append(
                SxOEdge(
                    source=f"O:{node.id}",
                    target=container_name_to_s_id[target],
                    kind="O_targets_S",
                    attributes={},
                )
            )

    return SXOStructure(nodes=tuple(sxo_nodes), edges=tuple(sxo_edges))


def sample_from_graph(graph: CompositionGraph) -> List[str]:
    """Sample a possible execution trace from the composition graph by traversing the execution structure and making random choices at composition and invocation nodes.

    Args:
        graph (CompositionGraph): The composition graph to sample from.

    Returns:
        List[str]: A sample execution trace based on the composition structure of the graph.
    """

    execution = build_execution_structure(graph)
    trace: List[str] = []

    for container_id in execution.container_ids:
        container = execution.containers[container_id]
        if container["node"].kind not in {"scenario", "behavior"}:
            continue
        for composition_id in ordered_compositions(container, execution):
            trace.extend(
                sample_composition(
                    execution.compositions[composition_id], execution.invocations
                )
            )

    return trace


def run_scenic_composition(source: Union[str, Path]) -> List[str]:
    """Run the composition structure of a Scenic source and return a sample execution trace.

    Args:
        source (Union[str, Path]): The Scenic source to run.

    Returns:
        List[str]: A sample execution trace based on the composition structure of the source.
    """

    graph = analyze_scenic_composition(source)
    return sample_from_graph(graph)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(
            "usage: python tools/scenic_composition_analysis.py <scenic-file>"
        )
    graph = analyze_scenic_composition(sys.argv[1])
    execution = build_execution_structure(graph)
    sxo = build_sxo_structure(graph)
    sample = sample_from_graph(graph)

    print(
        json.dumps(
            {
                "graph": graph.as_dict(),
                "execution_structure": execution.as_dict(),
                "sxo_structure": sxo.as_dict(),
                "sample_trace": sample,
            },
            indent=2,
        )
    )
