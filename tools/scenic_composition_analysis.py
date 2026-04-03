from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple, Union

if __package__ in (None, ""):
    if str(Path(__file__).resolve().parent) not in sys.path:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
    from scenic_composition_analysis_helpers import (
        CompositionGraph,
        CompositionStatement,
        GraphEdge,
        GraphNode,
        Container,
        extract_from_parser,
        load_source,
    )
else:
    from .scenic_composition_analysis_helpers import (
        CompositionGraph,
        CompositionStatement,
        GraphEdge,
        GraphNode,
        Container,
        extract_from_parser,
        load_source,
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


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(
            "usage: python tools/scenic_composition_analysis.py <scenic-file>"
        )
    print(json.dumps(analyze_scenic_composition(sys.argv[1]).as_dict(), indent=2))
