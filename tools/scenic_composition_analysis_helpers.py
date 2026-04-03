from __future__ import annotations

import ast
from dataclasses import asdict, dataclass, field
from pathlib import Path
import re
import sys
import random
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from scenic.syntax import ast as scenic_ast
from scenic.syntax.parser import parse_string


@dataclass(frozen=True)
class InvocationSpec:
    """Represents a single invocation of a scenario or behavior within a composition statement."""

    text: str
    target: Optional[str]
    weight: Optional[float] = None
    is_weighted: bool = False
    line: Optional[int] = None


@dataclass(frozen=True)
class CompositionStatement:
    """Represents a single composition statement in the Scenic source."""

    node_id: str
    container_name: str
    operator: str
    semantics: Dict[str, Any]
    invocations: Tuple[InvocationSpec, ...]
    line: Optional[int] = None
    nesting: Tuple[str, ...] = ()


@dataclass(frozen=True)
class GraphNode:
    """Represents a node in the composition graph."""

    id: str
    kind: str
    label: str
    attributes: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class GraphEdge:
    """Represents an edge in the composition graph."""

    source: str
    target: str
    kind: str
    attributes: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CompositionGraph:
    """Represents the entire composition graph."""

    source_path: Optional[str]
    source_kind: str
    container_names: Tuple[str, ...]
    statements: Tuple[CompositionStatement, ...]
    nodes: Tuple[GraphNode, ...]
    edges: Tuple[GraphEdge, ...]

    def as_dict(self) -> Dict[str, Any]:
        """Convert the composition graph to a dictionary.

        Returns:
            Dict[str, Any]: The dictionary representation of the composition graph.
        """
        return asdict(self)


@dataclass(frozen=True)
class Container:
    name: str
    kind: str
    body: Sequence[Any]


@dataclass(frozen=True)
class SXONode:
    id: str
    kind: str
    label: str
    attributes: Dict[str, Any]


@dataclass(frozen=True)
class SxOEdge:
    source: str
    target: str
    kind: str
    attributes: Dict[str, Any]


@dataclass(frozen=True)
class SXOStructure:
    nodes: Tuple[SXONode, ...]
    edges: Tuple[SxOEdge, ...]

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ExecutionStructure:
    container_ids: Tuple[str, ...]
    containers: Dict[str, Dict[str, Any]]
    compositions: Dict[str, Dict[str, Any]]
    invocations: Dict[str, GraphNode]
    node_by_id: Dict[str, GraphNode]

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


OPERATOR_INFO = {
    scenic_ast.Do: ("parallel", "parallel-all"),
    scenic_ast.DoChoose: ("choose", "single-enabled-choice"),
    scenic_ast.DoShuffle: ("shuffle", "random-permutation-of-enabled-choices"),
}


def load_source(source: Union[str, Path]) -> Tuple[str, Optional[Path], str]:
    """Load the source text and metadata.

    Args:
        source (Union[str, Path]): The Scenic source to load.

    Returns:
        Tuple[str, Optional[Path], str]: The source text, path, and kind.
    """

    if isinstance(source, Path):
        return source.read_text(encoding="utf-8"), source.resolve(), "path"
    candidate = Path(source).expanduser()
    if "\n" not in source and candidate.exists():
        return candidate.read_text(encoding="utf-8"), candidate.resolve(), "path"
    return str(source), None, "string"


def extract_from_parser(
    text: str,
) -> Tuple[List[Container], List[CompositionStatement]]:
    """Extract containers and composition statements from the parsed text.

    Args:
        text (str): The parsed text.

    Returns:
        Tuple[List[Container], List[CompositionStatement]]: The list of containers and composition statements.
    """

    module = parse_string(text, "exec", filename="<analysis>")
    containers = [Container(name="<initial>", kind="initial", body=module.body)]

    for node in module.body:
        if isinstance(node, scenic_ast.ScenarioDef):
            containers.append(Container(node.name, "scenario", node.compose or ()))
        elif isinstance(node, scenic_ast.BehaviorDef):
            containers.append(Container(node.name, "behavior", node.body or ()))

    statements: List[CompositionStatement] = []
    for container in containers:
        statements.extend(collect_statements(container))
    return containers, statements


def collect_statements(container: Container) -> List[CompositionStatement]:
    """Collect composition statements from a container.

    Args:
        container (Container): The container from which to collect statements.

    Returns:
        List[CompositionStatement]: The list of composition statements.
    """
    statements: List[CompositionStatement] = []

    def visit_block(block: Sequence[Any], nesting: Tuple[str, ...] = ()) -> None:
        for node in block:
            operator = operator_of(node)
            if operator:
                statements.append(
                    CompositionStatement(
                        node_id=f"{container.name}:{len(statements) + 1}",
                        container_name=container.name,
                        operator=operator,
                        semantics=operator_semantics(operator, container.kind),
                        invocations=tuple(extract_invocations(node)),
                        line=getattr(node, "lineno", None),
                        nesting=nesting,
                    )
                )
            if is_control_flow_node(node):
                label = control_label(node)
                next_nesting = nesting + ((label,) if label else ())
                child_body = getattr(node, "body", None)
                if child_body:
                    visit_block(child_body, next_nesting)
                for extra_block, extra_label in extra_control_blocks(node):
                    visit_block(
                        extra_block,
                        next_nesting + ((extra_label,) if extra_label else ()),
                    )

    visit_block(container.body)
    return statements


def operator_of(node: Any) -> Optional[str]:
    """Determine the composition operator of a node, if any.

    Args:
        node (Any): The node for which to determine the operator.

    Returns:
        Optional[str]: The composition operator, if any.
    """
    for node_type, (operator, _) in OPERATOR_INFO.items():
        if isinstance(node, node_type):
            return operator
    return None


def is_control_flow_node(node: Any) -> bool:
    """Determine if a node is a control flow node (e.g., if, while, for, try).

    Args:
        node (Any): The node for which to determine if it's a control flow node.

    Returns:
        bool: True if the node is a control flow node, False otherwise.
    """
    return isinstance(
        node, (ast.While, ast.If, ast.For, ast.Try, scenic_ast.TryInterrupt)
    )


def extract_invocations(node: Any) -> Iterable[InvocationSpec]:
    """Extract invocation specifications from a node.

    Args:
        node (Any): The node from which to extract invocations.

    Returns:
        Iterable[InvocationSpec]: The list of invocation specifications.

    Yields:
        Iterator[Iterable[InvocationSpec]]: An iterator over invocation specifications.
    """

    for expr in getattr(node, "elts", ()):
        if isinstance(expr, ast.Dict):
            for key, value in zip(expr.keys or (), expr.values or ()):
                key_text = ast.unparse(key)
                yield InvocationSpec(
                    text=key_text,
                    target=target_name(key_text),
                    weight=parse_weight(ast.unparse(value)),
                    is_weighted=True,
                    line=getattr(node, "lineno", None),
                )
        else:
            text = ast.unparse(expr)
            yield InvocationSpec(
                text=text,
                target=target_name(text),
                line=getattr(node, "lineno", None),
            )


def control_label(node: Any) -> Optional[str]:
    """Determine the control flow label of a node, if any.

    Args:
        node (Any): The node for which to determine the label.

    Returns:
        Optional[str]: The control flow label, if any.
    """
    if isinstance(node, ast.While):
        return f"while {ast.unparse(node.test)}"
    if isinstance(node, ast.If):
        return f"if {ast.unparse(node.test)}"
    if isinstance(node, ast.For):
        return f"for {ast.unparse(node.target)} in {ast.unparse(node.iter)}"
    if isinstance(node, (ast.Try, scenic_ast.TryInterrupt)):
        return "try"
    return None


def extra_control_blocks(node: Any) -> Iterable[Tuple[Sequence[Any], Optional[str]]]:
    """Determine any extra control flow blocks associated with a node, such as except/else/finally for try statements.

    Args:
        node (Any): The node for which to determine extra control flow blocks.

    Returns:
        Iterable[Tuple[Sequence[Any], Optional[str]]]: The list of extra control flow blocks.

    Yields:
        Iterator[Iterable[Tuple[Sequence[Any], Optional[str]]]]: An iterator over extra control flow blocks.
    """

    if isinstance(node, ast.Try):
        for handler in node.handlers:
            yield handler.body, "except"
        if node.orelse:
            yield node.orelse, "else"
        if node.finalbody:
            yield node.finalbody, "finally"
    elif isinstance(node, scenic_ast.TryInterrupt):
        for handler in node.interrupt_when_handlers:
            yield handler.body, f"interrupt when {ast.unparse(handler.cond)}"
        if node.orelse:
            yield node.orelse, "else"
        if node.finalbody:
            yield node.finalbody, "finally"


def target_name(text: str) -> Optional[str]:
    """Extract the target name from a text string.

    Args:
        text (str): The text string from which to extract the target name.

    Returns:
        Optional[str]: The extracted target name, or None if not found.
    """

    match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(?:\(|$)", text.strip())
    return match.group(1) if match else None


def parse_weight(text: str) -> Optional[float]:
    """Parse a weight from a text string.

    Args:
        text (str): The text string from which to parse the weight.

    Returns:
        Optional[float]: The parsed weight, or None if not found.
    """

    try:
        return float(text)
    except ValueError:
        return None


def operator_semantics(operator: str, container_kind: str) -> Dict[str, Any]:
    """Determine the semantics of a composition operator based on its type and container kind.

    Args:
        operator (str): The composition operator.
        container_kind (str): The kind of the container.

    Returns:
        Dict[str, Any]: The semantics of the composition operator.
    """

    execution = next(
        execution for op, execution in OPERATOR_INFO.values() if op == operator
    )
    return {
        "container_kind": container_kind,
        "execution": execution,
        "randomized": operator in {"choose", "shuffle"},
        "weighted": operator in {"choose", "shuffle"},
    }


def sort_node_ids(
    node_ids: Sequence[str], node_by_id: Dict[str, GraphNode]
) -> List[str]:
    return sorted(
        node_ids,
        key=lambda node_id: (
            node_by_id[node_id].attributes.get("line")
            if node_by_id[node_id].attributes.get("line") is not None
            else -1,
            node_id,
        ),
    )


def ordered_compositions(
    container: Dict[str, Any], execution: ExecutionStructure
) -> List[str]:
    ordered: List[str] = []
    visited = set()
    queue = list(container["start_ids"])
    composition_ids = set(container["composition_ids"])

    while queue:
        current = queue.pop(0)
        if current in visited:
            continue
        visited.add(current)
        ordered.append(current)
        queue.extend(
            next_id
            for next_id in execution.compositions[current]["next_ids"]
            if next_id in composition_ids
        )

    ordered.extend(
        node_id for node_id in container["composition_ids"] if node_id not in visited
    )
    return ordered


def sample_composition(
    composition: Dict[str, Any], invocation_nodes: Dict[str, GraphNode]
) -> List[str]:
    operator = composition["node"].label
    invocation_ids = list(composition["invocation_ids"])

    if not invocation_ids:
        return []
    if operator == "parallel":
        chosen = invocation_ids
    elif operator == "choose":
        chosen = [choose_invocation(invocation_ids, invocation_nodes)]
    elif operator == "shuffle":
        chosen = invocation_ids
        random.shuffle(chosen)
    else:
        raise ValueError(f"unknown operator {operator!r}")

    return [invocation_result(invocation_nodes[node_id]) for node_id in chosen]


def choose_invocation(
    invocation_ids: Sequence[str], invocation_nodes: Dict[str, GraphNode]
) -> str:
    weights = [
        invocation_nodes[node_id].attributes.get("weight") for node_id in invocation_ids
    ]
    if any(weight is not None for weight in weights):
        normalized = [weight if weight is not None else 1.0 for weight in weights]
        return random.choices(list(invocation_ids), weights=normalized, k=1)[0]
    return random.choice(list(invocation_ids))


def invocation_result(invocation: GraphNode) -> str:
    return invocation.attributes.get("target") or invocation.attributes["text"]
