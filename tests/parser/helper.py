from typing import Any
from scenic.parser import parse_string

def parse_string_helper(source: str) -> Any:
    "Parse string and return Scenic AST"
    return parse_string(source, "exec")
