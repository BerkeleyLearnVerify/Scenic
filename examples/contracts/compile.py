import ast
import os
from pathlib import Path

# r = open(Path(os.path.dirname(os.path.realpath(__file__))) / "test.py", 'r')
# print(ast.dump(ast.parse(r.read()), indent=2))
# breakpoint()

from inspect import cleandoc

from scenic.syntax.compiler import compileScenicAST
from scenic.syntax.parser import parse_file

filename = Path(os.path.dirname(os.path.realpath(__file__))) / "dev.contract"
scenic_ast = parse_file(filename)
python_ast, _ = compileScenicAST(scenic_ast)
print(ast.unparse(python_ast))
exec(compile(python_ast, filename, "exec"))
