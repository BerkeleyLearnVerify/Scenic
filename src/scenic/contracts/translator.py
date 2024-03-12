import scenic.contracts.veneer
from scenic.syntax.compiler import compileScenicAST
from scenic.syntax.parser import parse_file

preamble = """\
from scenic.syntax.veneer import *
from scenic.contracts.veneer import *
"""


def compileContractsFile(filename):
    namespace = {}

    # Execute preamble
    exec(compile(preamble, "<veneer>", "exec"), namespace)

    # Execute contract code
    scenic_ast = parse_file(filename)
    python_ast, _ = compileScenicAST(scenic_ast, filename=filename)

    import ast

    print(ast.unparse(python_ast))

    compiled_code = compile(python_ast, filename, "exec")
    exec(compiled_code, namespace)

    for v_stmt in scenic.contracts.veneer._verifyStatements:
        print(v_stmt.verify())
