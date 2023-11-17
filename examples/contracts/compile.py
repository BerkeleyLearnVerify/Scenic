import ast
import os
from pathlib import Path

# r = open(Path(os.path.dirname(os.path.realpath(__file__))) / "test.py", 'r')
# print(ast.dump(ast.parse(r.read()), indent=2))
# breakpoint()

# import random
# import numpy
# from scenic.syntax.translator import scenarioFromFile
# SEED=2
# random.seed(SEED)
# numpy.random.seed(SEED)
# scenario = scenarioFromFile(Path(os.path.dirname(os.path.realpath(__file__))) / "highway.scenic", mode2D=True)
# scene, _ = scenario.generate()
# simulator = scenario.getSimulator()
# simulation = simulator.simulate(scene, maxSteps=50, manual=True, verbosity=3)

# while True:
# 	simulation.advance()
# 	if simulation.terminationType:
# 		print(simulation.result)
# 		breakpoint()

from inspect import cleandoc

from scenic.syntax.compiler import compileScenicAST
from scenic.syntax.parser import parse_file

filename = Path(os.path.dirname(os.path.realpath(__file__))) / "dev.contract"
scenic_ast = parse_file(filename)
python_ast, _ = compileScenicAST(scenic_ast)
print(ast.unparse(python_ast))
exec(compile(python_ast, filename, "exec"))
