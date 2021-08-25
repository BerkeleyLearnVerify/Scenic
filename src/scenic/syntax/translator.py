
"""Translator turning Scenic programs into Scenario objects.

The top-level interface to Scenic is provided by two functions:

* `scenarioFromString` -- compile a string of Scenic code;
* `scenarioFromFile` -- compile a Scenic file.

These output a `Scenario` object, from which scenes can be generated.
See the documentation for `Scenario` for details.

When imported, this module hooks the Python import system so that Scenic
modules can be imported using the ``import`` statement. This is primarily for the
translator's own use, but you could import Scenic modules from Python to
inspect them. Because Scenic uses Python's import system, the latter's rules
for finding modules apply, including the handling of packages.

Scenic is compiled in two main steps: translating the code into Python, and
executing the resulting Python module to generate a Scenario object encoding
the objects, distributions, etc. in the scenario. For details, see the function
`compileStream` below.
"""

import sys
import os
import io
import builtins
import time
import inspect
import types
import typing
import importlib
import importlib.abc
import importlib.util
import itertools
import pathlib
from collections import namedtuple, defaultdict
from contextlib import contextmanager

import tokenize
from tokenize import NAME, NL, NEWLINE, ENDMARKER, OP, NUMBER, COLON, COMMENT, ENCODING
from tokenize import LPAR, RPAR, LSQB, RSQB, RBRACE, COMMA, DOUBLESLASH, DOUBLESLASHEQUAL
from tokenize import AT, LEFTSHIFT, RIGHTSHIFT, VBAR, AMPER, TILDE, CIRCUMFLEX, STAR
from tokenize import LEFTSHIFTEQUAL, RIGHTSHIFTEQUAL, VBAREQUAL, AMPEREQUAL, CIRCUMFLEXEQUAL
from tokenize import INDENT, DEDENT, STRING, SEMI

import ast
from ast import parse, dump, NodeVisitor, NodeTransformer, copy_location, fix_missing_locations
from ast import Load, Store, Del, Name, Call, Tuple, BinOp, MatMult, BitAnd, BitOr, BitXor, LShift
from ast import RShift, Starred, Lambda, AnnAssign, Set, Str, Subscript, Index, IfExp
from ast import NamedExpr, Yield, YieldFrom, FunctionDef, Attribute, Constant, Assign, Expr
from ast import Return, Raise, If, UnaryOp, Not, ClassDef, Nonlocal, Global, Compare, Is, Try
from ast import Break, Continue, AsyncFunctionDef, Pass, While

from scenic.core.distributions import Samplable, needsSampling, toDistribution
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.object_types import _Constructible
import scenic.core.errors as errors
from scenic.core.errors import (TokenParseError, PythonParseError, ASTParseError,
								InvalidScenarioError)
import scenic.core.dynamics as dynamics
import scenic.core.pruning as pruning
import scenic.syntax.veneer as veneer

### THE TOP LEVEL: compiling a Scenic program

def scenarioFromString(string, params={}, model=None, scenario=None,
					   filename='<string>', cacheImports=False):
	"""Compile a string of Scenic code into a `Scenario`.

	The optional **filename** is used for error messages."""
	stream = io.BytesIO(string.encode())
	return scenarioFromStream(stream, params=params, model=model, scenario=scenario,
							  filename=filename, cacheImports=cacheImports)

def scenarioFromFile(path, params={}, model=None, scenario=None, cacheImports=False):
	"""Compile a Scenic file into a `Scenario`.

	Args:
		path (str): path to a Scenic file
		params (dict): global parameters to override
		model (str): Scenic module to use as world model
		scenario (str): if there are multiple scenarios in the file, which one
		  to use
		cacheImports (bool): Whether to cache any imported Scenic modules.
		  The default behavior is to not do this, so that subsequent attempts
		  to import such modules will cause them to be recompiled. If it is
		  safe to cache Scenic modules across multiple compilations, set this
		  argument to True. Then importing a Scenic module will have the same
		  behavior as importing a Python module.

	Returns:
		A `Scenario` object representing the Scenic scenario.
	"""
	if not os.path.exists(path):
		raise FileNotFoundError(path)
	fullpath = os.path.realpath(path)
	head, extension = os.path.splitext(fullpath)
	if not extension or extension[1:] not in scenicExtensions:
		ok = ', '.join(scenicExtensions)
		err = f'Scenic scenario does not have valid extension ({ok})'
		raise RuntimeError(err)
	directory, name = os.path.split(head)

	with open(path, 'rb') as stream:
		return scenarioFromStream(stream, params=params, model=model, scenario=scenario,
								  filename=fullpath, path=path, cacheImports=cacheImports)

def scenarioFromStream(stream, params={}, model=None, scenario=None,
					   filename='<stream>', path=None, cacheImports=False):
	"""Compile a stream of Scenic code into a `Scenario`."""
	# Compile the code as if it were a top-level module
	oldModules = list(sys.modules.keys())
	try:
		with topLevelNamespace(path) as namespace:
			compileStream(stream, namespace, params=params, model=model, filename=filename)
	finally:
		if not cacheImports:
			toRemove = []
			for name, module in sys.modules.items():
				if name not in oldModules and getattr(module, '_isScenicModule', False):
					toRemove.append(name)
			for name in toRemove:
				del sys.modules[name]
	# Construct a Scenario from the resulting namespace
	return constructScenarioFrom(namespace, scenario)

@contextmanager
def topLevelNamespace(path=None):
	"""Creates an environment like that of a Python script being run directly.

	Specifically, __name__ is '__main__', __file__ is the path used to invoke
	the script (not necessarily its absolute path), and the parent directory is
	added to the path so that 'import blobbo' will import blobbo from that
	directory if it exists there.
	"""
	directory = os.getcwd() if path is None else os.path.dirname(path)
	namespace = { '__name__': '__main__' }
	if path is not None:
		namespace['__file__'] = path
	sys.path.insert(0, directory)
	try:
		yield namespace
	finally:
		del sys.path[0]

def compileStream(stream, namespace, params={}, model=None, filename='<stream>'):
	"""Compile a stream of Scenic code and execute it in a namespace.

	The compilation procedure consists of the following main steps:

		1. Tokenize the input using the Python tokenizer.
		2. Partition the tokens into blocks separated by import statements.
		   This is done by the `partitionByImports` function.
		3. Translate Scenic constructions into valid Python syntax.
		   This is done by the `TokenTranslator`.
		4. Parse the resulting Python code into an AST using the Python parser.
		5. Modify the AST to achieve the desired semantics for Scenic.
		   This is done by the `translateParseTree` function.
		6. Compile and execute the modified AST.
		7. After executing all blocks, extract the global state (e.g. objects).
		   This is done by the `storeScenarioStateIn` function.
	"""
	if verbosity >= 2:
		veneer.verbosePrint(f'  Compiling Scenic module from {filename}...')
		startTime = time.time()
	# Tokenize input stream
	try:
		tokens = list(tokenize.tokenize(stream.readline))
	except tokenize.TokenError as e:
		line = e.args[1][0] if isinstance(e.args[1], tuple) else e.args[1]
		raise TokenParseError(line, filename, 'file ended during multiline string or expression')
	# Partition into blocks with all imports at the end (since imports could
	# pull in new constructor (Scenic class) definitions, which change the way
	# subsequent tokens are transformed)
	blocks = partitionByImports(tokens)
	veneer.activate(params, model, filename, namespace)
	newSourceBlocks = []
	try:
		# Execute preamble
		exec(compile(preamble, '<veneer>', 'exec'), namespace)
		namespace[namespaceReference] = namespace
		# Execute each block
		for blockNum, block in enumerate(blocks):
			# Find all custom constructors defined so far (possibly imported)
			constructors = findConstructorsIn(namespace)
			# Translate tokens to valid Python syntax
			startLine = max(1, block[0][2][0])
			translator = TokenTranslator(constructors, filename)
			newSource, allConstructors = translator.translate(block)
			trimmed = newSource[2*(startLine-1):]	# fix up blank lines used to align errors
			newSource = '\n'*(startLine-1) + trimmed
			newSourceBlocks.append(trimmed)
			if dumpTranslatedPython:
				print(f'### Begin translated Python from block {blockNum} of {filename}')
				print(newSource)
				print('### End translated Python')
			# Parse the translated source
			tree = parseTranslatedSource(newSource, filename)
			# Modify the parse tree to produce the correct semantics
			newTree, requirements = translateParseTree(tree, allConstructors, filename)
			if dumpFinalAST:
				print(f'### Begin final AST from block {blockNum} of {filename}')
				print(ast.dump(newTree, include_attributes=True))
				print('### End final AST')
			if dumpASTPython:
				try:
					import astor
				except ModuleNotFoundError as e:
					raise RuntimeError('dumping the Python equivalent of the AST'
									   'requires the astor package')
				print(f'### Begin Python equivalent of final AST from block {blockNum} of {filename}')
				print(astor.to_source(newTree, add_line_information=True))
				print('### End Python equivalent of final AST')
			# Compile the modified tree
			code = compileTranslatedTree(newTree, filename)
			# Execute it
			executeCodeIn(code, namespace)
		# Extract scenario state from veneer and store it
		storeScenarioStateIn(namespace, requirements)
	finally:
		veneer.deactivate()
	if verbosity >= 2:
		totalTime = time.time() - startTime
		veneer.verbosePrint(f'  Compiled Scenic module in {totalTime:.4g} seconds.')
	allNewSource = ''.join(newSourceBlocks)
	return code, allNewSource

### TRANSLATION PHASE ZERO: definitions of language elements not already in Python

## Options

dumpTranslatedPython = False
dumpFinalAST = False
dumpASTPython = False
verbosity = 0
usePruning = True

## Preamble
# (included at the beginning of every module to be translated;
# imports the implementations of the public language features)
preamble = """\
from scenic.syntax.veneer import *
"""

## Get Python names of various elements
## (for checking consistency between the translator and the veneer)

api = set(veneer.__all__)

## Functions used internally

createDefault = 'PropertyDefault'
scenarioClass = 'DynamicScenario'
behaviorClass = 'Behavior'
monitorClass = 'Monitor'
createTerminationAction = 'makeTerminationAction'
internalFunctions = {
	createDefault, scenarioClass,
	behaviorClass, createTerminationAction,
}

# sanity check: these functions actually exist
for imp in internalFunctions:
	assert imp in api, imp

## Built-in functions

builtinFunctions = { 'resample', 'verbosePrint', 'simulation', 'localPath' }

# sanity check: implementations of built-in functions actually exist
for imp in builtinFunctions:
	assert imp in api, imp

## Built-in names (values which cannot be overwritten)

globalParametersName = 'globalParameters'

builtinNames = { globalParametersName }

# sanity check: built-in names actually exist
for name in builtinNames:
	assert name in api, name

## Simple statements

paramStatement = 'param'
mutateStatement = 'mutate'

requireStatement = 'require'
softRequirement = 'require_soft'	# not actually a statement, but a marker for the parser
requireAlwaysStatement = ('require', 'always')
terminateWhenStatement = ('terminate', 'when')
terminateSimulationWhenStatement = ('terminate', 'simulation', 'when')
terminateAfterStatement = ('terminate', 'after')

actionStatement = 'take'			# statement invoking a primitive action
waitStatement = 'wait'				# statement invoking a no-op action
terminateStatement = 'terminate'	# statement ending the simulation
abortStatement = 'abort'			# statement ending a try-interrupt statement
invokeStatement = 'do'				# statement invoking a behavior or scenario

modelStatement = 'model'
namespaceReference = '_Scenic_module_namespace'		# used in the implementation of 'model'

simulatorStatement = 'simulator'

oneWordStatements = {	# TODO clean up
	paramStatement, mutateStatement, requireStatement,
	actionStatement, waitStatement, terminateStatement,
	abortStatement, invokeStatement, simulatorStatement
}
twoWordStatements = {
	requireAlwaysStatement, terminateWhenStatement, terminateAfterStatement,
}
threeWordStatements = { terminateSimulationWhenStatement }

threeWordIncipits = { tokens[:2]: tokens[2] for tokens in threeWordStatements } # TODO improve
for incipit, last in threeWordIncipits.items():
	for tok2 in twoWordStatements:
		assert tok2 != incipit, (tok2, last)

# statements implemented by functions
functionStatements = {
	requireStatement, paramStatement, mutateStatement,
	modelStatement, simulatorStatement
}
twoWordFunctionStatements = {
	requireAlwaysStatement, terminateWhenStatement, terminateAfterStatement,
}
threeWordFunctionStatements = { terminateSimulationWhenStatement }
def functionForStatement(tokens):
	return '_'.join(tokens) if isinstance(tokens, tuple) else tokens
def nameForStatement(tokens):
	return ' '.join(tokens) if isinstance(tokens, tuple) else tokens

statementForImp = {
	functionForStatement(s): nameForStatement(s)
	for s in functionStatements | twoWordStatements
}

# sanity check: implementations actually exist
for imp in functionStatements:
	assert imp in api, imp
for tokens in twoWordFunctionStatements:
	assert len(tokens) == 2
	imp = functionForStatement(tokens)
	assert imp in api, imp
for tokens in threeWordFunctionStatements:
	assert len(tokens) == 3
	imp = functionForStatement(tokens)
	assert imp in api, imp

# statements allowed inside behaviors
behavioralStatements = {
	requireStatement, actionStatement, waitStatement,
	terminateStatement, abortStatement, invokeStatement,
}
behavioralImps = { functionForStatement(s) for s in behavioralStatements }

# statements allowed inside scenario composition blocks
compositionalStatements = {
	requireStatement, waitStatement, terminateStatement, abortStatement,
	invokeStatement,
}
compositionalImps = { functionForStatement(s) for s in compositionalStatements }

# statements encoding requirements which need special handling
requirementStatements = (
	requireStatement, softRequirement,
	functionForStatement(requireAlwaysStatement),
	functionForStatement(terminateWhenStatement),
)

statementRaiseMarker = '_Scenic_statement_'

## Try-interrupt blocks

interruptWhenStatement = ('interrupt', 'when')

interruptExceptMarker = '_Scenic_interrupt_'

## Constructors and specifiers

# statement defining a new constructor (Scenic class);
# we still recognize 'constructor' for backwards-compatibility
constructorStatements = ('class', 'constructor')

Constructor = namedtuple('Constructor', ('name', 'bases'))

builtinSpecifiers = {
	# position
	('visible', 'from'): 'VisibleFrom',
	('offset', 'by'): 'OffsetBy',
	('offset', 'along'): 'OffsetAlongSpec',
	('at',): 'At',
	('in',): 'In',
	('on',): 'In',
	('beyond',): 'Beyond',
	('visible',): 'VisibleSpec',
	('left', 'of'): 'LeftSpec',
	('right', 'of'): 'RightSpec',
	('ahead', 'of'): 'Ahead',
	('behind',): 'Behind',
	('following',): 'Following',

	# heading
	('apparently', 'facing'): 'ApparentlyFacing',
	('facing', 'toward'): 'FacingToward',
	('facing',): 'Facing'
}

# sanity check: implementations of specifiers actually exist
for imp in builtinSpecifiers.values():
	assert imp in api, imp

builtinConstructors = {
	'Point': Constructor('Point', None),
	'OrientedPoint': Constructor('OrientedPoint', 'Point'),
	'Object': Constructor('Object', 'OrientedPoint')
}

# sanity check: built-in constructors actually exist
for const in builtinConstructors:
	assert const in api, const

## Other compound statements

behaviorStatement = 'behavior'		# statement defining a new behavior
monitorStatement = 'monitor'		# statement defining a new monitor

scenarioMarker = '_Scenic_scenario_'	# not a statement, but a marker for the parser

# Scenario blocks
setupBlock = 'setup'
composeBlock = 'compose'
scenarioBlocks = { setupBlock, composeBlock, 'precondition', 'invariant' }

## Prefix operators

prefixOperators = {
	('relative', 'position'): 'RelativePosition',
	('relative', 'heading'): 'RelativeHeading',
	('apparent', 'heading'): 'ApparentHeading',
	('distance', 'from'): 'DistanceFrom',
	('distance', 'to'): 'DistanceFrom',
	('angle', 'from'): 'AngleFrom',
	('angle', 'to'): 'AngleTo',
	('ego', '='): 'ego',
	('front', 'left'): 'FrontLeft',
	('front', 'right'): 'FrontRight',
	('back', 'left'): 'BackLeft',
	('back', 'right'): 'BackRight',
	('front', 'of'): 'Front',
	('back', 'of'): 'Back',
	('left', 'of'): 'Left',
	('right', 'of'): 'Right',
	('follow',): 'Follow',
	('visible',): 'Visible',
	('not', 'visible'): 'NotVisible',
}
assert all(1 <= len(op) <= 2 for op in prefixOperators)
prefixIncipits = { op[0] for op in prefixOperators }
assert not any(op in oneWordStatements for op in prefixIncipits)
assert not any(op in twoWordStatements for op in prefixOperators)

# sanity check: implementations of prefix operators actually exist
for imp in prefixOperators.values():
	assert imp in api, imp

## Modifiers and terminators

class ModifierInfo(typing.NamedTuple):
	name: str
	terminators: typing.Tuple[str]
	contexts: typing.Optional[typing.Tuple[str]] = ()

modifiers = (
	ModifierInfo('for', ('seconds', 'steps'), (invokeStatement,)),
	ModifierInfo('until', (), (invokeStatement,)),
)

modifierNames = {}
for mod in modifiers:
	assert mod.name not in modifierNames, mod
	modifierNames[mod.name] = mod

terminatorsForStatements = {
	functionForStatement(terminateAfterStatement): ('seconds', 'steps'),
}

## Infix operators

# pseudo-operator for encoding argument packages for (3+)-ary ops
packageToken = (RIGHTSHIFT, '>>')
packageNode = RShift

class InfixOp(typing.NamedTuple):
	syntax: str
	implementation: typing.Optional[str]
	arity: int
	token: typing.Tuple[int, str]
	node: ast.AST
	contexts: typing.Optional[typing.Tuple[str]] = ()

infixOperators = (
	# existing Python operators with new semantics
	InfixOp('@', 'Vector', 2, None, MatMult),

	# operators not in Python (in decreasing precedence order)
	InfixOp('at', 'FieldAt', 2, (LEFTSHIFT, '<<'), LShift),
	InfixOp('relative to', 'RelativeTo', 2, (AMPER, '&'), BitAnd),
	InfixOp('offset by', 'RelativeTo', 2, (AMPER, '&'), BitAnd),
	InfixOp('offset along', 'OffsetAlong', 3, (CIRCUMFLEX, '^'), BitXor),
	InfixOp('can see', 'CanSee', 2, (VBAR, '|'), BitOr),

	# just syntactic conveniences, not really operators
	InfixOp('from', None, 2, (COMMA, ','), None),
	InfixOp('for', None, 2, (COMMA, ','), None, ('Follow', 'Following')),
	InfixOp('to', None, 2, (COMMA, ','), None),
	InfixOp('by', None, 2, packageToken, None)
)

infixTokens = {}
infixImplementations = {}
infixIncipits = set()
for op in infixOperators:
	# if necessary, set up map from Scenic to Python syntax
	if op.token is not None:
		tokens = tuple(op.syntax.split(' '))
		assert 1 <= len(tokens) <= 2, op
		assert tokens not in infixTokens, op
		assert tokens not in twoWordStatements, op
		infixTokens[tokens] = op
		incipit = tokens[0]
		assert incipit not in oneWordStatements, op
		infixIncipits.add(incipit)
	# if necessary, set up map from Python to Scenic semantics
	imp = op.implementation
	if imp is not None:
		assert imp in api, op
		node = op.node
		if node in infixImplementations:	# two operators may have the same implementation
			oldArity, oldName = infixImplementations[node]
			assert op.arity == oldArity, (op, oldName)
			assert imp == oldName, (op, oldName)
		else:
			infixImplementations[node] = (op.arity, imp)
generalInfixOps = { tokens: op.token for tokens, op in infixTokens.items() if not op.contexts }

## Direct syntax replacements

replacements = {	# TODO police the usage of these? could yield bizarre error messages
	'of': tuple(),
	'deg': ((STAR, '*'), (NUMBER, '0.01745329252')),
	'ego': ((NAME, 'ego'), (LPAR, '('), (RPAR, ')')),
	'globalParameters': ((NAME, 'globalParameters'), (LPAR, '('), (RPAR, ')')),
}

twoWordReplacements = {
	('initial', 'scenario'): ((NAME, 'in_initial_scenario'), (LPAR, '('), (RPAR, ')')),
}

## Illegal and reserved syntax

illegalTokens = {
	LEFTSHIFT, RIGHTSHIFT, VBAR, AMPER, TILDE, CIRCUMFLEX,
	LEFTSHIFTEQUAL, RIGHTSHIFTEQUAL, VBAREQUAL, AMPEREQUAL, CIRCUMFLEXEQUAL,
	DOUBLESLASH, DOUBLESLASHEQUAL
}

# sanity check: stand-in tokens for infix operators must be illegal
for op in infixTokens.values():
	ttype = op.token[0]
	assert (ttype is COMMA or ttype in illegalTokens), op

illegalConstructs = {
	('async', 'def'),		# used to parse behaviors, so disallowed otherwise
}

keywords = (
	set(constructorStatements)
	| (oneWordStatements - {simulatorStatement})
	| internalFunctions
	| replacements.keys()
)

### TRANSLATION PHASE ONE: handling imports

## Meta path finder and loader for Scenic files

scenicExtensions = ('sc', 'scenic')

class ScenicMetaFinder(importlib.abc.MetaPathFinder):
	def find_spec(self, name, paths, target=None):
		if paths is None:
			paths = sys.path
			modname = name
		else:
			modname = name.rpartition('.')[2]
		for path in paths:
			for extension in scenicExtensions:
				filename = modname + '.' + extension
				filepath = os.path.join(path, filename)
				if os.path.exists(filepath):
					filepath = os.path.abspath(filepath)
					spec = importlib.util.spec_from_file_location(name, filepath,
						loader=ScenicLoader(filepath, filename))
					return spec
		return None

class ScenicLoader(importlib.abc.InspectLoader):
	def __init__(self, filepath, filename):
		self.filepath = filepath
		self.filename = filename

	def create_module(self, spec):
		return None

	def exec_module(self, module):
		# Read source file and compile it
		with open(self.filepath, 'r') as stream:
			source = stream.read()
		with open(self.filepath, 'rb') as stream:
			code, pythonSource = compileStream(stream, module.__dict__, filename=self.filepath)
		# Mark as a Scenic module
		module._isScenicModule = True
		# Save code, source, and translated source for later inspection
		module._code = code
		module._source = source
		module._pythonSource = pythonSource

		# If we're in the process of compiling another Scenic module, inherit
		# objects, parameters, etc. from this one
		if veneer.isActive():
			veneer.currentScenario._inherit(module._scenario)

	def is_package(self, fullname):
		return False

	def get_code(self, fullname):
		module = importlib.import_module(fullname)
		assert module._isScenicModule, module
		return module._code

	def get_source(self, fullname):
		module = importlib.import_module(fullname)
		assert module._isScenicModule, module
		return module._pythonSource

# register the meta path finder
sys.meta_path.insert(0, ScenicMetaFinder())

## Miscellaneous utilities

def partitionByImports(tokens):
	"""Partition the tokens into blocks ending with import statements.

	We avoid splitting top-level try-except statements, to allow the pattern of trying
	to import an optional module and catching an ImportError. If someone tries to define
	objects inside such a statement, woe unto them.
	"""
	blocks = []
	currentBlock = []
	duringImport = False
	seenTry = False
	haveImported = False
	finishLine = False
	parenLevel = 0
	tokens = Peekable(tokens)
	for token in tokens:
		startNewBlock = False
		if token.exact_type == LPAR:
			parenLevel += 1
		elif token.exact_type == RPAR:
			parenLevel -= 1
		if finishLine:
			if token.type in (NEWLINE, NL) and parenLevel == 0:
				finishLine = False
				if duringImport:
					duringImport = False
					haveImported = True
		else:
			assert not duringImport
			finishLine = True
			if token.type in (DEDENT, NEWLINE, NL, COMMENT, ENCODING):
				finishLine = False
				if (seenTry and token.type == DEDENT and token.start[1] == 0
					and peek(tokens).string not in ('except', 'else', 'finally')):
					seenTry = False
					haveImported = True 	# just in case the try contained imports
			elif token.start[1] == 0:
				if token.string in ('import', 'from', modelStatement):
					duringImport = True
				else:
					if haveImported:
						# could use new constructors; needs to be in a new block
						startNewBlock = True
					if token.string == 'try':
						seenTry = True
		if startNewBlock:
			blocks.append(currentBlock)
			currentBlock = [token]
			haveImported = False
		else:
			currentBlock.append(token)
	blocks.append(currentBlock)		# add last block
	return blocks

def findConstructorsIn(namespace):
	"""Find all constructors (Scenic classes) defined in a namespace."""
	constructors = []
	for name, value in namespace.items():
		if inspect.isclass(value) and issubclass(value, _Constructible):
			if name in builtinConstructors:
				continue
			parents = []
			for base in value.__bases__:
				if issubclass(base, _Constructible):
					parents.append(base.__name__)
			constructors.append(Constructor(name, parents))
	return constructors

### TRANSLATION PHASE TWO: translation at the level of tokens

class Peekable:
	"""Utility class to allow iterator lookahead."""
	def __init__(self, gen):
		self.gen = iter(gen)
		self.current = next(self.gen, None)

	def __iter__(self):
		return self

	def __next__(self):
		cur = self.current
		if cur is None:
			raise StopIteration
		self.current = next(self.gen, None)
		return cur

	def peek(self):
		return self.current

def peek(thing):
	return thing.peek()

class TokenTranslator:
	"""Translates a Scenic token stream into valid Python syntax.

	This is a stateful process because constructor (Scenic class) definitions
	change the way subsequent code is parsed.
	"""
	def __init__(self, constructors=(), filename='<unknown>'):
		self.constructors = dict(builtinConstructors)
		for constructor in constructors:
			name = constructor.name
			assert name not in self.constructors
			self.constructors[name] = constructor
		self.filename = filename

	def parseError(self, tokenOrLine, message):
		raise TokenParseError(tokenOrLine, self.filename, message)

	def createConstructor(self, name, parents):
		parents = tuple(parents)
		assert parents
		self.constructors[name] = Constructor(name, parents)

	def specifiersForConstructor(self, const):
		# Currently all specifiers can be used with any constructor;
		# I'm leaving this here in case we later allow custom specifiers to be inherited
		return builtinSpecifiers
		#name, parents = self.constructors[const]

	def translate(self, tokens):
		"""Do the actual translation of the token stream."""
		tokens = Peekable(tokens)
		newTokens = []
		functionStack = []
		context, startLevel = None, None
		specifiersIndented = False
		parenLevel = 0
		row, col = 0, 0		# position of next token to write out
		orow, ocol = 0, 0	# end of last token in the original source
		startOfLine = True
		startOfStatement = True 	# position where a (simple) statement could begin
		constructors = self.constructors

		for token in tokens:
			ttype = token.exact_type
			tstring = token.string
			skip = False
			endToken = token 	# token to advance past in column count
			movedUpTo = False

			def injectToken(tok, spaceAfter=0):
				"""Add a token to the output stream, trying to preserve spacing."""
				nonlocal row, col, movedUpTo
				if not movedUpTo:
					moveUpTo(token)
					moveBeyond(token)
					movedUpTo = True
				ty, string = tok[:2]
				if len(tok) >= 3:
					moveBeyond(tok)
					srow, scol = tok[2]
					erow, ecol = tok[3]
					width = ecol - scol
					height = erow - srow
				else:
					width = len(string)
					height = 0
				ncol = ecol if height > 0 else col + width
				newToken = (ty, string, (row, col), (row+height, ncol), '')
				newTokens.append(newToken)
				if ty in (NEWLINE, NL):
					row += 1
					col = 0
				elif height > 0:
					row += height
					col = ncol
				else:
					col += width + spaceAfter
			def moveUpTo(tok):
				nonlocal row, col, orow, ocol
				nrow, ncol = tok[2]
				if nrow > orow:
					row = nrow
					col = ncol
				else:
					gap = ncol - ocol
					assert gap >= 0, (tok, row, col, ocol)
					col += gap
			def moveBeyond(tok):
				nonlocal orow, ocol
				nrow, ncol = tok[3]
				if nrow > orow or (nrow == orow and ncol > ocol):
					orow = nrow
					ocol = ncol
			def advance(skip=True):
				nextToken = next(tokens)
				if skip:
					moveBeyond(nextToken)
				else:
					injectToken(nextToken)
				return nextToken
			def callFunction(function, argument=None, implementation=None):
				nonlocal skip, matched, functionStack
				functionStack.append((function, parenLevel))
				implementation = function if implementation is None else implementation
				injectToken((NAME, implementation))
				injectToken((LPAR, '('))
				if argument is not None:
					injectToken(argument)
					injectToken((COMMA, ','))
				skip = True
				matched = True
			def popFunction():
				nonlocal context, startLevel
				functionStack.pop()
				injectToken((RPAR, ')'))
				context, startLevel = (None, 0) if len(functionStack) == 0 else functionStack[-1]
			def wrapStatementCall():
				injectToken((NAME, 'raise'), spaceAfter=1)
				injectToken((NAME, statementRaiseMarker), spaceAfter=1)
				injectToken((NAME, 'from'), spaceAfter=1)

			# Catch Python operators that can't be used in Scenic
			if ttype in illegalTokens:
				self.parseError(token, f'illegal operator "{tstring}"')

			# Determine which operators are allowed in current context
			allowedPrefixOps = prefixOperators
			allowedInfixOps = dict()
			allowedModifiers = dict()
			allowedTerminators = set()
			inConstructorContext = False
			context, startLevel = functionStack[-1] if functionStack else (None, None)
			if parenLevel == startLevel:
				if context in constructors:
					inConstructorContext = True
					allowedPrefixOps = self.specifiersForConstructor(context)
				else:
					for opTokens, op in infixTokens.items():
						if not op.contexts or context in op.contexts:
							allowedInfixOps[opTokens] = op.token
					for name, mod in modifierNames.items():
						if not mod.contexts or context in mod.contexts:
							allowedModifiers[name] = mod.name
					if context in modifierNames:
						allowedTerminators = modifierNames[context].terminators
					elif context in terminatorsForStatements:
						allowedTerminators = terminatorsForStatements[context]
			else:
				allowedInfixOps = generalInfixOps

			# Parse next token
			if ttype == LPAR or ttype == LSQB:		# keep track of nesting level
				parenLevel += 1
			elif ttype == RPAR or ttype == RSQB:	# ditto
				parenLevel -= 1
			elif ttype == STRING:
				# special case for global parameters with quoted names:
				# transform "name"=value into "name", value
				if (len(functionStack) > 0 and functionStack[-1][0] == paramStatement
					and peek(tokens).string == '='):
					next(tokens)	# consume '='
					injectToken(token)
					injectToken((COMMA, ','))
					skip = True
			elif ttype == NAME:		# the interesting case: almost all new syntax falls in here
				# try to match 2-word language constructs
				matched = False
				nextToken = peek(tokens)		# lookahead so we can give 2-word ops precedence
				if nextToken is not None:
					endToken = nextToken	# tentatively; will be overridden if no match
					nextString = nextToken.string
					twoWords = (tstring, nextString)
					if startOfLine and tstring == 'for':	# TODO improve hack?
						matched = True
						endToken = token
					elif startOfLine and tstring in constructorStatements:	# class definition
						if nextToken.type != NAME or nextString in keywords:
							self.parseError(nextToken, f'invalid class name "{nextString}"')
						nextToken = next(tokens)	# consume name
						bases, scenicParents = [], []
						if peek(tokens).exact_type == LPAR:		# superclass specification
							next(tokens)
							nextToken = next(tokens)
							while nextToken.exact_type != RPAR:
								base = nextToken.string
								if nextToken.exact_type != NAME:
									self.parseError(nextToken, f'invalid superclass "{base}"')
								bases.append(base)
								if base in self.constructors:
									scenicParents.append(base)
								if peek(tokens).exact_type == COMMA:
									next(tokens)
								nextToken = next(tokens)
							if not scenicParents and tstring != 'class':
								self.parseError(nextToken,
									f'Scenic class definition with no Scenic superclasses')
						if peek(tokens).exact_type != COLON:
							self.parseError(peek(tokens), 'malformed class definition')
						if not bases:
							bases = scenicParents = ('Object',)		# default superclass
						if scenicParents:
							self.createConstructor(nextString, scenicParents)
						injectToken((NAME, 'class'), spaceAfter=1)
						injectToken((NAME, nextString))
						injectToken((LPAR, '('))
						injectToken((NAME, bases[0]))
						for base in bases[1:]:
							injectToken((COMMA, ','), spaceAfter=1)
							injectToken((NAME, base))
						injectToken((RPAR, ')'))
						skip = True
						matched = True
						endToken = nextToken
					elif startOfLine and tstring == 'scenario':		# scenario definition
						if nextToken.type != NAME:
							self.parseError(nextToken, f'invalid scenario name "{nextString}"')
						className = scenarioMarker + nextString
						injectToken((NAME, 'async'), spaceAfter=1)
						injectToken((NAME, 'def'), spaceAfter=1)
						injectToken((NAME, className))
						advance()	# consume name
						skip = True
						matched = True
					elif startOfLine and tstring == behaviorStatement:		# behavior definition
						if nextToken.type != NAME:
							self.parseError(nextToken, f'invalid behavior name "{nextString}"')
						injectToken((NAME, 'async'), spaceAfter=1)
						injectToken((NAME, 'def'), spaceAfter=1)
						skip = True
						matched = True
						endToken = token
					elif startOfLine and tstring == monitorStatement:		# monitor definition
						if nextToken.type != NAME:
							self.parseError(nextToken, f'invalid monitor name "{nextString}"')
						injectToken((NAME, 'async'), spaceAfter=1)
						injectToken((NAME, 'def'), spaceAfter=1)
						injectToken((NAME, dynamics.functionForMonitor(nextString)))
						injectToken((LPAR, '('))
						injectToken((RPAR, ')'))
						advance()	# consume name
						if peek(tokens).exact_type != COLON:
							self.parseError(nextToken, 'malformed monitor definition')
						skip = True
						matched = True
					elif twoWords in allowedPrefixOps:	# 2-word prefix operator
						callFunction(allowedPrefixOps[twoWords])
						advance()	# consume second word
					elif not startOfStatement and twoWords in allowedInfixOps:	# 2-word infix operator
						injectToken(allowedInfixOps[twoWords])
						advance()	# consume second word
						skip = True
						matched = True
					elif startOfLine and twoWords == interruptWhenStatement:
						# special case for interrupt when
						injectToken((NAME, 'except'), spaceAfter=1)
						callFunction(interruptExceptMarker)
						advance()	# consume second word
						matched = True
					elif startOfStatement and twoWords in threeWordIncipits:	# 3-word statement
						advance()	# consume second word
						endToken = advance()	# consume third word
						thirdWord = endToken.tstring
						expected = threeWordIncipits[twoWords]
						if thirdWord != expected:
							self.parseError(endToken,
							                f'expected "{expected}", got "{thirdWord}"')
						wrapStatementCall()
						function = functionForStatement(twoWords + (thirdWord,))
						callFunction(function)
						matched = True
					elif startOfStatement and twoWords in twoWordStatements:	# 2-word statement
						wrapStatementCall()
						function = functionForStatement(twoWords)
						callFunction(function)
						advance()   # consume second word
						matched = True
					elif inConstructorContext and tstring == 'with':	# special case for 'with' specifier
						callFunction('With', argument=(STRING, f'"{nextString}"'))
						advance()	# consume property name
					elif startOfStatement and tstring == requireStatement and nextString == '[':
						# special case for require[p]
						next(tokens)	# consume '['
						nextToken = next(tokens)
						if nextToken.exact_type != NUMBER:
							self.parseError(nextToken,
								'soft requirement must have constant probability')
						prob = nextToken.string
						if not 0 <= float(prob) <= 1:
							self.parseError(nextToken, 'probability must be between 0 and 1')
						nextToken = next(tokens)
						if nextToken.exact_type != RSQB:
							self.parseError(nextToken, 'malformed soft requirement')
						wrapStatementCall()
						callFunction(softRequirement, argument=(NUMBER, prob))
						endToken = nextToken
					elif twoWords in twoWordReplacements:	# 2-word direct replacement
						for tok in twoWordReplacements[twoWords]:
							injectToken(tok, spaceAfter=1)
						advance()	# consume second word
						skip = True
					elif twoWords in illegalConstructs:
						construct = ' '.join(twoWords)
						self.parseError(token,
							f'Python construct "{construct}" not allowed in Scenic')
				if not matched:
					# 2-word constructs don't match; try 1-word
					endToken = token
					oneWord = (tstring,)
					if oneWord in allowedPrefixOps:		# 1-word prefix operator
						callFunction(allowedPrefixOps[oneWord])
					elif not startOfStatement and oneWord in allowedInfixOps:	# 1-word infix operator
						injectToken(allowedInfixOps[oneWord])
						skip = True
					elif inConstructorContext:		# couldn't match any 1- or 2-word specifier
						self.parseError(token, f'unknown specifier "{tstring}"')
					elif not startOfStatement and tstring in allowedModifiers:
						injectToken((COMMA, ','))
						callFunction(tstring, argument=(STRING, f'"{tstring}"'),
									 implementation='Modifier')
						skip = True
					elif not startOfStatement and tstring in allowedTerminators:
						injectToken((COMMA, ','))
						injectToken((STRING, f'"{tstring}"'))
						popFunction()
						skip = True
					elif startOfStatement and tstring in oneWordStatements:		# 1-word statement
						wrapStatementCall()
						callFunction(tstring)
					elif token.start[1] == 0 and tstring == modelStatement:		# model statement
						components = []
						while peek(tokens).exact_type not in (COMMENT, NEWLINE):
							nextToken = next(tokens)
							if nextToken.exact_type != NAME and nextToken.string != '.':
								self.parseError(nextToken, 'invalid module name')
							components.append(nextToken.string)
						if not components:
							self.parseError(token, 'model statement is missing module name')
						components.append("'")
						literal = "'" + ''.join(components)
						callFunction(modelStatement, argument=(NAME, namespaceReference))
						injectToken((STRING, literal))
						skip = True
					elif startOfLine and tstring in scenarioBlocks:		# named block of scenario
						if peek(tokens).exact_type != COLON:
							self.parseError(peek(tokens), f'malformed "{tstring}" block')
						injectToken((NAME, 'async'), spaceAfter=1)
						injectToken((NAME, 'def'), spaceAfter=1)
						injectToken((NAME, tstring))
						injectToken((LPAR, '('))
						injectToken((RPAR, ')'))
						skip = True
					elif (tstring in self.constructors
						  and peek(tokens).exact_type not in (RPAR, RSQB, RBRACE, COMMA)):
						# instance definition
						callFunction(tstring)
					elif tstring in replacements:	# direct replacement
						for tok in replacements[tstring]:
							injectToken(tok, spaceAfter=1)
						skip = True
					elif startOfLine and tstring == 'from':		# special case to allow 'from X import Y'
						pass
					elif tstring in keywords:		# some malformed usage
						self.parseError(token, f'unexpected keyword "{tstring}"')
					elif tstring in illegalConstructs:
						self.parseError(token,
							f'Python construct "{tstring}" not allowed in Scenic')
					else:
						pass	# nothing matched; pass through unchanged to Python

			# Detect the end of function argument lists
			if len(functionStack) > 0:
				context, startLevel = functionStack[-1]
				while parenLevel < startLevel:		# we've closed all parens for the current function
					popFunction()
				inConstructor = any(context in constructors for context, sl in functionStack)
				if inConstructor and parenLevel == startLevel and ttype == COMMA:		# starting a new specifier
					while functionStack and context not in constructors:
						popFunction()
					# allow the next specifier to be on the next line, if indented
					injectToken(token)		# emit comma immediately
					skip = True
					nextToken = peek(tokens)
					specOnNewLine = False
					while nextToken.exact_type in (NEWLINE, NL, COMMENT, ENDMARKER):
						specOnNewLine = True
						if nextToken.exact_type == COMMENT:
							advance(skip=False)		# preserve comment
							nextToken = peek(tokens)
						if nextToken.exact_type not in (NEWLINE, NL):
							self.parseError(nextToken, 'comma with no specifier following')
						advance(skip=False)		# preserve newline
						nextToken = peek(tokens)
					if specOnNewLine and not specifiersIndented:
						nextToken = next(tokens)		# consume indent
						if nextToken.exact_type != INDENT:
							self.parseError(nextToken,
								'expected indented specifier (extra comma on previous line?)')
						injectToken(nextToken)
						specifiersIndented = True
				elif ttype in (NEWLINE, ENDMARKER, COMMENT, SEMI):	# end of line or statement
					if parenLevel != 0:
						self.parseError(token, 'unmatched parens/brackets')
					interrupt = False
					if functionStack and functionStack[0][0] == interruptExceptMarker:
						lastToken = newTokens[-1]
						if lastToken[1] != ':':
							self.parseError(nextToken, 'expected colon for interrupt')
						newTokens.pop()		# remove colon for now
						interrupt = True
					while len(functionStack) > 0:
						functionStack.pop()
						injectToken((RPAR, ')'))
					if interrupt:
						injectToken((COLON, ':'))

			# Output token unchanged, unless handled above
			if not skip:
				injectToken(token)
			else:
				moveBeyond(endToken)
			startOfLine = (ttype in (ENCODING, NEWLINE, NL, INDENT, DEDENT))
			startOfStatement = startOfLine or (ttype == SEMI)

		rewrittenSource = tokenize.untokenize(newTokens)
		if not isinstance(rewrittenSource, str):	# TODO improve?
			rewrittenSource = str(rewrittenSource, encoding='utf-8')
		return rewrittenSource, self.constructors

### TRANSLATION PHASE THREE: parsing of Python resulting from token translation

def parseTranslatedSource(source, filename):
	try:
		tree = parse(source, filename=filename)
		return tree
	except SyntaxError as e:
		raise PythonParseError(e) from None

### TRANSLATION PHASE FOUR: modifying the parse tree

temporaryName = '_Scenic_temporary_name'
behaviorArgName = '_Scenic_current_behavior'
checkInvariantsName = '_Scenic_check_invariants'
interruptPrefix = '_Scenic_interrupt'

abortFlag = Attribute(Name('BlockConclusion', Load()), 'ABORT', Load())
breakFlag = Attribute(Name('BlockConclusion', Load()), 'BREAK', Load())
continueFlag = Attribute(Name('BlockConclusion', Load()), 'CONTINUE', Load())
returnFlag = Attribute(Name('BlockConclusion', Load()), 'RETURN', Load())
finishedFlag = Attribute(Name('BlockConclusion', Load()), 'FINISHED', Load())

noArgs = ast.arguments(
	posonlyargs=[],
	args=[], vararg=None,
	kwonlyargs=[], kw_defaults=[],
	kwarg=None, defaults=[])
selfArg = ast.arguments(
	posonlyargs=[],
	args=[ast.arg(arg='self', annotation=None)], vararg=None,
	kwonlyargs=[], kw_defaults=[],
	kwarg=None, defaults=[])
tempArg = ast.arguments(
	posonlyargs=[],
	args=[ast.arg(arg=temporaryName, annotation=None)], vararg=None,
	kwonlyargs=[], kw_defaults=[],
	kwarg=None, defaults=[Constant(None)])
initialBehaviorArgs = [
	ast.arg(arg=behaviorArgName, annotation=None),
	ast.arg(arg='self', annotation=None)
]
onlyBehaviorArgs = ast.arguments(
	posonlyargs=[],
	args=initialBehaviorArgs, vararg=None,
	kwonlyargs=[], kw_defaults=[],
	kwarg=None, defaults=[])

class AttributeFinder(NodeVisitor):
	"""Utility class for finding all referenced attributes of a given name."""
	@staticmethod
	def find(target, node):
		af = AttributeFinder(target)
		af.visit(node)
		return af.attributes

	def __init__(self, target):
		super().__init__()
		self.target = target
		self.attributes = set()

	def visit_Attribute(self, node):
		val = node.value
		if isinstance(val, Name) and val.id == self.target:
			self.attributes.add(node.attr)
		self.visit(val)

class LocalFinder(NodeVisitor):
	"""Utility class for finding all local variables of a code block."""
	@staticmethod
	def findIn(block, ignoreTemporaries=True):
		lf = LocalFinder()
		for statement in block:
			lf.visit(statement)
		if ignoreTemporaries:
			names = set(name for name in lf.names if not name.startswith(temporaryName))
		else:
			names = lf.names
		return names - lf.globals - lf.nonlocals

	def __init__(self):
		self.names = set()
		self.globals = set()
		self.nonlocals = set()

	def visit_Global(self, node):
		self.globals.update(node.names)

	def visit_Nonlocal(self, node):
		self.nonlocals.update(node.names)

	def visit_FunctionDef(self, node):
		self.names.add(node.name)
		self.visit(node.args)
		for decorator in node.decorator_list:
			self.visit(decorator)
		if node.returns is not None:
			self.visit(node.returns)
		# do not visit body; it's another block

	def visit_Lambda(self, node):
		self.visit(node.args)
		# do not visit body; it's another block

	def visit_ClassDef(self, node):
		self.names.add(node.name)
		for child in itertools.chain(node.bases, node.keywords, node.decorator_list):
			self.visit(child)
		# do not visit body; it's another block

	def visit_Import(self, node):
		for alias in node.names:
			bound = alias.asname
			if bound is None:
				bound = alias.name
			self.names.add(bound)

	def visit_ImportFrom(self, node):
		self.visit_Import(node)

	def visit_Name(self, node):
		if isinstance(node.ctx, (Store, Del)):
			self.names.add(node.id)

	def visit_ExceptHandler(self, node):
		if node.name is not None:
			self.names.add(node.name)
		self.generic_visit(node)

class ASTSurgeon(NodeTransformer):
	def __init__(self, constructors, filename):
		super().__init__()
		self.constructors = set(constructors.keys())
		self.filename = filename
		self.requirements = []
		self.inRequire = False
		self.inCompose = False
		self.inBehavior = False
		self.inGuard = False
		self.inTryInterrupt = False
		self.inInterruptBlock = False
		self.inLoop = False
		self.usedBreakOrContinue = False
		self.callDepth = 0

	def parseError(self, node, message):
		raise ASTParseError(node, message, self.filename)

	def unpack(self, arg, expected, node):
		"""Unpack arguments to ternary (and up) infix operators."""
		assert expected > 0
		if isinstance(arg, BinOp) and isinstance(arg.op, packageNode):
			if expected == 1:
				self.parseError(node, 'gave too many arguments to infix operator')
			else:
				return self.unpack(arg.left, expected - 1, node) + [self.visit(arg.right)]
		elif expected > 1:
			self.parseError(node, 'gave too few arguments to infix operator')
		else:
			return [self.visit(arg)]

	def visit(self, node):
		if isinstance(node, ast.AST):
			return super().visit(node)
		elif isinstance(node, list):
			newStatements = []
			for statement in node:
				newStatement = self.visit(statement)
				if isinstance(newStatement, ast.AST):
					newStatements.append(newStatement)
				else:
					newStatements.extend(newStatement)
			return newStatements
		else:
			raise RuntimeError(f'unknown object {node} encountered during AST surgery')

	def visit_Name(self, node):
		if node.id in builtinNames:
			if not isinstance(node.ctx, Load):
				self.parseError(node, f'unexpected keyword "{node.id}"')
		return node

	def visit_BinOp(self, node):
		"""Convert infix operators to calls to the corresponding Scenic internal functions."""
		left = node.left
		right = node.right
		op = node.op
		if isinstance(op, packageNode):		# unexpected argument package
			self.parseError(node, 'unexpected keyword "by"')
		elif type(op) in infixImplementations:	# an operator with non-Python semantics
			arity, impName = infixImplementations[type(op)]
			implementation = Name(impName, Load())
			copy_location(implementation, node)
			assert arity >= 2
			args = [self.visit(left)] + self.unpack(right, arity-1, node)
			newNode = Call(implementation, args, [])
		else:	# all other operators have the Python semantics
			newNode = BinOp(self.visit(left), op, self.visit(right))
		return copy_location(newNode, node)

	def visit_Raise(self, node):
		"""Handle Scenic statements encoded as raise statements.

		In particular:
		  * wrap require statements with lambdas;
		  * handle primitive action invocations inside behaviors;
		  * call the veneer implementations of other statements.
		"""
		if not isinstance(node.exc, Name) or node.exc.id != statementRaiseMarker:
			return self.generic_visit(node)		# an ordinary raise statement
		assert isinstance(node.cause, Call)
		assert isinstance(node.cause.func, Name)
		node = node.cause 	# move to inner call
		func = node.func
		assert isinstance(func, Name)
		if self.inCompose and func.id not in compositionalImps:
			statement = statementForImp[func.id]
			self.parseError(node, f'"{statement}" cannot be used in a {composeBlock} block')
		if self.inBehavior and func.id not in behavioralImps:
			statement = statementForImp[func.id]
			self.parseError(node, f'"{statement}" cannot be used in a behavior')
		if func.id in requirementStatements:		# require, terminate when, etc.
			if func.id == softRequirement:
				func.id = requireStatement
				numArgs = 2
				if len(node.args) != 2:
					self.parseError(node, f'"require" takes exactly 1 argument')
			else:
				numArgs = 1
			self.validateSimpleCall(node, numArgs)
			cond = node.args[-1]
			assert not self.inRequire
			self.inRequire = True
			req = self.visit(cond)
			self.inRequire = False
			reqID = Constant(len(self.requirements), None)	# save ID number
			self.requirements.append(req)		# save condition for later inspection when pruning
			closure = Lambda(noArgs, req)		# enclose requirement in a lambda
			lineNum = Constant(node.lineno, None)			# save line number for error messages
			copy_location(closure, req)
			copy_location(lineNum, req)
			newArgs = [reqID, closure, lineNum]
			if numArgs == 2:		# get probability for soft requirements
				prob = node.args[0]
				assert isinstance(prob, Constant) and isinstance(prob.value, (float, int))
				newArgs.append(prob)
			return copy_location(Expr(Call(func, newArgs, [])), node)
		elif func.id == simulatorStatement:
			self.validateSimpleCall(node, 1)
			sim = self.visit(node.args[0])
			closure = copy_location(Lambda(noArgs, sim), sim)
			return copy_location(Expr(Call(func, [closure], [])), node)
		elif func.id == invokeStatement:		# Sub-behavior or sub-scenario statement
			seenModifier = False
			invoked = []
			args = []
			for arg in node.args:
				if (isinstance(arg, Call) and isinstance(arg.func, Name)
					and arg.func.id == 'Modifier'):
					if seenModifier:
						self.parseError(arg,
							f'incompatible qualifiers for "{invokeStatement}" statement')
					seenModifier = True
					assert len(arg.args) >= 2
					assert isinstance(arg.args[0], Constant)
					if arg.args[0].value == 'until':
						arg.args[1] = Lambda(noArgs, arg.args[1])
					args.append(self.visit(arg))
				elif seenModifier:
					self.parseError(arg, f'malformed "{invokeStatement}" statement')
				else:
					invoked.append(self.visit(arg))
			maxInvoked = 1 if self.inBehavior and not self.inCompose else None
			self.validateSimpleCall(node, (1, maxInvoked), onlyInBehaviors=True, args=invoked)
			subHandler = Attribute(Name(behaviorArgName, Load()), '_invokeSubBehavior', Load())
			subArgs = [Name('self', Load()), Tuple(invoked, Load())] + args
			subRunner = Call(subHandler, subArgs, [])
			return self.generateInvocation(node, subRunner, invoker=YieldFrom)
		elif func.id == actionStatement:		# Action statement
			if self.inCompose:
				self.parseError(func, f'cannot use "{actionStatement}" in a {composeBlock} block')
			self.validateSimpleCall(node, (1, None), onlyInBehaviors=True)
			action = Tuple(self.visit(node.args), Load())
			return self.generateInvocation(node, action)
		elif func.id == waitStatement:		# Wait statement
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			return self.generateInvocation(node, Constant((), None))
		elif func.id == terminateStatement:		# Terminate statement
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			termination = Call(Name(createTerminationAction, Load()),
							   [Constant(node.lineno, None)], [])
			return self.generateInvocation(node, termination)
		elif func.id == abortStatement:		# abort statement for try-interrupt statements
			if not self.inTryInterrupt:
				self.parseError(node, '"abort" outside of try-interrupt statement')
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			return copy_location(Return(abortFlag), node)
		else:
			# statement implemented directly by a function; leave call intact
			newCall = self.visit_Call(node)
			return copy_location(Expr(newCall), node)

	def validateSimpleCall(self, node, numArgs, onlyInBehaviors=False, args=None):
		func = node.func
		name = func.id
		if onlyInBehaviors and not self.inBehavior:
			self.parseError(node, f'"{name}" can only be used in a behavior')
		args = node.args if args is None else args
		if isinstance(numArgs, tuple):
			assert len(numArgs) == 2
			low, high = numArgs
			if high is not None and len(args) > high:
				self.parseError(node, f'"{name}" takes at most {high} argument(s)')
			if len(args) < low:
				self.parseError(node, f'"{name}" takes at least {low} argument(s)')
		elif len(args) != numArgs:
			self.parseError(node, f'"{name}" takes exactly {numArgs} argument(s)')
		if len(node.keywords) != 0:
			self.parseError(node, f'"{name}" takes no keyword arguments')
		for arg in args:
			if isinstance(arg, Starred):
				self.parseError(node, f'argument unpacking cannot be used with "{name}"')

	def generateInvocation(self, node, actionlike, invoker=Yield):
		"""Generate an invocation of an action, behavior, or scenario."""
		invokeAction = Expr(invoker(actionlike))
		checkInvariants = Expr(Call(Name(checkInvariantsName, Load()), [], []))
		return [copy_location(invokeAction, node), copy_location(checkInvariants, node)]

	def visit_Try(self, node):
		"""Handle try-interrupt blocks."""
		interrupts = []
		exceptionHandlers = []
		for handler in node.handlers:
			ty = handler.type
			if (isinstance(ty, Call) and isinstance(ty.func, Name)
				and ty.func.id == interruptExceptMarker):
				assert handler.name is None
				if len(ty.args) != 1:
					self.parseError(handler, '"interrupt when" takes a single expression')
				interrupts.append((ty.args[0], handler.body))
			else:
				exceptionHandlers.append(handler)
		if not interrupts:		# an ordinary try-except block
			return self.generic_visit(node)

		# Add dead copy of all interrupts to ensure all local variables defined inside
		# the body and handlers are also defined as locals in the top-level function
		statements = []
		oldInTryInterrupt = self.inTryInterrupt
		if not self.inTryInterrupt:
			deadcopy = If(Constant(False), [node], [])
			statements.append(deadcopy)
			self.inTryInterrupt = True

		# Construct body
		oldInInterruptBlock, oldInLoop = self.inInterruptBlock, self.inLoop
		self.inInterruptBlock = True
		self.inLoop = False
		self.usedBreakOrContinue = False

		def makeInterruptBlock(name, body):
			newBody = self.visit(body)
			allLocals = LocalFinder.findIn(newBody)
			if allLocals:
				newBody.insert(0, Nonlocal(list(allLocals)))
			newBody.append(Return(finishedFlag))
			return FunctionDef(name, onlyBehaviorArgs, newBody, [], None)

		bodyName = f'{interruptPrefix}_body'
		statements.append(makeInterruptBlock(bodyName, node.body))

		# Construct interrupt handlers and condition checkers
		handlerNames, conditionNames = [], []
		for i, (condition, block) in enumerate(interrupts):
			handlerName = f'{interruptPrefix}_handler_{i}'
			handlerNames.append(handlerName)
			conditionName = f'{interruptPrefix}_condition_{i}'
			conditionNames.append(conditionName)

			statements.append(makeInterruptBlock(handlerName, block))
			self.inGuard = True
			checker = Lambda(noArgs, self.visit(condition))
			self.inGuard = False
			defChecker = Assign([Name(conditionName, Store())], checker)
			statements.append(defChecker)

		self.inInterruptBlock, self.inLoop = oldInInterruptBlock, oldInLoop
		self.inTryInterrupt = oldInTryInterrupt

		# Prepare tuples of interrupt conditions and handlers
		# (in order from high priority to low, so reversed relative to the syntax)
		conditions = Tuple([Name(n, Load()) for n in reversed(conditionNames)], Load())
		handlers = Tuple([Name(n, Load()) for n in reversed(handlerNames)], Load())

		# Construct code to execute the try-interrupt statement
		args = [
			Name(behaviorArgName, Load()),
			Name('self', Load()),
			Name(bodyName, Load()),
			conditions,
			handlers
		]
		callRuntime = Call(Name('runTryInterrupt', Load()), args, [])
		runTI = Assign([Name(temporaryName, Store())], YieldFrom(callRuntime))
		statements.append(runTI)
		result = Name(temporaryName, Load())
		if self.usedBreakOrContinue:
			test = Compare(result, [Is()], [breakFlag])
			statements.append(If(test, [Break()], []))
			test = Compare(result, [Is()], [continueFlag])
			statements.append(If(test, [Continue()], []))
		test = Compare(result, [Is()], [returnFlag])
		retCheck = If(test, [Return(Attribute(result, 'return_value', Load()))], [])
		statements.append(retCheck)

		# Construct overall try-except statement
		if exceptionHandlers or node.finalbody:
			newTry = Try(statements,
						 [self.visit(handler) for handler in exceptionHandlers],
						 self.visit(node.orelse),
						 self.visit(node.finalbody))
			return copy_location(newTry, node)
		else:
			return statements

	def visit_For(self, node):
		old = self.inLoop
		self.inLoop = True
		newNode = self.generic_visit(node)
		self.inLoop = old
		return newNode

	def visit_While(self, node):
		old = self.inLoop
		self.inLoop = True
		newNode = self.generic_visit(node)
		self.inLoop = old
		return newNode

	def visit_FunctionDef(self, node):
		oldInLoop, oldInInterruptBlock = self.inLoop, self.inInterruptBlock
		self.inLoop, self.inInterruptBlock = False, False
		newNode = self.generic_visit(node)
		self.inLoop, self.inInterruptBlock = oldInLoop, oldInInterruptBlock
		return newNode

	def visit_Break(self, node):
		if self.inInterruptBlock and not self.inLoop:
			self.usedBreakOrContinue = True
			newNode = Return(breakFlag)
			return copy_location(newNode, node)
		else:
			return self.generic_visit(node)

	def visit_Continue(self, node):
		if self.inInterruptBlock and not self.inLoop:
			self.usedBreakOrContinue = True
			newNode = Return(continueFlag)
			return copy_location(newNode, node)
		else:
			return self.generic_visit(node)

	def visit_Return(self, node):
		if self.inInterruptBlock:
			value = Constant(None) if node.value is None else node.value
			ret = Return(Call(returnFlag, [value], []))
			return copy_location(ret, node)
		else:
			return self.generic_visit(node)

	def visit_Call(self, node):
		"""Handle Scenic syntax and semantics of function calls.

		In particular:
		  * unpack argument packages for operators;
		  * check for iterable unpacking applied to distributions.
		"""
		func = node.func
		newArgs = []
		# Translate arguments, unpacking any argument packages
		self.callDepth += 1
		wrappedStar = False
		for arg in node.args:
			if isinstance(arg, BinOp) and isinstance(arg.op, packageNode):
				newArgs.extend(self.unpack(arg, 2, node))
			elif isinstance(arg, Starred) and not self.inBehavior:
				wrappedStar = True
				checkedVal = Call(Name('wrapStarredValue', Load()),
								  [self.visit(arg.value), Constant(arg.value.lineno, None)],
								  [])
				newArgs.append(Starred(checkedVal, Load()))
			else:
				newArgs.append(self.visit(arg))
		newKeywords = [self.visit(kwarg) for kwarg in node.keywords]
		newFunc = self.visit(func)
		self.callDepth -= 1
		if wrappedStar:
			newNode = Call(Name('callWithStarArgs', Load()), [newFunc] + newArgs, newKeywords)
		else:
			newNode = Call(newFunc, newArgs, newKeywords)
		newNode = copy_location(newNode, node)
		ast.fix_missing_locations(newNode)
		return newNode

	def visit_AsyncFunctionDef(self, node):
		"""Process Scenic constructs parsed as async function definitions.

		These include:
			* scenario definitions;
			* behavior (and monitor) definitions.
		"""
		if node.name.startswith(scenarioMarker):	# scenario definition
			return self.transformScenarioDefinition(node)
		else:
			return self.transformBehaviorDefinition(node)

	def transformScenarioDefinition(self, node):
		if self.inCompose:
			self.parseError(node, f'cannot define a scenario inside a {composeBlock} block')
		if self.inBehavior:
			self.parseError(node, 'cannot define a scenario inside a behavior')

		# Set up arguments for setup block (also inherited by compose block)
		args = node.args
		args.args = initialBehaviorArgs + args.args
		args = self.visit(args)

		# Extract named blocks from scenario body, if any
		simple = False	# simple scenario with no blocks
		setup, compose = None, None
		preconditions, invariants = [], []
		for statement in node.body:
			if isinstance(statement, AsyncFunctionDef):
				if statement.name == setupBlock:
					if setup:
						self.parseError(statement,
							f'scenario contains multiple "{setupBlock}" blocks')
					setup = statement
				elif statement.name == composeBlock:
					if compose:
						self.parseError(statement,
							f'scenario contains multiple "{composeBlock}" blocks')
					compose = statement
				elif statement.name in ('precondition', 'invariant'):
					if len(statement.body) != 1 or not isinstance(statement.body[0], Expr):
						self.parseError(statement.body[0], f'malformed precondition/invariant')
					self.inGuard = True
					test = self.visit(statement.body[0].value)
					self.inGuard = False
					assert isinstance(test, ast.AST)
					if statement.name == 'precondition':
						preconditions.append(test)
					else:
						invariants.append(test)
				else:
					simple = True
			else:
				# scenario contains actual code; assume it is simple
				simple = True
		if simple:	# simple scenario: entire body is implicitly the setup block
			if setup:
				self.parseError(setup, f'simple scenario cannot have a "{setupBlock}" block')
			if compose:
				self.parseError(compose, f'simple scenario cannot have a "{composeBlock}" block')
			if preconditions or invariants:
				self.parseError(node, f'simple scenario cannot have preconditions/invariants')

		# Construct compose block
		if compose or preconditions or invariants:
			self.inCompose = self.inBehavior = True
			preamble = self.makeBehaviorPreamble(preconditions, invariants)
			if compose:
				body = self.visit(compose.body)
				if setup:
					# make all locals in compose block actually use scope of setup block
					# (the compose definition will be placed inside the setup function)
					setupLocals = LocalFinder.findIn(setup.body)
					composeLocals = LocalFinder.findIn(body)
					commonLocals = setupLocals & composeLocals
					if commonLocals:
						body.insert(0, Nonlocal(list(commonLocals)))
			else:
				# generate no-op compose block to ensure invariants are checked
				wait = self.generateInvocation(node, Constant((), None))
				body = [While(Constant(True, None), wait, [])]
				compose = node 	# for copy_location below
			newBody = preamble + body
			newDef = FunctionDef('_compose', noArgs, newBody, [], None)
			self.inCompose = self.inBehavior = False
			compose = copy_location(newDef, compose)
		else:
			compose = Assign([Name('_compose', Store())], Constant(None, None))

		# Construct setup block
		if setup:
			oldBody = setup.body
			oldLoc = setup
		elif simple:
			oldBody = node.body
			oldLoc = node
		else:
			oldBody = []
			oldLoc = node
		getLocals = Call(Name('locals', Load()), [], [])
		returnStmt = Return(getLocals)
		newBody = self.visit(oldBody) + [compose, returnStmt]
		newDef = FunctionDef('_setup', args, newBody,
							 [Name(id='staticmethod', ctx=Load())], None)
		setup = copy_location(newDef, oldLoc)

		# Assemble scenario definition
		name = node.name[len(scenarioMarker):]
		body = [setup]
		newDef = ClassDef(name, [Name(scenarioClass, Load())], [], body, [])
		return copy_location(newDef, node)

	def transformBehaviorDefinition(self, node):
		if self.inCompose:
			self.parseError(node, f'cannot define a behavior inside a {composeBlock} block')
		if self.inBehavior:
			self.parseError(node, 'cannot define a behavior inside a behavior')

		# process body
		self.inBehavior = True
		oldInLoop = self.inLoop
		self.inLoop = False
		preconditions = []
		invariants = []
		newStatements = []
		# preserve docstring, if any
		if (isinstance(node.body[0], Expr) and isinstance(node.body[0].value, Constant)
		    and isinstance(node.body[0].value.value, str)):
			docstring = [node.body[0]]
			oldStatements = node.body[1:]
		else:
			docstring = []
			oldStatements = node.body
		# find precondition and invariant definitions
		for statement in oldStatements:
			if (isinstance(statement, AnnAssign)
				and isinstance(statement.target, Name)
				and statement.value is None
				and statement.simple):
				group = None
				if statement.target.id == 'precondition':
					group = preconditions
				elif statement.target.id == 'invariant':
					group = invariants
				else:
					self.parseError(statement, 'unknown type of behavior attribute')
				self.inGuard = True
				test = self.visit(statement.annotation)
				self.inGuard = False
				assert isinstance(test, ast.AST)
				group.append(test)
			else:
				newStatement = self.visit(statement)
				if isinstance(newStatement, ast.AST):
					newStatements.append(newStatement)
				else:
					newStatements.extend(newStatement)

		newBody = self.makeBehaviorPreamble(preconditions, invariants) + newStatements
		self.inBehavior = False
		self.inLoop = oldInLoop

		# add private current behavior argument and implicit 'self' argument
		newArgs = self.visit(node.args)
		newArgs.args = initialBehaviorArgs + newArgs.args

		# convert to class definition
		decorators = [self.visit(decorator) for decorator in node.decorator_list]
		genDefn = FunctionDef('makeGenerator', newArgs, newBody, decorators, node.returns)
		classBody = docstring + [genDefn]
		name = node.name
		if dynamics.isAMonitorName(name):
			superclass = monitorClass
			name = dynamics.monitorName(name)
		else:
			superclass = behaviorClass
		newDefn = ClassDef(name, [Name(superclass, Load())], [], classBody, [])

		return copy_location(newDefn, node)

	def makeBehaviorPreamble(self, preconditions, invariants):
		# generate precondition checks
		precondChecks = []
		for precondition in preconditions:
			call = Call(Name('PreconditionViolation', Load()),
			            [Name(behaviorArgName, Load()), Constant(precondition.lineno, None)],
			            [])
			throw = Raise(exc=call, cause=None)
			check = If(test=UnaryOp(Not(), precondition), body=[throw], orelse=[])
			precondChecks.append(copy_location(check, precondition))
		# generate invariant checker
		invChecks = []
		for invariant in invariants:
			call = Call(Name('InvariantViolation', Load()),
			            [Name(behaviorArgName, Load()), Constant(invariant.lineno, None)],
			            [])
			throw = Raise(exc=call, cause=None)
			check = If(test=UnaryOp(Not(), invariant), body=[throw], orelse=[])
			invChecks.append(copy_location(check, invariant))
		defineChecker = FunctionDef(checkInvariantsName, tempArg,
									invChecks + [Return(Name(temporaryName, Load()))], [], None)
		spot = Attribute(Name(behaviorArgName, Load()), 'checkInvariants', Store())
		saveChecker = Assign([spot], Name(checkInvariantsName, Load()))
		callChecker = Expr(Call(Name(checkInvariantsName, Load()), [], []))
		# assemble function body preamble
		preamble = precondChecks + [defineChecker, saveChecker, callChecker]
		return preamble

	def visit_Yield(self, node):
		if self.inCompose:
			self.parseError(node, f'"yield" is not allowed inside a {composeBlock} block')
		if self.inBehavior:
			self.parseError(node, '"yield" is not allowed inside a behavior')
		return self.generic_visit(node)

	def visit_YieldFrom(self, node):
		if self.inCompose:
			self.parseError(node, f'"yield from" is not allowed inside a {composeBlock} block')
		if self.inBehavior:
			self.parseError(node, '"yield from" is not allowed inside a behavior')
		return self.generic_visit(node)

	def visit_ClassDef(self, node):
		"""Handle Scenic constructs parsed as class definitions.

		In particular:
		  * transform Scenic class definitions into ordinary Python ones;
		  * leave ordinary Python class definitions alone.
		"""
		if node.name in self.constructors:		# Scenic class definition
			return self.transformScenicClass(node)
		else:		# ordinary Python class
			for base in node.bases:
				name = None
				if isinstance(base, Call):
					name = base.func.id
				elif isinstance(base, Name):
					name = base.id
				if name is not None and name in self.constructors:
					self.parseError(node,
									f'Python class {node.name} derives from Scenic class {name}')
			return self.generic_visit(node)

	def transformScenicClass(self, node):
		"""Process property defaults for Scenic classes."""
		newBody = []
		for child in node.body:
			child = self.visit(child)
			if isinstance(child, AnnAssign):	# default value for property
				origValue = child.annotation
				target = child.target
				# extract any attributes for this property
				metaAttrs = []
				if isinstance(target, Subscript):
					sl = target.slice
					if isinstance(sl, Index):	# needed for compatibility with Python 3.8 and earlier
						sl = sl.value
					if isinstance(sl, Name):
						metaAttrs.append(sl.id)
					elif isinstance(sl, Tuple):
						for elt in sl.elts:
							if not isinstance(elt, Name):
								self.parseError(elt,
									'malformed attributes for property default')
							metaAttrs.append(elt.id)
					else:
						self.parseError(sl, 'malformed attributes for property default')
					newTarget = Name(target.value.id, Store())
					copy_location(newTarget, target)
					target = newTarget
				# find dependencies of the default value
				properties = AttributeFinder.find('self', origValue)
				# create default value object
				args = [
					Set([Str(prop) for prop in properties]),
					Set([Str(attr) for attr in metaAttrs]),
					Lambda(selfArg, origValue)
				]
				value = Call(Name(createDefault, Load()), args, [])
				copy_location(value, origValue)
				newChild = AnnAssign(
					target=target, annotation=value,
					value=None, simple=True)
				child = copy_location(newChild, child)
			newBody.append(child)
		node.body = newBody
		return node

def translateParseTree(tree, constructors, filename):
	"""Modify the Python AST to produce the desired Scenic semantics."""
	surgeon = ASTSurgeon(constructors, filename)
	tree = fix_missing_locations(surgeon.visit(tree))
	return tree, surgeon.requirements

### TRANSLATION PHASE FIVE: AST compilation

def compileTranslatedTree(tree, filename):
	try:
		return compile(tree, filename, 'exec')
	except SyntaxError as e:
		raise PythonParseError(e) from None

### TRANSLATION PHASE SIX: Python execution

def executeCodeIn(code, namespace):
	"""Execute the final translated Python code in the given namespace."""
	exec(code, namespace)

### TRANSLATION PHASE SEVEN: scenario construction

def storeScenarioStateIn(namespace, requirementSyntax):
	"""Post-process an executed Scenic module, extracting state from the veneer."""

	# Save requirement syntax and other module-level information
	moduleScenario = veneer.currentScenario
	factory = veneer.simulatorFactory
	bns = gatherBehaviorNamespacesFrom(moduleScenario._behaviors)
	def handle(scenario):
		scenario._requirementSyntax = requirementSyntax
		if isinstance(scenario, type):
			scenario._simulatorFactory = staticmethod(factory)
		else:
			scenario._simulatorFactory = factory
		scenario._behaviorNamespaces = bns
	handle(moduleScenario)
	namespace['_scenarios'] = tuple(veneer.scenarios)
	for scenarioClass in veneer.scenarios:
		handle(scenarioClass)

	# Extract requirements, scan for relations used for pruning, and create closures
	# (only for top-level scenario; modular scenarios will be handled when instantiated)
	moduleScenario._compileRequirements()

	# Save global parameters
	for name, value in veneer._globalParameters.items():
		if needsLazyEvaluation(value):
			raise InvalidScenarioError(f'parameter {name} uses value {value}'
									   ' undefined outside of object definition')
	for scenario in veneer.scenarios:
		scenario._bindGlobals(veneer._globalParameters)
	moduleScenario._bindGlobals(veneer._globalParameters)

	namespace['_scenario'] = moduleScenario

def gatherBehaviorNamespacesFrom(behaviors):
	"""Gather any global namespaces which could be referred to by behaviors.

	We'll need to rebind any sampled values in them at runtime.
	"""
	behaviorNamespaces = {}
	def registerNamespace(modName, ns):
		if modName not in behaviorNamespaces:
			behaviorNamespaces[modName] = ns
		else:
			assert behaviorNamespaces[modName] is ns
			return
		for name, value in ns.items():
			if (isinstance(value, types.ModuleType)
				and getattr(value, '_isScenicModule', False)):
				registerNamespace(value.__name__, value.__dict__)
			else:
				# Convert values requiring sampling to Distributions
				dval = toDistribution(value)
				if dval is not value:
					ns[name] = dval
	for behavior in behaviors:
		modName = behavior.__module__
		globalNamespace = behavior.makeGenerator.__globals__
		registerNamespace(modName, globalNamespace)
	return behaviorNamespaces

def constructScenarioFrom(namespace, scenarioName=None):
	"""Build a Scenario object from an executed Scenic module."""
	modularScenarios = namespace['_scenarios']
	if scenarioName:
		ty = namespace.get(scenarioName, None)
		if not (isinstance(ty, type) and issubclass(ty, dynamics.DynamicScenario)):
			raise RuntimeError(f'no scenario "{scenarioName}" found')
		if ty._requiresArguments():
			raise RuntimeError(f'cannot instantiate scenario "{scenarioName}"'
			                   ' with no arguments') from None

		dynScenario = ty()
	elif len(modularScenarios) > 1:
		raise RuntimeError('multiple choices for scenario to run '
		                   '(specify using the --scenario option)')
	elif modularScenarios and not modularScenarios[0]._requiresArguments():
		dynScenario = modularScenarios[0]()
	else:
		dynScenario = namespace['_scenario']

	if not dynScenario._prepared:	# true for all except top-level scenarios
		# Execute setup block (if any) to create objects and requirements;
		# extract any requirements and scan for relations used for pruning
		dynScenario._prepare()
	scenario = dynScenario._toScenario(namespace)

	# Prune infeasible parts of the space
	if usePruning:
		pruning.prune(scenario, verbosity=verbosity)

	return scenario
