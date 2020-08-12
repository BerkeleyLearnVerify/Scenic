
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
import importlib
import importlib.abc
import importlib.util
import itertools
import pathlib
from collections import namedtuple, defaultdict
from contextlib import contextmanager

import tokenize
from tokenize import NAME, NL, NEWLINE, ENDMARKER, OP, NUMBER, COLON, COMMENT, ENCODING
from tokenize import LPAR, RPAR, LSQB, RSQB, COMMA, DOUBLESLASH, DOUBLESLASHEQUAL
from tokenize import AT, LEFTSHIFT, RIGHTSHIFT, VBAR, AMPER, TILDE, CIRCUMFLEX, STAR
from tokenize import LEFTSHIFTEQUAL, RIGHTSHIFTEQUAL, VBAREQUAL, AMPEREQUAL, CIRCUMFLEXEQUAL
from tokenize import INDENT, DEDENT, STRING

import ast
from ast import parse, dump, NodeVisitor, NodeTransformer, copy_location, fix_missing_locations
from ast import Load, Store, Name, Call, Tuple, BinOp, MatMult, BitAnd, BitOr, BitXor, LShift
from ast import RShift, Starred, Lambda, AnnAssign, Set, Str, Subscript, Index, IfExp
from ast import NamedExpr, Yield, YieldFrom, FunctionDef, Attribute, Constant, Assign, Expr
from ast import Return, Raise, If, UnaryOp, Not, ClassDef, Nonlocal, Global, Compare, Is, Try
from ast import Break, Continue

from scenic.core.distributions import Samplable, needsSampling, toDistribution
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.workspaces import Workspace
from scenic.core.simulators import Simulator
from scenic.core.scenarios import Scenario
from scenic.core.object_types import Constructible
import scenic.core.errors as errors
from scenic.core.errors import (TokenParseError, PythonParseError, ASTParseError,
                                InvalidScenarioError)
import scenic.core.pruning as pruning
import scenic.syntax.veneer as veneer
import scenic.syntax.relations as relations

### THE TOP LEVEL: compiling a Scenic program

def scenarioFromString(string, filename='<string>', cacheImports=False):
	"""Compile a string of Scenic code into a `Scenario`.

	The optional **filename** is used for error messages."""
	stream = io.BytesIO(string.encode())
	return scenarioFromStream(stream, filename=filename, cacheImports=cacheImports)

def scenarioFromFile(path, cacheImports=False):
	"""Compile a Scenic file into a `Scenario`.

	Args:
		path (str): path to a Scenic file
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
		return scenarioFromStream(stream, filename=fullpath, path=path,
		                          cacheImports=cacheImports)

def scenarioFromStream(stream, filename='<stream>', path=None, cacheImports=False):
	"""Compile a stream of Scenic code into a `Scenario`."""
	# Compile the code as if it were a top-level module
	oldModules = list(sys.modules.keys())
	try:
		with topLevelNamespace(path) as namespace:
			compileStream(stream, namespace, filename=filename)
	finally:
		if not cacheImports:
			toRemove = []
			for name, module in sys.modules.items():
				if name not in oldModules and getattr(module, '_isScenicModule', False):
					toRemove.append(name)
			for name in toRemove:
				del sys.modules[name]
	# Construct a Scenario from the resulting namespace
	return constructScenarioFrom(namespace)

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

def compileStream(stream, namespace, filename='<stream>'):
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
	veneer.activate()
	newSourceBlocks = []
	try:
		# Execute preamble
		exec(compile(preamble, '<veneer>', 'exec'), namespace)
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
		storeScenarioStateIn(namespace, requirements, filename)
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

rangeConstructor = 'Range'
createDefault = 'PropertyDefault'
behaviorClass = 'Behavior'
monitorClass = 'Monitor'
behaviorChecker = 'isABehavior'
createTerminationAction = 'makeTerminationAction'
internalFunctions = {
	rangeConstructor, createDefault, behaviorClass, behaviorChecker, createTerminationAction,
}

# sanity check: these functions actually exist
for imp in internalFunctions:
	assert imp in api, imp

## Built-in functions

builtinFunctions = { 'resample', 'verbosePrint', 'simulation' }

# sanity check: implementations of built-in functions actually exist
for imp in builtinFunctions:
	assert imp in api, imp

## Simple statements

paramStatement = 'param'
mutateStatement = 'mutate'

requireStatement = 'require'
softRequirement = 'require_soft'	# not actually a statement, but a marker for the parser
requireAlwaysStatement = ('require', 'always')
terminateWhenStatement = ('terminate', 'when')

actionStatement = 'take'			# statement invoking a primitive action
waitStatement = 'wait'				# statement invoking a no-op action
terminateStatement = 'terminate'	# statement ending the simulation
abortStatement = 'abort'			# statement ending a try-interrupt statement

oneWordStatements = {	# TODO clean up
	paramStatement, mutateStatement, requireStatement,
	actionStatement, waitStatement, terminateStatement,
	abortStatement
}
twoWordStatements = { requireAlwaysStatement, terminateWhenStatement }

# statements implemented by functions
functionStatements = { requireStatement, paramStatement, mutateStatement }
twoWordFunctionStatements = { requireAlwaysStatement, terminateWhenStatement }
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

# statements allowed inside behaviors
behavioralStatements = {
	requireStatement, actionStatement, waitStatement,
	terminateStatement, abortStatement
}
behavioralImps = { functionForStatement(s) for s in behavioralStatements }

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
	('visible',): 'Visible'
}
assert all(1 <= len(op) <= 2 for op in prefixOperators)
prefixIncipits = { op[0] for op in prefixOperators }
assert not any(op in oneWordStatements for op in prefixIncipits)
assert not any(op in twoWordStatements for op in prefixOperators)

# sanity check: implementations of prefix operators actually exist
for imp in prefixOperators.values():
	assert imp in api, imp

## Infix operators

# pseudo-operator for encoding argument packages for (3+)-ary ops
packageToken = (RIGHTSHIFT, '>>')
packageNode = RShift

InfixOp = namedtuple('InfixOp', ('syntax', 'implementation', 'arity', 'token', 'node'))
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
	InfixOp('for', None, 2, (COMMA, ','), None),
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
		infixTokens[tokens] = op.token
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

allIncipits = prefixIncipits | infixIncipits

## Direct syntax replacements

replacements = {	# TODO police the usage of these? could yield bizarre error messages
	'of': tuple(),
	'deg': ((STAR, '*'), (NUMBER, '0.01745329252')),
	'ego': ((NAME, 'ego'), (LPAR, '('), (RPAR, ')')),
	behaviorStatement: ((NAME, 'async'), (NAME, 'def')),
}

## Illegal and reserved syntax

illegalTokens = {
	LEFTSHIFT, RIGHTSHIFT, VBAR, AMPER, TILDE, CIRCUMFLEX,
	LEFTSHIFTEQUAL, RIGHTSHIFTEQUAL, VBAREQUAL, AMPEREQUAL, CIRCUMFLEXEQUAL,
	DOUBLESLASH, DOUBLESLASHEQUAL
}

# sanity check: stand-in tokens for infix operators must be illegal
for token in infixTokens.values():
	ttype = token[0]
	assert (ttype is COMMA or ttype in illegalTokens), token

illegalConstructs = {
	('async', 'def'),		# used to parse behaviors, so disallowed otherwise
}

keywords = (set(constructorStatements) | {monitorStatement}
	| internalFunctions | oneWordStatements
	| replacements.keys())

### TRANSLATION PHASE ONE: handling imports

## Meta path finder and loader for Scenic files

scenicExtensions = ('sc', 'scenic')

class ScenicMetaFinder(importlib.abc.MetaPathFinder):
	def find_spec(self, name, paths, target):
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
			veneer.allObjects.extend(module._objects)
			veneer.globalParameters.update(module._params)
			veneer.externalParameters.extend(module._externalParams)
			veneer.inheritedReqs.extend(module._requirements)
			veneer.behaviors.extend(module._behaviors)
			veneer.monitors.extend(module._monitors)

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
	"""Partition the tokens into blocks ending with import statements."""
	blocks = []
	currentBlock = []
	duringImport = False
	haveImported = False
	finishLine = False
	parenLevel = 0
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
			if token.type == NAME and token.string == 'import' or token.string == 'from':
				duringImport = True
			elif token.type in (NEWLINE, NL, COMMENT, ENCODING):
				finishLine = False
			elif haveImported:
				# could use new constructors; needs to be in a new block
				startNewBlock = True
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
		if inspect.isclass(value) and issubclass(value, Constructible):
			if name in builtinConstructors:
				continue
			parents = []
			for base in value.__bases__:
				if issubclass(base, Constructible):
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
		return TokenParseError(tokenOrLine, self.filename, message)

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
		inConstructor = False	# inside a constructor or one of its specifiers
		specifiersIndented = False
		parenLevel = 0
		row, col = 0, 0		# position of next token to write out
		orow, ocol = 0, 0	# end of last token in the original source
		startOfLine = True		# TODO improve hack?
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
			def callFunction(function, argument=None):
				nonlocal skip, matched, functionStack
				functionStack.append((function, parenLevel))
				injectToken((NAME, function))
				injectToken((LPAR, '('))
				if argument is not None:
					injectToken((NAME, argument))
					injectToken((COMMA, ','))
				skip = True
				matched = True
			def wrapStatementCall():
				injectToken((NAME, 'raise'), spaceAfter=1)
				injectToken((NAME, statementRaiseMarker), spaceAfter=1)
				injectToken((NAME, 'from'), spaceAfter=1)

			# Catch Python operators that can't be used in Scenic
			if ttype in illegalTokens:
				raise self.parseError(token, f'illegal operator "{tstring}"')

			# Determine which operators are allowed in current context
			context, startLevel = functionStack[-1] if functionStack else (None, None)
			inConstructorContext = (context in constructors and parenLevel == startLevel)
			if inConstructorContext:
				inConstructor = True
				allowedPrefixOps = self.specifiersForConstructor(context)
				allowedInfixOps = dict()
			else:
				allowedPrefixOps = prefixOperators
				allowedInfixOps = infixTokens

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
							raise self.parseError(nextToken,
							    f'invalid class name "{nextString}"')
						nextToken = next(tokens)	# consume name
						bases, scenicParents = [], []
						if peek(tokens).exact_type == LPAR:		# superclass specification
							next(tokens)
							while (nextToken := next(tokens)).exact_type != RPAR:
								base = nextToken.string
								if nextToken.exact_type != NAME:
									raise self.parseError(nextToken,
									    f'invalid superclass "{base}"')
								bases.append(base)
								if base in self.constructors:
									scenicParents.append(base)
								if peek(tokens).exact_type == COMMA:
									next(tokens)
							if not scenicParents and tstring != 'class':
								raise self.parseError(nextToken,
								    f'Scenic class definition with no Scenic superclasses')
						if peek(tokens).exact_type != COLON:
							raise self.parseError(peek(tokens), 'malformed class definition')
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
					elif startOfLine and tstring == monitorStatement:		# monitor definition
						if nextToken.type != NAME:
							raise self.parseError(nextToken,
							    f'invalid monitor name "{nextString}"')
						injectToken((NAME, 'async'), spaceAfter=1)
						injectToken((NAME, 'def'), spaceAfter=1)
						injectToken((NAME, veneer.functionForMonitor(nextString)))
						injectToken((LPAR, '('))
						injectToken((RPAR, ')'))
						advance()	# consume name
						if peek(tokens).exact_type != COLON:
							raise self.parseError(nextToken, 'malformed monitor definition')
						skip = True
						matched = True
					elif twoWords in allowedPrefixOps:	# 2-word prefix operator
						callFunction(allowedPrefixOps[twoWords])
						advance()	# consume second word
					elif not startOfLine and twoWords in allowedInfixOps:	# 2-word infix operator
						injectToken(allowedInfixOps[twoWords])
						advance()	# consume second word
						skip = True
						matched = True
					elif twoWords == interruptWhenStatement:	# special case for interrupt when
						injectToken((NAME, 'except'), spaceAfter=1)
						callFunction(interruptExceptMarker)
						advance()	# consume second word
						matched = True
					elif twoWords in twoWordStatements:	# 2-word statement
						wrapStatementCall()
						function = functionForStatement(twoWords)
						callFunction(function)
						advance()   # consume second word
						matched = True
					elif inConstructorContext and tstring == 'with':	# special case for 'with' specifier
						callFunction('With', argument=f'"{nextString}"')
						advance()	# consume property name
					elif tstring == requireStatement and nextString == '[':		# special case for require[p]
						next(tokens)	# consume '['
						nextToken = next(tokens)
						if nextToken.exact_type != NUMBER:
							raise self.parseError(nextToken,
							    'soft requirement must have constant probability')
						prob = nextToken.string
						if not 0 <= float(prob) <= 1:
							raise self.parseError(nextToken,
							                      'probability must be between 0 and 1')
						nextToken = next(tokens)
						if nextToken.exact_type != RSQB:
							raise self.parseError(nextToken, 'malformed soft requirement')
						wrapStatementCall()
						callFunction(softRequirement, argument=prob)
						endToken = nextToken
					elif twoWords in illegalConstructs:
						construct = ' '.join(twoWords)
						raise self.parseError(token,
						                      f'Python construct "{construct}" not allowed in Scenic')
				if not matched:
					# 2-word constructs don't match; try 1-word
					endToken = token
					oneWord = (tstring,)
					if oneWord in allowedPrefixOps:		# 1-word prefix operator
						callFunction(allowedPrefixOps[oneWord])
					elif not startOfLine and oneWord in allowedInfixOps:	# 1-word infix operator
						injectToken(allowedInfixOps[oneWord])
						skip = True
					elif inConstructorContext:		# couldn't match any 1- or 2-word specifier
						raise self.parseError(token, f'unknown specifier "{tstring}"')
					elif tstring in oneWordStatements:		# 1-word statement
						wrapStatementCall()
						callFunction(tstring)
					elif tstring in self.constructors:      # instance definition
						callFunction(tstring)
					elif tstring in replacements:	# direct replacement
						for tok in replacements[tstring]:
							injectToken(tok, spaceAfter=1)
						skip = True
					elif startOfLine and tstring == 'from':		# special case to allow 'from X import Y'
						pass
					elif tstring in keywords:		# some malformed usage
						raise self.parseError(token, f'unexpected keyword "{tstring}"')
					elif tstring in illegalConstructs:
						raise self.parseError(token, f'Python construct "{tstring}" not allowed in Scenic')
					else:
						pass	# nothing matched; pass through unchanged to Python

			# Detect the end of function argument lists
			if len(functionStack) > 0:
				context, startLevel = functionStack[-1]
				while parenLevel < startLevel:		# we've closed all parens for the current function
					functionStack.pop()
					injectToken((RPAR, ')'))
					context, startLevel = (None, 0) if len(functionStack) == 0 else functionStack[-1]
				if inConstructor and parenLevel == startLevel and ttype == COMMA:		# starting a new specifier
					while functionStack and context not in constructors:
						functionStack.pop()
						injectToken((RPAR, ')'))
						context, startLevel = (None, 0) if len(functionStack) == 0 else functionStack[-1]
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
							raise self.parseError(nextToken, 'comma with no specifier following')
						advance(skip=False)		# preserve newline
						nextToken = peek(tokens)
					if specOnNewLine and not specifiersIndented:
						nextToken = next(tokens)		# consume indent
						if nextToken.exact_type != INDENT:
							raise self.parseError(nextToken,
							                      'expected indented specifier (extra comma on previous line?)')
						injectToken(nextToken)
						specifiersIndented = True
				elif ttype == NEWLINE or ttype == ENDMARKER or ttype == COMMENT:	# end of line
					inConstructor = False
					if parenLevel != 0:
						raise self.parseError(token, 'unmatched parens/brackets')
					interrupt = False
					if functionStack and functionStack[0][0] == interruptExceptMarker:
						lastToken = newTokens[-1]
						if lastToken[1] != ':':
							raise self.parseError(nextToken, 'expected colon for interrupt')
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
		self.globals |= node.names

	def visit_Nonlocal(self, node):
		self.nonlocals |= node.names

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
		if isinstance(node.ctx, Store):
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
				raise self.parseError(node, 'gave too many arguments to infix operator')
			else:
				return self.unpack(arg.left, expected - 1, node) + [self.visit(arg.right)]
		elif expected > 1:
			raise self.parseError(node, 'gave too few arguments to infix operator')
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

	def visit_BinOp(self, node):
		"""Convert infix operators to calls to the corresponding Scenic operator implementations."""
		left = node.left
		right = node.right
		op = node.op
		if isinstance(op, packageNode):		# unexpected argument package
			raise self.parseError(node, 'unexpected keyword "by"')
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

	def visit_Tuple(self, node):
		"""Convert pairs into uniform distributions."""
		if isinstance(node.ctx, Store):
			return self.generic_visit(node)
		if len(node.elts) != 2:
			raise self.parseError(node, 'interval must have exactly two endpoints')
		newElts = [self.visit(elt) for elt in node.elts]
		return copy_location(Call(Name(rangeConstructor, Load()), newElts, []), node)

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
		if self.inBehavior and func.id not in behavioralImps:
			statement = statementForImp[func.id]
			raise self.parseError(node, f'"{statement}" cannot be used in a behavior')
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
		elif func.id == actionStatement:		# Action statement
			self.validateSimpleCall(node, (1, None), onlyInBehaviors=True)
			action = Tuple(self.visit(node.args), Load())
			return self.generateActionInvocation(node, action)
		elif func.id == waitStatement:		# Wait statement
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			return self.generateActionInvocation(node, Constant((), None))
		elif func.id == terminateStatement:		# Terminate statement
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			termination = Call(Name(createTerminationAction, Load()),
			                   [Constant(node.lineno, None)], [])
			return self.generateActionInvocation(node, termination)
		elif func.id == abortStatement:		# abort statement for try-interrupt statements
			if not self.inTryInterrupt:
				raise self.parseError(node, '"abort" outside of try-interrupt statement')
			self.validateSimpleCall(node, 0, onlyInBehaviors=True)
			return copy_location(Return(abortFlag), node)
		else:
			# statement implemented directly by a function; leave call intact
			newCall = self.visit_Call(node, subBehaviorCheck=False)
			return copy_location(Expr(newCall), node)

	def generateActionInvocation(self, node, action):
		invokeAction = Expr(Yield(action))
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
					raise self.parseError(handler, '"interrupt when" takes a single expression')
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

	def visit_Call(self, node, subBehaviorCheck=True):
		"""Handle Scenic syntax and semantics of function calls.

		In particular:
		  * unpack argument packages for operators;
		  * check for iterable unpacking applied to distributions;
		  * check for sub-behavior invocation inside behaviors.
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
		if subBehaviorCheck and self.inBehavior and not self.inGuard and not self.inRequire:
			# Add check for sub-behavior invocation
			newNode = self.wrapBehaviorCall(newFunc, newArgs, newKeywords)
		elif wrappedStar:
			newNode = Call(Name('callWithStarArgs', Load()), [newFunc] + newArgs, newKeywords)
		else:
			newNode = Call(newFunc, newArgs, newKeywords)
		newNode = copy_location(newNode, node)
		ast.fix_missing_locations(newNode)
		return newNode

	def validateSimpleCall(self, node, numArgs, onlyInBehaviors=False):
		func = node.func
		name = func.id
		if onlyInBehaviors and not self.inBehavior:
			raise self.parseError(node, f'"{name}" can only be used in a behavior')
		if isinstance(numArgs, tuple):
			assert len(numArgs) == 2
			low, high = numArgs
			if high is not None and len(node.args) > high:
				raise self.parseError(node, f'"{name}" takes at most {high} argument(s)')
			if len(node.args) < low:
				raise self.parseError(node, f'"{name}" takes at least {low} argument(s)')
		elif len(node.args) != numArgs:
			raise self.parseError(node, f'"{name}" takes exactly {numArgs} argument(s)')
		if len(node.keywords) != 0:
			raise self.parseError(node, f'"{name}" takes no keyword arguments')
		for arg in node.args:
			if isinstance(arg, Starred):
				raise self.parseError(node, f'argument unpacking cannot be used with "{name}"')

	def wrapBehaviorCall(self, func, args, keywords):
		"""Wraps a function call to handle calling a behavior.

		Specifically, transforms a call of the form:
			function(args)
		into:
			(CHECK(yield from BEHAVIOR.callSubBehavior(TEMP, self, TEMP2))
			if isABehavior(TEMP := function, TEMP2 := args)
			else TEMP(TEMP2))
		where TEMP and TEMP2 are temporary names (we actually save each argument),
		BEHAVIOR is a hidden argument storing the current Behavior object, and
		CHECK is a function which checks the invariants and returns its argument.

		The := expressions are necessary to prevent unexpected behavior if the expression
		'function' has side effects, and exponential blowup for nested function calls. We
		use separate temporary names for each nesting depth, in case 'args' contains
		further wrapped function calls.
		"""
		newArgs, savedArgs = [], []
		for i, arg in enumerate(args):
			name = f'{temporaryName}_arg_{self.callDepth}_{i}'
			if isinstance(arg, Starred):
				newArg = Starred(NamedExpr(Name(name, Store()), arg.value), arg.ctx)
				savedArg = Starred(Name(name, Load()), Load())
			else:
				newArg = NamedExpr(Name(name, Store()), arg)
				savedArg = Name(name, Load())
			newArgs.append(newArg)
			savedArgs.append(savedArg)
		newKwargs, savedKwargs = [], []
		for i, arg in enumerate(keywords):
			name = f'{temporaryName}_kwarg_{self.callDepth}_{i}'
			newArg = ast.keyword(arg.arg, NamedExpr(Name(name, Store()), arg.value))
			newKwargs.append(newArg)
			savedKwargs.append(ast.keyword(arg.arg, Name(name, Load())))

		name = f'{temporaryName}_func_{self.callDepth}'
		savedFunc = NamedExpr(Name(name, Store()), func)
		condition = Call(Name(behaviorChecker, Load()), [savedFunc] + newArgs, newKwargs)
		subHandler = Attribute(Name(behaviorArgName, Load()), 'callSubBehavior', Load())

		subArgs = [Name(name, Load()), Name('self', Load())] + savedArgs
		subCall = Call(subHandler, subArgs, savedKwargs)
		yieldFrom = YieldFrom(subCall)
		checkedSubCall = Call(Name(checkInvariantsName, Load()), [yieldFrom], [])
		ordinaryCall = Call(Name(name, Load()), savedArgs, savedKwargs)
		return IfExp(condition, checkedSubCall, ordinaryCall)

	def visit_AsyncFunctionDef(self, node):
		"""Process behavior definitions."""
		if self.inBehavior:
			raise self.parseError(node, 'cannot define a behavior inside a behavior')

		# process body
		self.inBehavior = True
		oldInLoop = self.inLoop
		self.inLoop = False
		preconditions = []
		invariants = []
		newStatements = []
		# find precondition and invariant definitions
		for statement in node.body:
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
					raise self.parseError(statement, 'unknown type of behavior attribute')
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
		# generate precondition checks
		precondChecks = []
		for precondition in preconditions:
			throw = Raise(exc=Name('PreconditionFailure', Load()), cause=None)
			check = If(test=UnaryOp(Not(), precondition), body=[throw], orelse=[])
			precondChecks.append(copy_location(check, precondition))
		# generate invariant checker
		invChecks = []
		for invariant in invariants:
			throw = Raise(exc=Name('InvariantFailure', Load()), cause=None)
			check = If(test=UnaryOp(Not(), invariant), body=[throw], orelse=[])
			invChecks.append(copy_location(check, invariant))
		defineChecker = FunctionDef(checkInvariantsName, tempArg,
		                            invChecks + [Return(Name(temporaryName, Load()))],
		                            [], None)
		spot = Attribute(Name(behaviorArgName, Load()), 'checkInvariants', Store())
		saveChecker = Assign([spot], Name(checkInvariantsName, Load()))
		callChecker = Expr(Call(Name(checkInvariantsName, Load()), [], []))
		# assemble new function body
		newBody = precondChecks + [defineChecker, saveChecker, callChecker] + newStatements
		self.inBehavior = False
		self.inLoop = oldInLoop

		# add private current behavior argument and implicit 'self' argument
		newArgs = self.visit(node.args)
		newArgs.args = initialBehaviorArgs + newArgs.args

		# convert to class definition
		decorators = [self.visit(decorator) for decorator in node.decorator_list]
		genDefn = FunctionDef('makeGenerator', newArgs, newBody, decorators, node.returns)
		name = node.name
		if veneer.isAMonitorName(name):
			superclass = monitorClass
			name = veneer.monitorName(name)
		else:
			superclass = behaviorClass
		newDefn = ClassDef(name, [Name(superclass, Load())], [], [genDefn], [])

		return copy_location(newDefn, node)

	def visit_Yield(self, node):
		if self.inBehavior:
			raise self.parseError(node, '"yield" is not allowed inside a behavior')
		return self.generic_visit(node)

	def visit_YieldFrom(self, node):
		if self.inBehavior:
			raise self.parseError(node, '"yield from" is not allowed inside a behavior')
		return self.generic_visit(node)

	def visit_ClassDef(self, node):
		"""Process property defaults for Scenic classes."""
		if node.name in self.constructors:		# Scenic class definition
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
						if not isinstance(sl, Index):
							self.parseError(sl, 'malformed attributes for property default')
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
		else:		# ordinary Python class
			# it's impossible at the moment to define a Python class in a Scenic file,
			# but we'll leave this check here for future-proofing
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

def storeScenarioStateIn(namespace, requirementSyntax, filename):
	"""Post-process an executed Scenic module, extracting state from the veneer."""
	# Extract created Objects
	namespace['_objects'] = tuple(veneer.allObjects)
	namespace['_egoObject'] = veneer.egoObject

	# Extract global parameters
	namespace['_params'] = veneer.globalParameters
	for name, value in veneer.globalParameters.items():
		if needsLazyEvaluation(value):
			raise InvalidScenarioError(f'parameter {name} uses value {value}'
			                           ' undefined outside of object definition')

	# Extract external parameters
	namespace['_externalParams'] = tuple(veneer.externalParameters)

	# Extract requirements, scan for relations used for pruning, and create closures
	requirements = veneer.pendingRequirements
	finalReqs = veneer.inheritedReqs
	requirementDeps = set()	# things needing to be sampled to evaluate the requirements
	namespace['_requirements'] = finalReqs
	namespace['_requirementDeps'] = requirementDeps
	def makeClosure(req, bindings, ego, line):
		"""Create a closure testing the requirement in the correct runtime state."""
		def evaluator():
			result = req()
			assert not needsSampling(result)
			if needsLazyEvaluation(result):
				raise InvalidScenarioError(f'requirement on line {line} uses value'
				                           ' undefined outside of object definition')
			return result
		def closure(values):
			# rebind any names referring to sampled objects
			for name, value in bindings.items():
				if value in values:
					namespace[name] = values[value]
			# evaluate requirement condition, reporting errors on the correct line
			try:
				veneer.evaluatingRequirement = True
				# rebind ego object, which can be referred to implicitly
				oldEgo = veneer.egoObject
				assert veneer.currentSimulation or veneer.egoObject is None
				if ego is not None:
					veneer.egoObject = values[ego]
				result = evaluator()
			finally:
				veneer.evaluatingRequirement = False
				veneer.egoObject = oldEgo
			return result
		return closure
	for reqID, requirement in requirements.items():
		bindings, ego = requirement.bindings, requirement.egoObject
		line = requirement.line
		# Check whether requirement implies any relations used for pruning
		if requirement.ty.constrainsSampling:
			reqNode = requirementSyntax[reqID]
			relations.inferRelationsFrom(reqNode, bindings, ego, line)
		# Gather dependencies of the requirement
		for value in bindings.values():
			if needsSampling(value):
				requirementDeps.add(value)
			if needsLazyEvaluation(value):
				raise InvalidScenarioError(f'requirement on line {line} uses value {value}'
				                           ' undefined outside of object definition')
		if ego is not None:
			assert isinstance(ego, Samplable)
			requirementDeps.add(ego)
		# Construct closure
		closure = makeClosure(requirement.condition, bindings, ego, line)
		finalReqs.append(veneer.CompiledRequirement(requirement, closure))

	# Extract monitors
	namespace['_behaviors'] = veneer.behaviors
	namespace['_monitors'] = veneer.monitors

	# Gather all global namespaces which could be referred to by behaviors;
	# we'll need to rebind any sampled values in them at runtime
	behaviorNamespaces = {}
	def registerNamespace(modName, ns):
		if modName not in behaviorNamespaces:
			behaviorNamespaces[modName] = ns
		else:
			assert behaviorNamespaces[modName] is ns
		for name, value in ns.items():
			if isinstance(value, types.ModuleType) and getattr(value, '_isScenicModule', False):
				registerNamespace(value.__name__, value.__dict__)
			else:
				# Convert values requiring sampling to Distributions
				dval = toDistribution(value)
				if dval is not value:
					ns[name] = dval
	for behavior in veneer.behaviors:
		modName = behavior.__module__
		globalNamespace = behavior.makeGenerator.__globals__
		registerNamespace(modName, globalNamespace)

	namespace['_behaviorNamespaces'] = behaviorNamespaces

def constructScenarioFrom(namespace):
	"""Build a Scenario object from an executed Scenic module."""
	# Extract ego object
	if namespace['_egoObject'] is None:
		raise InvalidScenarioError('did not specify ego object')

	# Extract workspace, if one is specified
	if 'workspace' in namespace:
		workspace = namespace['workspace']
		if not isinstance(workspace, Workspace):
			raise InvalidScenarioError(f'workspace {workspace} is not a Workspace')
		if needsSampling(workspace):
			raise InvalidScenarioError('workspace must be a fixed region')
		if needsLazyEvaluation(workspace):
			raise InvalidScenarioError('workspace uses value undefined '
			                           'outside of object definition')
	else:
		workspace = None

	# Extract simulator, if one is specified
	if 'simulator' in namespace:
		simulator = namespace['simulator']
		if not isinstance(simulator, Simulator):
			raise InvalidScenarioError(f'simulator {simulator} is not a Simulator')
	else:
		simulator = None

	# Create Scenario object
	scenario = Scenario(workspace, simulator,
	                    namespace['_objects'], namespace['_egoObject'],
	                    namespace['_params'], namespace['_externalParams'],
	                    namespace['_requirements'], namespace['_requirementDeps'],
                        namespace['_monitors'], namespace['_behaviorNamespaces'])

	# Prune infeasible parts of the space
	if usePruning:
		pruning.prune(scenario, verbosity=verbosity)

	return scenario
