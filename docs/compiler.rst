Guide to the Scenic Parser & Compiler
=====================================

Architecture & Terminology
--------------------------

.. figure:: /images/parser_architecture.png
  :alt: Scenic Parser & Compiler Architecture
  :figclass: align-center

Scenic AST
~~~~~~~~~~


A Scenic AST is an abstract syntax tree for representing Scenic programs.
It is a superset of Python AST and includes nodes for Scenic-specific
language constructs.

``syntax/ast.py`` defines all Scenic-specific AST nodes. Note that all
Scenic AST nodes must extend the ``AST`` class defined in the same file.

AST nodes should include fields to store objects. To add fields, add a
parameter to the initializer and define fields by assigning values to
``self``.

When adding fields, be sure to update the ``_fields`` and
``__match_args__`` fields. ``__fields`` lists the names of the fields in
the AST node and is used by the AST module to traverse the tree, fill in
the missing information, etc. ``__match_args__`` is used by the test
suite to assert the structure of the AST node using Python's structural
pattern matching.

Scenic Grammar
~~~~~~~~~~~~~~

Scenic Grammar (``scenic.gram``) is a formal grammar that defines the syntax
of the Scenic language. It is written as a parsing expression grammar
(PEG) using `the Pegen parser generator <https://we-like-parsers.github.io/pegen/index.html>`_.

Please refer to `Pegen's documentation <https://we-like-parsers.github.io/pegen/grammar.html>`_ on how to write a grammar.

Scenic Parser
~~~~~~~~~~~~~

Scenic Parser takes a Scenic source code and outputs the corresponding
abstract syntax tree. It is generated from the grammar file using Pegen.

When you modify ``scenic.gram``, you need to generate the parser again.
Call ``make`` at the project root to do so. Pytest is configured to
update the parser before test execution.

``test_parser.py`` includes parser tests and ensures that the parser
generates the desired AST.

Scenic Compiler
~~~~~~~~~~~~~~~

Scenic Compiler is a Scenic AST-to-Python AST compiler. The generated
Python AST can be passed to the Python interpreter for execution.

Internally, Scenic Compiler is a subclass of ``NodeTransformer``. It
must define visitors for each Scenic AST node and return corresponding
Python AST nodes.

Tutorial: Adding New Syntax
---------------------------

In order to add new syntax, you'll want to do the following:

1. add AST nodes to ``ast.py``
2. add grammar to ``scenic.gram``
3. write parser tests
4. add visitor to ``compiler.py``
5. write compiler tests

The rest of this section will demonstrate how we can add the ``implies``
operator using the new parser architecture.

Step 1: AST Nodes
~~~~~~~~~~~~~~~~~

First, we define AST nodes that represent the syntax. Since the
``implies`` operator is a binary operator, the AST node will have two
fields for each operand.

.. code:: python

   class ImpliesOp(AST): # 1
       __match_args__ = ("hypothesis", "conclusion") # 2

   		# 3
       def __init__(
           self, hypothesis: ast.AST, conclusion: ast.AST, *args: any, **kwargs: any
       ) -> None:
           super().__init__(*args, **kwargs)
           self.hypothesis = hypothesis
           self.conclusion = conclusion
   				# 4
           self._fields = ["hypothesis", "conclusion"]

1. ``AST`` (not ``ast.AST`` ) is a Scenic base class that all Scenic AST
   nodes extends.
2. ``__match_args__`` is a syntax for using `structural pattern
   matching <https://peps.python.org/pep-0636/#matching-positional-attributes>`__
   on argument positions. This is to make it easier to write parser
   tests.
3. The initializer takes two operands (hypothesis and conclusion). Note
   that their types are ``ast.AST`` , which is a base for all AST nodes,
   including both Scenic AST nodes and Python AST nodes. ``*args`` and
   ``**kwargs`` should be passed to the base class’ initializer to store
   line number, offset, etc.
4. ``_fields`` is a special field that specifies the child nodes. This is used by
   the library functions such as ``generic_visit`` to traverse the
   syntax tree.

Step 2: Add Grammar
~~~~~~~~~~~~~~~~~~~

.. note::
    
    The grammar described here is simplified for the sake of brevity and is not the actual grammar used in the Scenic language.

The next step is to update the ``scenic.gram`` file.

::

   scenic_implication (memo):
       | invalid_scenic_implication
       | a=disjunction 'implies' b=disjunction { s.ImpliesOp(a, b, LOCATIONS) }
       | disjunction

In the grammar, precedence and associativity is given by using
separate rules for each precedence. Since implication should bind less tightly than ``or``,
it takes two ``disjunction``\ s for its operands.

All Scenic grammar rules should be prefixed with ``scenic_`` so we can
easily distinguish Scenic-specific rules from those in the original Python grammar.

The production rule is defined inside the brackets. ``s`` refers to the
Scenic AST module and we can write the plain Python to construct the AST
node. ``LOCATIONS`` will be replaced with a set of named arguments to
express source code locations.

The ``implies`` operator is unique in that it takes exactly two
operands. Since the associativity of ``implies`` operator is not clear,
developers must specify the associativity by explicitly wrapping one of
the operands with parenthesis.

Rules with the ``invalid_`` prefix are special rules for generating
custom error messages. Pegen first tries to parse the input without
using ``invalid_`` rules. If it fails, it tries again with ``invalid_``
rules and generate errors if there is a matching rule.

::

   invalid_scenic_implication[NoReturn]:
       | a=disjunction 'implies' implication {
           self.raise_syntax_error_known_location(
               f"`implies` must take exactly two operands", a
           )
        }

The ``invalid_scenic_implication`` rule looks for an implication with more
than two arguments (e.g. ``A implies B implies C``) and raises a syntax
error with a detailed error message.

Once we are done with the grammar, run ``make`` to generate the parser
from the grammar. If there is no error, ``parser.py`` will be created at
``src/scenic/syntax``.

Step 3: Write Parser Tests
~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have parser, we need to add test cases to check that the
parser works as we expect.

The number of test cases depends on the complexity of the grammar rule.
Here, I decided to add the following three cases:

.. code:: python

   class TestOperator: # 1
       def test_implies_basic(self): # 2
           mod = parse_string_helper("x implies y") # 3
           stmt = mod.body[0]
           match stmt:
               case Expr(ImpliesOp(Name("x"), Name("y"))): # 4
                   assert True
               case _:
                   assert False # 5

       def test_implies_precedence(self):
           mod = parse_string_helper("x implies y or z")
           stmt = mod.body[0]
           match stmt:
               case Expr(ImpliesOp(Name("x"), BoolOp(Or(), [Name("y"), Name("z")]))):
                   assert True
               case _:
                   assert False

       def test_implies_three_operands(self):
   				# 6
           with pytest.raises(SyntaxError) as e:
               parse_string_helper("x implies y implies z")
           assert "must take exactly two operands" in e.value.msg

1. ``TestOperator`` is a test class that has all tests related to Scenic
   operators, so it is natural for us to add test cases here.
2. The test case name should contain the names of the grammar we’re
   testing (``implies`` in this case)
3. ``parse_string_helper`` is a thin wrapper around the parser. The
   return value would be a module, but we’re only concerned about the
   first statement of the body, so we extract that to the ``stmt``
   variable.
4. We use structural pattern matching to match the result with the
   expected AST structure. In this case, the statement is expected to be
   an ``Expr`` whose value is an ``ImpliesOp`` that takes ``Name``\ s,
   ``x`` and ``y``.
5. Be sure to add an otherwise case (with ``_``) and assert false.
   Otherwise, no error will be caught even if the returned node does not
   match the expected structure.
6. Errors can be tested with Pytest’s ``raises`` feature.
   https://docs.pytest.org/en/7.1.x/reference/reference.html?highlight=raises#pytest.raises

Step 4: Add Visitor to Compiler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The next step is to add a visitor to the compiler so it knows how to
compile the ImpliesOP AST Nodes to the corresponding Python AST.

In this case, we want to compile ``A implies B`` to a function call
``Implies(A, B)``.

.. code:: python

   class ScenicToPythonTransformer(ast.NodeTransformer):
   		def visit_ImpliesOp(self, node: s.ImpliesOp):
           return ast.Call(
               func=ast.Name(id="Implies", ctx=loadCtx),
               args=[self.visit(node.hypothesis), self.visit(node.conclusion)],
               keywords=[],
           )

NodeTransformer uses the class name of the AST node to determine the
visitor to be called. Since we need to support ``ImpliesOp``, we create
a method named ``visit_ImpliesOp``, which will be called every time an
instance of ``ImpliesOp`` is visited.

Inside the visitor, we construct a Call to a name ``Implies`` with
``node.hypothesis`` and ``node.conclusion`` as its arguments. Note that
they need to be visited with ``self.visit``; otherwise Scenic AST nodes
inside them won't be compiled.

Step 5: Write Compiler Tests
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Similarly to step 3, we add tests for the compiler.

.. code:: python

   def test_implies_op(self):
       node, _ = compileScenicAST(ImpliesOp(Name("x"), Name("y")))
       match node:
           case Call(Name("Implies"), [Name("x"), Name("y")]):
               assert True
           case _:
               assert False

``compileScenicAST`` is a function that invokes the node transformer. We
match the compiled node against the desired structure, which in this
case is a call to a function with two arguments.

This completes adding the ``implies`` operator.
