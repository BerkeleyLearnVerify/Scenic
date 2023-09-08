Guide to the Scenic Parser & Compiler
=====================================

This page describes the process of parsing Scenic code and compiling it into equivalent Python.
We also include a tutorial illustrating how to add a new syntax construct to Scenic.

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

The `scenic.syntax.ast` module defines all Scenic-specific AST nodes, which are instances of the `AST` class defined in the same file.

AST nodes should include fields to store objects. To add fields, add a
parameter to the initializer and define fields by assigning values to
``self``.

When adding fields, be sure to update the ``_fields`` and
``__match_args__`` fields. ``_fields`` lists the names of the fields in
the AST node and is used by the AST module to traverse the tree, fill in
the missing information, etc. :obj:`~object.__match_args__` is used by the test
suite to assert the structure of the AST node using Python's structural
pattern matching.

Scenic Grammar
~~~~~~~~~~~~~~

The Scenic Grammar (:file:`syntax/scenic.gram`) is a formal grammar that defines the syntax
of the Scenic language. It is written as a `Parsing Expression Grammar <https://en.wikipedia.org/wiki/Parsing_expression_grammar>`_
(PEG) using `the Pegen parser generator <https://we-like-parsers.github.io/pegen/index.html>`_.

Please refer to `Pegen's documentation <https://we-like-parsers.github.io/pegen/grammar.html>`_ on how to write a grammar.

Scenic Parser
~~~~~~~~~~~~~

The Scenic Parser takes Scenic source code and outputs the corresponding
abstract syntax tree. It is generated from the grammar file using Pegen.

When you modify :file:`scenic.gram`, you need to regenerate the parser
by calling :command:`make` or running

.. code-block:: console

    $ python -m pegen ./src/scenic/syntax/scenic.gram -o ./src/scenic/syntax/parser.py

at the project root.
When running the test suite with :command:`pytest`, the parser is automatically updated before test execution.

:file:`tests/syntax/test_parser.py` includes parser tests and ensures that the parser
generates the desired AST.

Scenic Compiler
~~~~~~~~~~~~~~~

The Scenic Compiler is a Scenic AST-to-Python AST compiler. The generated
Python AST can be passed to the Python interpreter for execution.

Internally, the compiler is a subclass of `ast.NodeTransformer`. It
must define visitors for each Scenic AST node which return corresponding
Python AST nodes.

Tutorial: Adding New Syntax
---------------------------

In order to add new syntax, you'll want to do the following:

1. add AST nodes to :file:`ast.py`
2. add grammar to :file:`scenic.gram`
3. write parser tests
4. add visitor to :file:`compiler.py`
5. write compiler tests

The rest of this section will demonstrate how we can add the :keyword:`implies`
operator using the new parser architecture.

Step 1: Add AST Nodes
~~~~~~~~~~~~~~~~~~~~~

First, we define AST nodes that represent the syntax. Since the
``implies`` operator is a binary operator, the AST node will have two
fields for each operand.

.. code-block:: python
    :linenos:

    class ImpliesOp(AST):
        __match_args__ = ("hypothesis", "conclusion")

        def __init__(
           self, hypothesis: ast.AST, conclusion: ast.AST, *args: Any, **kwargs: Any
        ) -> None:
           super().__init__(*args, **kwargs)
           self.hypothesis = hypothesis
           self.conclusion = conclusion
           self._fields = ["hypothesis", "conclusion"]

* On line 1, `AST` (`scenic.syntax.ast.AST`, not :external:obj:`ast.AST`) is the base class that all Scenic AST nodes extend.

* On line 2, ``__match_args__`` is a syntax for using `structural pattern
  matching <https://peps.python.org/pep-0636/#matching-positional-attributes>`__
  on argument positions. This is to make it easier to write parser tests.

* On line 5, the initializer takes two required arguments corresponding to the operator's operands (``hypothesis`` and ``conclusion``). Note
  that their types are :external:obj:`ast.AST`, which is the base class for *all* AST nodes,
  including both Scenic AST nodes and Python AST nodes. The additional arguments ``*args`` and
  ``**kwargs`` should be passed to the base class’ initializer to store
  extra information such as line number, offset, etc.

* On line 10, ``_fields`` is a special field that specifies the child nodes. This is used by
  the library functions such as ``generic_visit`` to traverse the
  syntax tree.

Step 2: Add Grammar
~~~~~~~~~~~~~~~~~~~

.. note::

    The grammar described here is slightly simplified for the sake of brevity.
    For the actual grammar used by the parser, see the `formal grammar`.

The next step is to update the :file:`scenic.gram` file with a rule that matches our new construct.
We'll add a rule called ``scenic_implication``: all Scenic grammar rules should be prefixed with ``scenic_`` so that we can
easily distinguish Scenic-specific rules from those in the original Python grammar.

.. code-block:: pegen

   scenic_implication (memo):
       | invalid_scenic_implication  # special rule to explain invalid uses of "implies"
       | a=disjunction "implies" b=disjunction { s.ImpliesOp(a, b, LOCATIONS) }
       | disjunction

Our rule has three alternatives, which the parser considers in order.
For the moment, let's consider the second alternative, which is the one defining the actual syntax of ``implies``: it matches any text matching the ``disjunction`` rule, followed by the word ``implies``, followed by any text matching the ``disjunction`` rule.
In the grammar, precedence and associativity of operators are defined by using
separate rules for each precedence level.
The ``disjunction`` rule matches any expression defined using ``or`` or an operator with higher precedence than ``or``.
Since implication should bind less tightly than ``or``, we use ``disjunction`` for its operands in our rule.
To allow ``scenic_implication`` to match higher-precedence operators as well as just ``implies``, we add the third alternative, which matches any ``disjunction``.

Returning to the second alternative, we define its outcome, i.e., the AST node which it generates if it matches, using the ordinary Python code inside the curly brackets.
Here ``s`` refers to the Scenic AST module, so :python:`s.ImpliesOp(a, b, LOCATIONS)` creates an instance of the ``ImpliesOp`` class we defined above with ``a`` the ``hypothesis`` and ``b`` the ``conclusion``.
The special term ``LOCATIONS`` will be replaced with a set of named arguments to
express source code locations.

The ``implies`` operator is unique in that it takes exactly two
operands: we disallow :requirement:`A implies B implies C` as being ambiguous, rather than parsing it as :requirement:`(A implies B) implies C` (left-associatively) or :requirement:`A implies (B implies C)` (right-associatively).
In order to block the ambiguous case and force the developer to make the meaning clear by wrapping one of the operands in parentheses, our rule says that the right-hand side of the implication must be a ``disjunction`` rather than an arbitrary expression.
This will cause the code :requirement:`A implies B implies C` to result in a syntax error, because no rules will match.

In order to replace the generic syntax error with a more informative one, we add the ``invalid_scenic_implication`` rule as the first alternative.
Rules with the ``invalid_`` prefix are special rules for generating
custom error messages.
Pegen first tries to parse the input *without*
using ``invalid_`` rules. If that fails, it tries parsing again, this time allowing ``invalid_``
rules: those rules can then generate errors when they match.

.. code-block:: pegen

   invalid_scenic_implication[NoReturn]:
       | a=disjunction "implies" disjunction "implies" b=disjunction {
           self.raise_syntax_error_known_range(
               f"`implies` must take exactly two operands", a, b
           )
        }

The ``invalid_scenic_implication`` rule looks for an implication with more
than two arguments (e.g. :requirement:`A implies B implies C`) and raises a syntax
error with a detailed error message.

Once we are done with the grammar, run :command:`make` to generate the parser
from the grammar. If there is no error, the file :file:`src/scenic/syntax/parser.py` will be created.

Step 3: Write Parser Tests
~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the parser, we need to add test cases to check that it works as we expect.

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
           with pytest.raises(SyntaxError) as e:  # 6
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
6. Errors can be tested using `pytest.raises`.

Step 4: Add Visitor to Compiler
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The next step is to add a visitor method to the compiler so it knows how to
compile the ``ImpliesOp`` AST node to the corresponding Python AST.
In this case, we want to compile :requirement:`A implies B` to a Python function call
:python:`Implies(A, B)`.

The visitor class used in the compiler, ``ScenicToPythonTransformer``, is a subclass of `ast.NodeTransformer`, which transforms an AST node of class ``C`` by calling a method called ``visit_C`` if one exists, otherwise just recursively transforming its child nodes.
So to add the ability to compile ``ImpliesOp`` nodes, we'll add
a method named ``visit_ImpliesOp``:

.. code:: python

   class ScenicToPythonTransformer(ast.NodeTransformer):
        def visit_ImpliesOp(self, node: s.ImpliesOp):
           return ast.Call(
               func=ast.Name(id="Implies", ctx=loadCtx),
               args=[self.visit(node.hypothesis), self.visit(node.conclusion)],
               keywords=[],
           )

Inside the visitor, we construct a Call to a name ``Implies`` with
:python:`node.hypothesis` and :python:`node.conclusion` as its arguments. Note that
the arguments need to be recursively visited using :python:`self.visit`; otherwise Scenic AST nodes
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
