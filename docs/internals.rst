..  _internals:

Scenic Internals
================

Scenic Modules
--------------

This section of the documentation describes the implementation of Scenic.
Much of this information will probably only be useful for people who need to make some change to the language (e.g. adding a new type of distribution).
However, the detailed documentation on Scenic's abstract application domains (in `scenic.domains`) and simulator interfaces (in `scenic.simulators`) may be of interest to people using those features.

The documentation is organized by the submodules of the main ``scenic`` module:

.. autosummary::
   :toctree: modules

   scenic.core
   scenic.domains
   scenic.formats
   scenic.simulators
   scenic.syntax

The ``scenic`` module itself provides the top-level API for using Scenic: see :doc:`api`.

Scenic Pipeline
---------------

The scenic compilation can be split into several phases. Understanding what each phase is useful if you plan to modify the Scenic language and can help understand the sometimes cryptic Scenic error messages.

Phase 1: Import Handling
~~~~~~~~~~~~~~~~~~~~~~~~
In this phase Python and Scenic modules are imported. For more details on the various rules for import statements in Scenic, see :ref:`import *module*`.

Phase 2: Token Translation
~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the Scenic program is tokenized using the Python tokenizer. Many of the resulting tokens are meaningless to the Python parser however, which we use to parse Scenic. To remedy this, all Scenic specific tokens are translated into sequences of tokens that will pass through the Python parser if and only if they are syntactically valid Scenic.

Phase 3: Python Parser
~~~~~~~~~~~~~~~~~~~~~~
In this phase we parse the token stream resulting from Phase 2 using the Python parser. If the tokens do not parse properly, i.e. they come from a malformed Scenic program, an error will be thrown here. However, since the tokens being parsed are not the original Scenic tokens and likely not semantically valid Python either, the error messages can be quite confusing.

Phase 4: Python AST Modification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase we crawl the AST outputted from Phase 3 to transform it into a Python AST with equivalent semantics to the original Scenic program. In other words, now that we have checked that our Scenic program is syntactically valid and parsed the modified version of it, we need to undo those modifications as they oftentimes do not form semantically valid Python. 

Phase 5: AST Compilation
~~~~~~~~~~~~~~~~~~~~~~~~
TBD

Phase 6: Python Execution
~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the Python code outputted by Phase 5, which is semantically equivalent to our initial Scenic program, is executed.

Phase 7: Scenario Construction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase a Scenario object is extracted from the Python environment that results from phase 6, which is then returned to the user.
