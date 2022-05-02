..  _internals:

Scenic Internals
================

This section of the documentation describes the implementation of Scenic.
Much of this information will probably only be useful for people who need to make some change to the language (e.g. adding a new type of distribution).
However, the detailed documentation on Scenic's abstract application domains (in `scenic.domains`) and simulator interfaces (in `scenic.simulators`) may be of interest to people using those features.

Scenic Modules
--------------

Detailed documentation on Scenic's components is organized by the submodules of the main ``scenic`` module:

.. autosummary::
   :toctree: modules

   scenic.core
   scenic.domains
   scenic.formats
   scenic.simulators
   scenic.syntax

The ``scenic`` module itself provides the top-level API for using Scenic: see :doc:`api`.

How Scenic is Compiled
----------------------

.. note::

   This section describes the current compiler, which goes through various convolutions
   in order to make use of the Python parser as much as possible. This has the advantage
   of allowing almost all Python constructs to be used in Scenic, but the disadvantage of
   sometimes producing cryptic error messages. We are in the process of writing a native
   parser for Scenic which will replace Phases 1-4 below.

The process of compiling a Scenic program into a `Scenario` object can be split into several phases.
Understanding what each phase does is useful if you plan to modify the Scenic language.

Phase 1: Import Handling
~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the program is segmented into blocks ending with ``import`` statements, which will then be compiled separately.
This is done so that if a Scenic module defining new Scenic classes is imported, subsequent parsing can properly detect instantiations of those classes.
For more details on the various rules for import statements in Scenic, see :sampref:`import`.

Phase 2: Token Translation
~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the Scenic program is tokenized using the Python tokenizer.
Various Scenic constructs yield sequences of tokens which are meaningless to the Python parser, which we will use to parse the code in the next phase.
To remedy this, all Scenic-specific token sequences are translated into new sequences that will pass through the Python parser if they are syntactically-valid Scenic.

Phase 3: Python Parser
~~~~~~~~~~~~~~~~~~~~~~
In this phase we parse the token stream resulting from Phase 2 using the Python parser.
If the tokens do not parse properly, they come from a malformed Scenic program, and an error will be raised here.
Unfortunately, since the tokens being parsed are not the original Scenic tokens and likely not semantically-valid Python either, the error messages can be confusing.

Phase 4: Python AST Modification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase we walk the AST generated in Phase 3 to transform it into a Python AST with equivalent semantics to the original Scenic program.
This undoes the translation performed in Phase 2 to get syntactically-valid Python, restoring the intended semantics of the program.

Phase 5: AST Compilation
~~~~~~~~~~~~~~~~~~~~~~~~
Compile the Python AST down to a Python ``code`` object.

Phase 6: Python Execution
~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the Python ``code`` object compiled in Phase 5 is executed.
When run, the definitions of objects, global parameters, requirements, behaviors, etc. produce Python data structures used internally by Scenic to keep track of the distributions, functions, coroutines, etc. used in their definitions.
For example, a random value will evaluate to a `Distribution` object storing information about which distribution it is drawn from; actually sampling from that distribution will not occur until after the compilation process (when calling `Scenario.generate`).
A ``require`` statement will likewise produce a closure which can be used at sampling time to check whether its condition is satisfied or not.

Note that since this phase only happens once, at compile time and not sampling time, top-level code in a Scenic program [#f1]_ is only executed **once** even when sampling many scenes from it.
This is done deliberately, in order to generate a static representation of the semantics of the Scenic program which can be used for sampling without needing to re-run the entire program.

Phase 7: Scenario Construction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the various pieces of the internal representation of the program resulting from Phase 6 are bundled into a `Scenario` object and returned to the user.
This phase is also where the program is analyzed and pruning techniques applied to optimize the scenario for later sampling.

Sampling and Executing Scenarios
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Sampling scenes and executing dynamic simulations from them are not part of the compilation process [#f2]_.
For documentation on how those are done, see `Scenario.generate` and `scenic.core.simulators` respectively.


.. [#f1] As compared to code inside a ``require`` statement or a :term:`dynamic behavior`,
   which will execute every time a scene is sampled or a simulation is run respectively.

.. [#f2] Although there are some syntax errors which are currently not detected until those stages.
