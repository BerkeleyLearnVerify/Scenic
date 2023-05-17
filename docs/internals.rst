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
   :recursive:

   scenic.core
   scenic.domains
   scenic.formats
   scenic.simulators
   scenic.syntax

The ``scenic`` module itself provides the top-level API for using Scenic: see :doc:`api`.

.. _how Scenic is compiled:

How Scenic is Compiled
----------------------

The process of compiling a Scenic program into a `Scenario` object can be split into several phases.
Understanding what each phase does is useful if you plan to modify the Scenic language.

If you want to make a change to the language, also see :doc:`compiler`.

Phase 1: Scenic Parser
~~~~~~~~~~~~~~~~~~~~~~
In this phase the program is parsed using the Scenic parser. The parser is generated from a PEG grammar (``scenic.gram``) using `the Pegen parser generator <https://we-like-parsers.github.io/pegen/index.html>`_.
The parser generates an abstract syntax tree (Scenic AST) for the program. Scenic AST is a superset of Python AST defined in ``ast.py`` and has additional nodes for representing Scenic-specific constructs.

Phase 2: Scenic Compiler
~~~~~~~~~~~~~~~~~~~~~~~~
In this phase, the Scenic AST is transformed into a Python AST. The Scenic Compiler walks the Scenic AST and replaces Scenic-specific nodes with corresponding Python AST nodes.

Phase 3: AST Compilation
~~~~~~~~~~~~~~~~~~~~~~~~
Compile the Python AST down to a Python ``code`` object.

Phase 4: Python Execution
~~~~~~~~~~~~~~~~~~~~~~~~~
In this phase the Python ``code`` object compiled in Phase 5 is executed.
When run, the definitions of objects, global parameters, requirements, behaviors, etc. produce Python data structures used internally by Scenic to keep track of the distributions, functions, coroutines, etc. used in their definitions.
For example, a random value will evaluate to a `Distribution` object storing information about which distribution it is drawn from; actually sampling from that distribution will not occur until after the compilation process (when calling `Scenario.generate`).
A ``require`` statement will likewise produce a closure which can be used at sampling time to check whether its condition is satisfied or not.

Note that since this phase only happens once, at compile time and not sampling time, top-level code in a Scenic program [#f1]_ is only executed **once** even when sampling many scenes from it.
This is done deliberately, in order to generate a static representation of the semantics of the Scenic program which can be used for sampling without needing to re-run the entire program.

Phase 5: Scenario Construction
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
