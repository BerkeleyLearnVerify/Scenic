..  _developing:

Developing Scenic
=================

This page covers information useful if you will be developing Scenic, either changing the
language itself or adding new built-in libraries or simulator interfaces.

To find documentation (and code) for specific parts of Scenic's implementation, see our page on :doc:`internals`.

Getting Started
---------------

Start by cloning our repository on GitHub and setting up your virtual environment.
Then to install Scenic and its development dependencies in your virtual environment run:

.. code-block:: console

	$ python -m pip install -e ".[dev]"

This will perform an "editable" install, so that any changes you make to Scenic's code will take effect immediately when running Scenic in your virtual environment.

Scenic uses the `isort <https://pycqa.github.io/isort/>`_ and `black <https://black.readthedocs.io/en/stable/index.html>`_ tools to automatically sort ``import`` statements and enforce a consistent code style.
Run the command :command:`pre-commit install` to set up hooks which will run every time you commit and correct any formatting problems (you can then inspect the files and try committing again).
You can also manually run the formatters on the files changed since the last commit with :command:`pre-commit run`. [#f1]_

Running the Test Suite
----------------------

Scenic has an extensive test suite exercising most of the features of the language. We
use the `pytest <https://docs.pytest.org/en/latest/index.html>`_ Python testing tool. To
run the entire test suite, run the command :command:`pytest` inside the virtual
environment from the root directory of the repository.

Some of the tests are quite slow, e.g. those which test the parsing and construction of
road networks. We add a ``--fast`` option to pytest	which skips such tests, while
still covering all of the core features of the language. So it is convenient to often run
:command:`pytest --fast` as a quick check, remembering to run the full :command:`pytest`
before making any final commits. You can also run specific parts of the test suite with a
command like :command:`pytest tests/syntax/test_specifiers.py`, or use pytest's ``-k``
option to filter by test name, e.g. :command:`pytest -k specifiers`.

Note that many of Scenic's tests are probabilistic, so in order to reproduce a test
failure you may need to set the random seed. We use the
`pytest-randomly <https://github.com/pytest-dev/pytest-randomly>`_ plugin to help with
this: at the beginning of each run of ``pytest``, it prints out a line like:

.. code-block:: none

	Using --randomly-seed=344295085

Adding this as an option, i.e. running :command:`pytest --randomly-seed=344295085`, will
reproduce the same sequence of tests with the same Python/Scenic random seed. As a
shortcut, you can use :command:`--randomly-seed=last` to use the seed from the previous
testing run.

If you're running the test suite on a headless server or just want to stop windows from
popping up during testing, use the :command:`--no-graphics` option to skip graphical
tests.

.. _debugging:

Debugging
---------

You can use Python's built-in debugger `pdb` to debug the parsing, compilation, sampling,
and simulation of Scenic programs. The Scenic command-line option :option:`-b` will cause the
backtraces printed from uncaught exceptions to include Scenic's internals; you can also
use the :option:`--pdb` option to automatically enter the debugger on such exceptions.
If you're trying to figure out why a scenario is taking many iterations of rejection
sampling, first use the :option:`--verbosity` option to print out the reason for each
rejection. If the problem doesn't become clear, you can use the :option:`--pdb-on-reject`
option to automatically enter the debugger when a scene or simulation is rejected.

If you're using the Python API instead of invoking Scenic from the command line, these
debugging features can be enabled using the following function from the ``scenic`` module:

.. autofunction:: scenic.setDebuggingOptions

It is possible to put breakpoints into a Scenic program using the Python built-in
function `breakpoint`. Note however that since code in a Scenic program is not always
executed the way you might expect (e.g. top-level code is only run once, whereas code in
requirements can run every time we generate a sample: see :ref:`how Scenic is compiled`), some care is needed when
interpreting what you see in the debugger. The same consideration applies when adding
`print` statements to a Scenic program. For example, a top-level :scenic:`print(x)` will
not print out the actual value of :scenic:`x` every time a sample is generated: instead,
you will get a single print at compile time, showing the `Distribution` object which
represents the distribution of :scenic:`x` (and which is bound to :scenic:`x` in the Python
namespace used internally for the Scenic module).

Building the Documentation
--------------------------

Scenic's documentation is built using `Sphinx <https://www.sphinx-doc.org/>`_. The
freestanding documentation pages (like this one) are found under the :file:`docs`
folder, written in the :ref:`reStructuredText format <rst-primer>`.
The detailed documentation of Scenic's internal classes, functions, etc. is largely
auto-generated from their docstrings, which are written in a variant of Google's style
understood by the `Napoleon <sphinx.ext.napoleon>`
Sphinx extension (see the docstring of `Scenario.generate` for a simple example: click
the ``[source]`` link to the right of the function signature to see the code).

If you modify the documentation, you should build a copy of it locally to make sure
everything looks good before you push your changes to GitHub (where they will be picked
up automatically by `ReadTheDocs <https://readthedocs.org/>`_). To compile the
documentation, enter the :file:`docs` folder and run :command:`make html`. The output
will be placed in the :file:`docs/_build/html` folder, so the root page will be at
:file:`docs/_build/html/index.html`. If your changes do not appear, it's possible that Sphinx
has not detected them; you can run :command:`make clean` to delete all the files from the
last compilation and start from a clean slate.

Scenic extends Sphinx in a number of ways to improve the presentation of Scenic code and
add various useful features: see :file:`docs/conf.py` for full details. Some of the most
commonly-used features are:

	* a ``scenic`` `role <https://www.sphinx-doc.org/en/master/usage/restructuredtext/roles.html>`_
	  which extends the standard Sphinx :rst:role:`samp` role with Scenic syntax highlighting;
	* a ``sampref`` role which makes a cross-reference like :rst:role:`keyword` but allows
	  emphasizing variables like :rst:role:`samp`;
	* the :rst:role:`term` role for glossary terms is extended so that the cross-reference will
	  work even if the link is plural but the glossary entry is singular or vice versa.

.. rubric:: Footnotes

.. [#f1] To run the formatters on *all* files, changed or otherwise, use :command:`make format` in the root directory of the repository. But this should not be necessary if you installed the pre-commit hooks and so all files already committed are clean.
