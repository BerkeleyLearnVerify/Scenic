..  _developing:

Developing Scenic
=================

This page covers information useful if you will be developing Scenic, either changing the
language itself or adding new built-in libraries or simulator interfaces.

Getting Started
---------------

Start by cloning our repository on GitHub and installing Poetry as described in
:doc:`quickstart`. When using Poetry to install Scenic in your virtual environment, use
the command :command:`poetry install -E dev` to make sure you get all the dependencies
needed for development.

Running the Test Suite
----------------------

Scenic has an extensive test suite exercising most of the features of the language. We
use the `pytest <https://docs.pytest.org/en/latest/index.html>`_ Python testing tool. To
run the entire test suite, run the command :command:`pytest` inside the virtual
environment.

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
this: at the beginning of each run of ``pytest``, it prints out a line like::

	Using --randomly-seed=344295085

Adding this as an option, i.e. running :command:`pytest --randomly-seed=344295085`, will
reproduce the same sequence of tests with the same Python/Scenic random seed.

Debugging
---------

You can use Python's built-in debugger `pdb` to debug the parsing, compilation, sampling,
and simulation of Scenic programs. The command-line option :option:`-b` will cause the
backtraces printed from uncaught exceptions to include Scenic's internals; you can also
use the :option:`--pdb` option to automatically enter the debugger on such exceptions.
