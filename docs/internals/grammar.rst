.. _formal grammar:

Scenic Grammar
==============

This page gives the formal `Parsing Expression Grammar <https://en.wikipedia.org/wiki/Parsing_expression_grammar>`_ (PEG) used to parse the Scenic language.
It is in the format of `the Pegen parser generator <https://we-like-parsers.github.io/pegen/index.html>`_, and is based on the Python grammar from `CPython <https://github.com/python/cpython>`_ (see :file:`Grammar/python.gram` in the CPython repository).
In the source code, the grammar can be found at :file:`src/scenic/syntax/scenic.gram`.

.. literalinclude:: /../src/scenic/syntax/scenic.gram
    :language: pegen
