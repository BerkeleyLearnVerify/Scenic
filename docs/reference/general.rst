
***********************
General Notes on Syntax
***********************

Keywords
========

.. rubric:: Keywords

The following words are reserved by Scenic and cannot be used as identifiers (i.e. as names of variables, functions, classes, properties, etc.).

.. literalinclude:: /_build/keywords.txt
    :language: text

.. rubric:: Soft Keywords

The following words have special meanings in Scenic in certain contexts, but are still available for use as identifiers.
Users should take care not to use these names when doing so would introduce ambiguity.
For example, consider the following code::

    distance = 5  # not a good variable name to use here
    new Object beyond A by distance from B

This might appear to use the three-argument form of the :keyword:`beyond` specifier, creating the new object at distance 5 beyond ``A`` from the point of view of ``B``.
But in fact Scenic parses the code as :specifier:`beyond A by (distance from B)`, because the interpretation of ``distance`` as being part of the :keyword:`distance from` operator takes precedence.

To avoid confusion, we recommend not using ``distance``, ``angle``, ``offset``, ``altitude``, or ``visible`` as identifiers in code that uses Scenic operators or specifiers (inside pure-Python helper functions is fine).

.. literalinclude:: /_build/keywords_soft.txt
    :language: text
