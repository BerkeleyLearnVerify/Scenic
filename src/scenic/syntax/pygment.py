"""Pygments lexer for Scenic."""

from pygments.lexers.python import PythonLexer

class ScenicLexer(PythonLexer):
    """Lexer for Scenic code. Currently just uses the Python lexer."""
    name = 'Scenic'
    aliases = ['scenic']
    filenames = ['*.scenic', '*.sc']
