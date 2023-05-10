"""Pygments lexer and style for Scenic.

These work with the `Pygments syntax highlighter <https://pygments.org/>`_.
The module actually defines several lexers used for the Scenic documentation;
the main `ScenicLexer` and its associated style `ScenicStyle` are exported by
:file:`pyproject.toml` as plugins to Pygments. This means that if you have the
``scenic`` package installed, the Pygments command-line tool and Python API
will automatically recognize Scenic files. For example, to highlight a Scenic
program as a self-contained HTML or LaTeX file:

.. code-block:: console

    $ pygmentize -f html -Ofull,style=scenic prog.scenic > out.html
    $ pygmentize -f latex -Ofull,style=scenic prog.scenic > out.tex

If highlighting multiple pieces of code, remove the ``full`` option to avoid
having the requisite CSS/preamble material duplicated in all your outputs; you
can run :command:`pygmentize -S scenic -f html` (or :command:`latex`) to
generate that material separately.
"""

import re

try:
    from pygments.lexers.python import PythonLexer
except ModuleNotFoundError as e:
    raise ImportError(
        "need the 'pygments' package to use the Scenic Pygments lexer"
    ) from e
from pygments.lexer import (
    RegexLexer,
    bygroups,
    combined,
    default,
    include,
    inherit,
    words,
)
from pygments.token import *
from pygments.style import Style
from pygments.styles.default import DefaultStyle
import pygments.unistring as uni


class BetterPythonLexer(PythonLexer):
    """Python lexer with better highlighting of function calls, parameters, etc.

    OK, 'better' is a matter of opinion; but it provides more informative tokens.
    These tokens will not cause errors under any Pygments style, but require the
    style to be aware of them in order to actually get better highlighting: use
    the `ScenicStyle` below for best results.

    Adapted from the PythonLexer and the MagicPython grammar by MagicStack Inc.,
    available at https://github.com/MagicStack/MagicPython.
    """

    name = "Python (a la MagicPython)"
    aliases = ["better-python"]

    uni_name = PythonLexer.uni_name

    tokens = {
        "root": [
            # This is largely copied from PythonLexer: the differences are noted below
            (r"\n", Whitespace),
            (
                r'^(\s*)([rRuUbB]{,2})("""(?:.|\n)*?""")',
                bygroups(Whitespace, String.Affix, String.Doc),
            ),
            (
                r"^(\s*)([rRuUbB]{,2})('''(?:.|\n)*?''')",
                bygroups(Whitespace, String.Affix, String.Doc),
            ),
            (r"\A#!.+$", Comment.Hashbang),
            (r"#.*$", Comment.Single),
            (r"\\\n", Punctuation),  # we count line continuations as punctuation
            (r"\\", Punctuation),  # ditto
            (r"\b(lambda)\b", Keyword, "lambda"),  # parse parameters of lambdas
            include("keywords"),
            include("soft-keywords"),
            (r"(def)((?:\s|\\\s)+)", bygroups(Keyword, Punctuation), "funcname"),
            (r"(class)((?:\s|\\\s)+)", bygroups(Keyword, Punctuation), "classname"),
            (
                r"(from)((?:\s|\\\s)+)",
                bygroups(Keyword.Namespace, Punctuation),
                "fromimport",
            ),
            (
                r"(import)((?:\s|\\\s)+)",
                bygroups(Keyword.Namespace, Punctuation),
                "import",
            ),
            include("extra-statements"),  # hook for subclasses
            # Parse expressions inside decorators
            (
                r"^(\s*)(\@)(\s*)",
                bygroups(Whitespace, Name.Decorator, Whitespace),
                "decorator",
            ),
            # Finally, we can have ordinary expressions
            include("expr"),
        ],
        "extra-statements": [],
        "decorator": [
            include("magicfuncs"),
            include("magicvars"),
            include("builtins"),
            (rf"({uni_name})(?=\s|\#)", bygroups(Name.Other.FunctionCall)),
            include("expr"),
            (r"(\#.*)?(?=\n)", bygroups(Comment.Single), "#pop"),
        ],
        "expr": [
            # allow comments
            (r"#.*$", Comment.Single),
            # properly track parentheses and brackets
            (r"\(", Punctuation, "parens"),
            (r"\[", Punctuation, "brackets"),
            (r"\{", Punctuation, "curly-brackets"),
            # the following is copied from PythonLexer
            # raw f-strings
            (
                '(?i)(rf|fr)(""")',
                bygroups(String.Affix, String.Double),
                combined("rfstringescape", "tdqf"),
            ),
            (
                "(?i)(rf|fr)(''')",
                bygroups(String.Affix, String.Single),
                combined("rfstringescape", "tsqf"),
            ),
            (
                '(?i)(rf|fr)(")',
                bygroups(String.Affix, String.Double),
                combined("rfstringescape", "dqf"),
            ),
            (
                "(?i)(rf|fr)(')",
                bygroups(String.Affix, String.Single),
                combined("rfstringescape", "sqf"),
            ),
            # non-raw f-strings
            (
                '([fF])(""")',
                bygroups(String.Affix, String.Double),
                combined("fstringescape", "tdqf"),
            ),
            (
                "([fF])(''')",
                bygroups(String.Affix, String.Single),
                combined("fstringescape", "tsqf"),
            ),
            (
                '([fF])(")',
                bygroups(String.Affix, String.Double),
                combined("fstringescape", "dqf"),
            ),
            (
                "([fF])(')",
                bygroups(String.Affix, String.Single),
                combined("fstringescape", "sqf"),
            ),
            # raw bytes and strings
            ('(?i)(rb|br|r)(""")', bygroups(String.Affix, String.Double), "tdqs"),
            ("(?i)(rb|br|r)(''')", bygroups(String.Affix, String.Single), "tsqs"),
            ('(?i)(rb|br|r)(")', bygroups(String.Affix, String.Double), "dqs"),
            ("(?i)(rb|br|r)(')", bygroups(String.Affix, String.Single), "sqs"),
            # non-raw strings
            (
                '([uU]?)(""")',
                bygroups(String.Affix, String.Double),
                combined("stringescape", "tdqs"),
            ),
            (
                "([uU]?)(''')",
                bygroups(String.Affix, String.Single),
                combined("stringescape", "tsqs"),
            ),
            (
                '([uU]?)(")',
                bygroups(String.Affix, String.Double),
                combined("stringescape", "dqs"),
            ),
            (
                "([uU]?)(')",
                bygroups(String.Affix, String.Single),
                combined("stringescape", "sqs"),
            ),
            # non-raw bytes
            (
                '([bB])(""")',
                bygroups(String.Affix, String.Double),
                combined("bytesescape", "tdqs"),
            ),
            (
                "([bB])(''')",
                bygroups(String.Affix, String.Single),
                combined("bytesescape", "tsqs"),
            ),
            (
                '([bB])(")',
                bygroups(String.Affix, String.Double),
                combined("bytesescape", "dqs"),
            ),
            (
                "([bB])(')",
                bygroups(String.Affix, String.Single),
                combined("bytesescape", "sqs"),
            ),
            # mostly copied from PythonLexer, with changes noted
            (r"[^\S\n]+", Text),
            include("numbers"),
            (r"\b(lambda)\b", Keyword, "lambda"),  # as above, parse lambdas in detail
            (r"!=|==|<<|>>|:=|[-~+/*%=<>&^|@]", Operator),  # include @ but not .
            (r"\.\.\.", Keyword.Constant),  # recognize ellipsis
            (r"\.", Operator.Member),  # special token for member access
            (r"[:,;]", Punctuation),  # exclude brackets (handled above)
            (r"(in|is|and|or|not)\b", Operator.Word),
            include("extra-exprs"),  # hook for subclasses
            include("expr-keywords"),
            (rf"\b(?={uni_name}\s*\()", Text, "function-call"),  # parse function calls
            include("builtins"),
            include("magicfuncs"),
            include("magicvars"),
            (r"(\\\n)", Punctuation),  # extra rule for line continuations
            include("name"),
        ],
        "parens": [
            (r"\)", Punctuation, "#pop"),
            (r"\n", Whitespace),
            include("expr"),
        ],
        "brackets": [
            (r"\]", Punctuation, "#pop"),
            (r"\n", Whitespace),
            include("expr"),
        ],
        "curly-brackets": [
            (r"\}", Punctuation, "#pop"),
            (r"\n", Whitespace),
            include("expr"),
        ],
        "extra-exprs": [],
        "funcname": [
            # parse not only the name, but the list of parameters
            include("magicfuncs"),
            (uni_name, Name.Function),
            (r"\\\n", Punctuation),
            (r"\s+", Whitespace),
            (r"\(", Punctuation, "params"),  # start param list
            (r"->", Punctuation, "return-annotation"),
            (r":", Punctuation, "#pop"),
        ],
        "params": [
            # Use a specific token for parameter names
            (r"(/)(?:(,)|(?=\s*\)))", bygroups(Operator, Punctuation)),
            (r"(\*)(\s*)(,)", bygroups(Operator, Whitespace, Punctuation)),
            (r"\*\*|\*", Operator),
            (
                rf"""(?x)
                (?: (self|cls) | ({uni_name})) (\s*)
                (?: (,) | (?=[)#\n=]))""",
                bygroups(
                    Name.Variable.Parameter.Magic,
                    Name.Variable.Parameter,
                    Whitespace,
                    Punctuation,
                ),
            ),
            (r"#.*$", Comment.Single),
            (r"\\\n", Punctuation),
            (r"=", Operator, "default"),
            (
                rf"\b({uni_name})(\s*)(:)",
                bygroups(Name.Variable.Parameter, Whitespace, Punctuation),
                "annotated-param",
            ),
            (r"\s+", Whitespace),
            (r"\)", Punctuation, "#pop"),
        ],
        "default": [
            (r"\s+", Whitespace),
            (r"(,)|(?=\))", bygroups(Punctuation), "#pop"),
            include("expr"),
        ],
        "annotated-param": [
            (r"\s+", Whitespace),
            (r"(,)|(?=\))", bygroups(Punctuation), "#pop"),
            include("expr"),
            (r"=(?!=)", Operator),
        ],
        "return-annotation": [
            (r"\s+", Whitespace),
            (r"(?=:)", Text, "#pop"),
            include("expr"),
        ],
        "lambda": [
            # Similar to 'params' above
            (r"\*\*|\*", Operator),
            (
                rf"({uni_name})(\s*)(?:(,)|(?=:|$))",
                bygroups(Name.Variable.Parameter, Whitespace, Punctuation),
            ),
            (r"#.*$", Comment.Single),
            (
                rf"\b({uni_name})(\s*)(=)",
                bygroups(Name.Variable.Parameter, Whitespace, Operator),
                "lambda-default",
            ),
            (r"\s+", Whitespace),
            (r"\\\n", Punctuation),
            (r"(:)|(\n)", bygroups(Punctuation, Whitespace), "#pop"),
        ],
        "lambda-default": [
            (r"(,)|(?=:|$)", bygroups(Punctuation), "#pop"),
            include("expr"),
        ],
        "function-call": [
            # Use a specific token for called functions, and parse arguments
            (r"\)", Punctuation, "#pop"),
            (r"\s+", Whitespace),
            (r"(self|cls)\b", Name.Builtin.Pseudo),
            include("magicfuncs"),
            include("magicvars"),
            include("builtins"),
            (rf"\b{uni_name}\b", Name.Other.FunctionCall),
            (r"\(", Punctuation, "function-arguments"),
        ],
        "function-arguments": [
            # Label names of default parameters as parameters
            (r"(?=\))(?!\)\s*\()", Text, "#pop"),
            (r",", Punctuation),
            (r"\*\*|\*", Operator),
            (
                rf"\b({uni_name})(\s*)(=)(?!=)",
                bygroups(Name.Variable.Parameter, Whitespace, Operator),
            ),
            include("expr"),
            (
                r"(\s*)(\))(\s*)(\()",
                bygroups(Whitespace, Punctuation, Whitespace, Punctuation),
            ),
            (r"\s+", Whitespace),
        ],
    }


class ScenicLexer(BetterPythonLexer):
    """Lexer for Scenic code."""

    name = "Scenic"
    aliases = ["scenic"]
    filenames = ["*.scenic"]
    alias_filenames = ["*.sc"]
    mimetypes = ["application/x-scenic", "text/x-scenic"]
    url = "https://scenic-lang.readthedocs.org/"

    uni_name = PythonLexer.uni_name
    obj_name = rf"(?:(ego)|({uni_name}))"

    simple_spec = words(
        (
            "at",
            "offset by",
            "offset along",
            "left of",
            "right of",
            "ahead of",
            "above",
            "below",
            "behind",
            "beyond",
            "visible from",
            "visible",
            "not visible from",
            "not visible",
            "in",
            "on",
            "contained in",
            "following",
            "facing toward",
            "facing away from",
            "facing directly toward",
            "facing directly away from",
            "facing",
            "apparently facing",
        )
    ).get()
    specifier = rf"""
        \b(?<!\.) (?:
            (with) (\s+) ({uni_name})
            | ({simple_spec})
        )\b"""

    simple_statements = words(
        (
            "simulator",
            "param",
            "require",
            "require monitor",
            "terminate when",
            "terminate after",
            "mutate",
            "record initial",
            "record final",
            "record",
            "take",
            "wait",
            "terminate",
            "do choose",
            "do shuffle",
            "do",
            "abort",
            "interrupt when",
        ),
        suffix=r"\b(?!\s*[:.;,])",
    ).get()

    constructible_types = ("Point", "OrientedPoint", "Object")
    primitive_types = (
        "Vector",
        "Orientation",
        "VectorField",
        "PolygonalVectorField",
        "Shape",
        "MeshShape",
        "BoxShape",
        "CylinderShape",
        "ConeShape",
        "SpheroidShape",
        "Region",
        "PointSetRegion",
        "RectangularRegion",
        "CircularRegion",
        "SectorRegion",
        "PolygonalRegion",
        "PolylineRegion",
        "PathRegion",
        "MeshVolumeRegion",
        "MeshSurfaceRegion",
        "BoxRegion",
        "SpheroidRegion",
        "Workspace",
        "Range",
        "DiscreteRange",
        "Options",
        "Discrete",
        "Uniform",
        "Normal",
        "TruncatedNormal",
        "VerifaiParameter",
        "VerifaiRange",
        "VerifaiDiscreteRange",
        "VerifaiOptions",
    )
    exceptions = ("GuardViolation", "PreconditionViolation", "InvariantViolation")
    builtin_names = (
        "everywhere",
        "nowhere",
        "resample",
        "localPath",
        "verbosePrint",
        "simulation",
        "sin",
        "cos",
        "hypot",
    )
    magic_names = ("ego", "workspace", "globalParameters")
    builtin_classes = words(constructible_types, suffix=r"\b").get()

    tokens = {
        "extra-statements": [
            # Scenario and other block definitions
            (
                rf"^(\s*)(scenario|behavior|monitor)(\s+)",
                bygroups(Whitespace, Keyword, Text),
                "scenario-behavior",
            ),
            # Model statement (which highlights the module name specially, like import)
            (r"^(\s*)(model)(\s+)", bygroups(Whitespace, Keyword, Whitespace), "model"),
            # Override statement (which ends with a list of specifiers)
            (
                rf"""(?x)
                ^(\s*) (override)
                (\s+) {obj_name} ([^\S\n]*)
                (?={specifier})""",
                bygroups(
                    Whitespace, Keyword, Whitespace, Name.Variable.Magic, Name, Whitespace
                ),
                "specifier-start",
            ),
            # Param statement
            (
                r"^(\s*)(param)(\s+)",
                bygroups(Whitespace, Keyword, Whitespace),
                "param-statement",
            ),
            # Require statement (within which we can have temporal operators)
            (
                r"^(\s*)(require)(\s+)(?!monitor)",
                bygroups(Whitespace, Keyword, Whitespace),
                "require-statement",
            ),
            # Keywords that can only occur at the start of a line
            (rf"^(\s*)({simple_statements})", bygroups(Whitespace, Keyword)),
            (
                r"^(\s*)(try|setup|compose|precondition|invariant)(:)",
                bygroups(Whitespace, Keyword, Punctuation),
            ),
            # Keywords that can only occur at the end of a line
            (r"(?<!\.)(seconds|steps)\b(?=\s*(\#.*)?$)", Keyword),
            # Property declarations
            (
                rf"""(?x)
                ^(\s+)
                (?!else|except|finally) ({uni_name})
                (\s*)
                (?: (\[) (?= [\w\s,]* \] \s* : \s* [^\s\#])
                    | (?= : \s* [^\s\#]))""",
                bygroups(Whitespace, Name.Variable.Instance, Whitespace, Punctuation),
                "property-declaration",
            ),
            # Instance creations
            (
                rf"""(?x)
                (new) ([^\S\n]+)
                (?:{builtin_classes}|({uni_name})) ([^\S\n]*)""",
                bygroups(
                    Keyword,
                    Whitespace,
                    Name.Builtin.Instance,
                    Name.Class.Instance,
                    Whitespace,
                ),
                "specifier-start",
            ),
        ],
        "scenario-behavior": [
            (uni_name, Name.Class),
            (r"\\\n", Punctuation),
            (r"\s+", Whitespace),
            (r"\(", Punctuation, "params"),
            (r":", Punctuation, "#pop"),
        ],
        "model": [
            (r"\.", Name.Namespace),
            (uni_name, Name.Namespace),
            default("#pop"),
        ],
        "param-statement": [
            (
                rf"""(?x)
                ({uni_name} | '[^\\\'"%{{\n]+' | "[^\\\'"%{{\n]+")
                (\s*) (=) (\s*)""",
                bygroups(Name.Variable.Global, Whitespace, Operator, Whitespace),
            ),
            include("expr"),
            (r"(\#.*)?\n", Comment.Single, "#pop"),
        ],
        "require-statement": [
            (
                words(("always", "eventually", "next", "until", "implies"), suffix=r"\b"),
                Keyword,
            ),
            (r"\(", Punctuation, "require-parens"),
            include("expr"),
            (r"(\#.*)?\n", Comment.Single, "#pop"),
        ],
        "require-parens": [
            (r"\)", Punctuation, "#pop"),
            (r"\n", Whitespace),
            include("require-statement"),
        ],
        "property-declaration": [
            (words(("additive", "dynamic", "final"), suffix=r"\b"), Keyword.Pseudo),
            (rf"{uni_name}\b", Text),
            (r"(,)(\s*)", bygroups(Punctuation, Whitespace)),
            (r"(\])?(\s*)(:)", bygroups(Punctuation, Whitespace, Punctuation), "#pop"),
        ],
        "specifier-start": [
            (
                r"(?x)" + specifier,
                bygroups(
                    Keyword.Specifier,
                    Whitespace,
                    Name.Variable.Instance,
                    Keyword.Specifier,
                ),
                "specifier-inner",
            ),
            (r"(\#.*)?\n", Comment.Single, "#pop"),
        ],
        "specifier-inner": [
            (
                r"""(?x)
             (,) (\s*) (?: (?: (\#.*)? | (\\)) (\n\s*))?
             | (\#.*)? (?=\n)""",
                bygroups(
                    Punctuation,
                    Whitespace,
                    Comment.Single,
                    Punctuation,
                    Whitespace,
                    Comment.Single,
                ),
                "#pop",
            ),
            include("expr"),
        ],
        "extra-exprs": [
            # Operators
            (
                words(
                    (
                        "deg",
                        "relative heading of",
                        "apparent heading of",
                        "distance from",
                        "distance to",
                        "distance past",
                        "angle from",
                        "angle to",
                        "altitude from",
                        "altitude to",
                        "can see",
                        "at",
                        "relative to",
                        "offset by",
                        "offset along",
                        "visible",
                        "not visible",
                        "front of",
                        "back of",
                        "left of",
                        "right of",
                        "front left of",
                        "front right of",
                        "back left of",
                        "back right of",
                    ),
                    prefix=r"(?<!\.)",
                    suffix=r"\b",
                ),
                Operator.Word,
            ),
            # Keywords that can occur anywhere (w.r.t. our simple analysis)
            (
                words(
                    ("initial scenario", "until", "to", "by", "from"),
                    prefix=r"(?<!\.)",
                    suffix=r"\b",
                ),
                Keyword,
            ),
        ],
        "builtins": [
            inherit,
            (
                words(
                    constructible_types + primitive_types + builtin_names,
                    prefix=r"(?<!\.)",
                    suffix=r"\b",
                ),
                Name.Builtin,
            ),
            (words(magic_names, prefix=r"(?<!\.)", suffix=r"\b"), Name.Variable.Magic),
            (words(exceptions, prefix=r"(?<!\.)", suffix=r"\b"), Name.Exception),
        ],
    }

    def analyse_text(text):
        score = 0
        if "new Object" in text:
            score += 0.2
        if "ego = new " in text:
            score += 0.2
        if " facing " in text:
            score += 0.2
        if "param " in text:
            score += 0.1
        if "require " in text:
            score += 0.1
        return score


class ScenicSnippetLexer(ScenicLexer):
    """Variant ScenicLexer for code snippets rather than complete programs.

    Specifically, this lexer formats syntactic variables of the form "{name}"
    as "name" italicized.
    """

    name = "Scenic (snippets)"
    aliases = ["scenic-snippet"]
    filenames = []
    alias_filenames = []
    mimetypes = []

    tokens = {
        "expr": [
            (r"\{([\w.]*)\}", bygroups(Generic.Emph)),
            inherit,
        ],
    }

    def analyse_text(text):
        return 0.0


class ScenicSpecifierLexer(ScenicSnippetLexer):
    """Further variant lexer for specifiers at the top level."""

    name = "Scenic (specifier snippets)"
    aliases = ["scenic-specifier"]

    tokens = {
        "root": [
            include("specifier-start"),
            (r"with", Keyword.Specifier),  # special case for incomplete 'with'
        ],
    }


class ScenicPropertyLexer(RegexLexer):
    """Silly lexer to color property names consistently with the real lexer."""

    name = "Scenic (properties)"
    aliases = ["scenic-property"]

    tokens = {
        "root": [
            (ScenicLexer.uni_name, Name.Variable.Instance),
        ],
    }

    def analyse_text(text):
        return 0.0


class ScenicGrammarLexer(RegexLexer):
    """Lexer for the grammar notation used in the Scenic docs."""

    name = "Scenic Grammar"
    aliases = ["scenic-grammar"]

    tokens = {
        "root": [
            (
                r"(<)([ -;=?-~]+)(>)",
                bygroups(Punctuation, Name.Class.Grammar, Punctuation),
            ),
            (r"[\[\]|/*+]", Operator),
            (r"\#.*\n", Comment.Single),
            (r"[^<>\[\]|*+#]+", Text),  # for performance
            (r".", Text),
        ]
    }

    def analyse_text(text):
        return 0.0


class ScenicStyle(Style):
    """A style providing specialized highlighting for the Scenic language.

    The color scheme is a loose hybrid of that used in the Scenic papers and
    the 'Mariana' color scheme from Sublime Text. The chosen colors all have a
    contrast ratio of at least 4.5:1 against the background color, per the
    W3C's Web Content Accessibility Guidelines.
    """

    background_color = "#f8f8f8"

    # Inherit styles from the default style for any token types we don't use
    styles = DefaultStyle.styles

    styles.update(
        {
            Punctuation: "#656E81",
            Comment: "italic #707274",
            Keyword: "#6852BB",
            Keyword.Specifier: "#146666",
            Keyword.Constant: "italic #4042B3",
            Operator: "#C7161C",
            Operator.Word: "#992024",
            Operator.Member: "#656E81",
            Name: "#404040",
            Name.Builtin: "italic #1773cf",
            Name.Builtin.Pseudo: "#A66214",
            Name.Builtin.Instance: "bold #3C8031",
            Name.Exception: "italic #CB3F38",
            Name.Variable.Parameter: "#A66214",
            Name.Namespace: "#0077A6",
            Name.Function: "#0077A6",
            Name.Function.Magic: "italic #1773cf",
            Name.Other.FunctionCall: "#1773cf",
            Name.Class: "bold #0077A6",
            Name.Class.Grammar: "nobold italic",
            Name.Class.Instance: "#3C8031",
            Name.Variable: "#B35824",
            Name.Variable.Parameter.Magic: "italic",
            Name.Variable.Magic: "italic #B32483",
            String: "#34822B",
            String.Affix: "#6852BB",
            String.Interpol: "bold #B32483",
            String.Escape: "#B32483",
            Number: "#4042B3",
            Error: "#FFFFFF bg:#FF0000",
        }
    )
