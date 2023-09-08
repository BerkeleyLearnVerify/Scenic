"""Version of sphinx.pycode.ModuleAnalyzer supporting Scenic modules.

This code is mostly copied from ModuleAnalyzer and its helper classes,
with minor changes.
"""

from collections import OrderedDict
from token import DEDENT, INDENT, NAME, NEWLINE, NUMBER, OP, STRING
from tokenize import COMMENT, NL

from sphinx.errors import PycodeError
from sphinx.pycode import ModuleAnalyzer
from sphinx.pycode.parser import DefinitionFinder, Parser, VariableCommentPicker

from scenic.syntax.parser import parse_string


class ScenicModuleAnalyzer(ModuleAnalyzer):
    def analyze(self):
        if self._analyzed:
            return
        if not self.srcname.endswith((".scenic", ".sc")):
            super().analyze()
            return

        try:
            parser = ScenicParser(self.code)
            parser.parse()

            self.attr_docs = OrderedDict()
            for scope, comment in parser.comments.items():
                if comment:
                    self.attr_docs[scope] = comment.splitlines() + [""]
                else:
                    self.attr_docs[scope] = [""]

            self.annotations = parser.annotations
            self.finals = parser.finals
            self.overloads = parser.overloads
            self.tags = parser.definitions
            self.tagorder = parser.deforders
            self._analyzed = True
        except Exception as exc:
            raise PycodeError("parsing %r failed: %r" % (self.srcname, exc)) from exc


class ScenicDefinitionFinder(DefinitionFinder):
    def parse(self):
        """Parse the code to obtain location of definitions."""
        while True:
            token = self.fetch_token()
            if token is None:
                break
            elif token == COMMENT:
                pass
            elif token == [OP, "@"] and (
                self.previous is None or self.previous.match(NEWLINE, NL, INDENT, DEDENT)
            ):
                if self.decorator is None:
                    self.decorator = token
            elif token.match([NAME, "class"]):
                self.parse_definition("class")
            elif token.match([NAME, "def"]):
                self.parse_definition("def")
            elif token.match([NAME, "behavior"]):
                self.parse_definition("behavior")
            elif token.match([NAME, "monitor"]):
                self.parse_definition("monitor")
            elif token.match([NAME, "scenario"]):
                self.parse_definition("scenario")
            elif token == INDENT:
                self.indents.append(("other", None, None))
            elif token == DEDENT:
                self.finalize_block()


class ScenicParser(Parser):
    def parse_comments(self):
        """Parse the code and pick up comments."""
        tree = parse_string(self.code, "exec")
        picker = VariableCommentPicker(self.code.splitlines(True), self.encoding)
        picker.visit(tree)
        self.annotations = picker.annotations
        self.comments = picker.comments
        self.deforders = picker.deforders
        self.finals = picker.finals
        self.overloads = picker.overloads

    def parse_definition(self) -> None:
        """Parse the location of definitions from the code."""
        parser = ScenicDefinitionFinder(self.code.splitlines(True))
        parser.parse()
        self.definitions = parser.definitions
