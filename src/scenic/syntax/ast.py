import ast

class AST(ast.AST):
    "Scenic AST base class"
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)

# special statements
class EgoAssign(AST):
    def __init__(self, value: any, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]

# Instance Creation

class New(AST):
    def __init__(self, className: str, specifiers: list, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.className = className
        self.specifiers = specifiers
        self._fields = ["className", "specifiers"]

# Specifiers

class WithSpecifier(AST):
    def __init__(self, prop: str, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.prop = prop
        self.value = value
        self._fields = ["prop", "value"]
