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

class New(AST):
    def __init__(self, className: str, specifiers: list, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.className = className
        self.specifiers = specifiers
