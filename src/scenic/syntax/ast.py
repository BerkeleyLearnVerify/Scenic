import ast


class AST(ast.AST):
    "Scenic AST base class"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


# special statements
class EgoAssign(AST):
    __match_args__ = ("value",)

    def __init__(self, value: any, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]


# Instance Creation


class New(AST):
    __match_args__ = ("className", "specifiers")

    def __init__(
        self, className: str, specifiers: list, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.className = className
        self.specifiers = specifiers
        self._fields = ["className", "specifiers"]


# Specifiers


class WithSpecifier(AST):
    __match_args__ = ("prop", "value")

    def __init__(self, prop: str, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.prop = prop
        self.value = value
        self._fields = ["prop", "value"]


class AtSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]
