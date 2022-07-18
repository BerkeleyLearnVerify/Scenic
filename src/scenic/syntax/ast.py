import ast
from typing import Optional, Union


class AST(ast.AST):
    "Scenic AST base class"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


# special statements
class TrackedAssign(AST):
    __match_args__ = (
        "target",
        "value",
    )

    def __init__(
        self,
        target: Union["Ego", "Workspace"],
        value: ast.AST,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.value = value
        self._fields = ["target", "value"]


class Ego(AST):
    "`ego` tracked assign target"
    functionName = "ego"


class Workspace(AST):
    "`workspace` tracked assign target"
    functionName = "workspace"


# simple statements


class Model(AST):
    __match_args__ = ("name",)

    def __init__(self, name: str, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.name = name
        self._fields = ["name"]


class Param(AST):
    "`param identifier = value, …` statements"

    __match_args__ = ("elts",)

    def __init__(self, elts: list["parameter"], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class parameter(AST):
    "represents a parameter that is defined with `param` statements"
    __match_args__ = ("identifier", "value")

    def __init__(
        self, identifier: str, value: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.identifier = identifier
        self.value = value
        self._fields = ["identifier" ,"value"]


class Require(AST):
    __match_args__ = ("cond", "prob", "name")

    def __init__(
        self,
        cond: ast.AST,
        prob: Optional[float] = None,
        name: Optional[str] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.prob = prob
        self.name = name
        self._fields = ["cond", "prob", "name"]


class Mutate(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: list[ast.Name], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


# Instance Creation


class New(AST):
    __match_args__ = ("className", "specifiers")

    def __init__(
        self, className: str, specifiers: list, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.className = className
        self.specifiers = specifiers if specifiers is not None else []
        self._fields = ["className", "specifiers"]


# Specifiers


class WithSpecifier(AST):
    __match_args__ = ("prop", "value")

    def __init__(self, prop: str, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.prop = prop
        self.value = value
        self._fields = ["prob", "value"]


class AtSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]


class OffsetBySpecifier(AST):
    __match_args__ = ("offset",)

    def __init__(self, offset: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.offset = offset
        self._fields = ["offset"]


class OffsetAlongSpecifier(AST):
    __match_args__ = ("direction", "offset")

    def __init__(
        self, direction: ast.AST, offset: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.direction = direction
        self.offset = offset
        self._fields = ["direction", "offset"]


class DirectionOfSpecifier(AST):
    __match_args__ = ("direction", "position", "distance")

    def __init__(
        self,
        direction: Union["LeftOf", "RightOf", "AheadOf", "Behind"],
        position: ast.AST,
        distance: Optional[ast.AST],
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.direction = direction
        self.position = position
        self.distance = distance
        self._fields = ["direction", "position", "distance"]


class LeftOf(AST):
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class RightOf(AST):
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class AheadOf(AST):
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Behind(AST):
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BeyondSpecifier(AST):
    __match_args__ = ("position", "offset", "base")

    def __init__(
        self,
        position: ast.AST,
        offset: ast.AST,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self.offset = offset
        self.base = base
        self._fields = ["position", "offset", "base"]


class VisibleSpecifier(AST):
    __match_args__ = ("base",)

    def __init__(
        self, base: Optional[ast.AST] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.base = base
        self._fields = ["base"]


class InSpecifier(AST):
    __match_args__ = ("region",)

    def __init__(self, region: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self._fields = ["region"]


class FollowingSpecifier(AST):
    __match_args__ = ("field", "distance", "base")

    def __init__(
        self,
        field: ast.AST,
        distance: ast.AST,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.field = field
        self.distance = distance
        self.base = base
        self._fields = ["field", "distance", "base"]


class FacingSpecifier(AST):
    __match_args__ = ("heading",)

    def __init__(self, heading: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.heading = heading
        self._fields = ["heading"]


class FacingTowardSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]


class ApparentlyFacingSpecifier(AST):
    __match_args__ = ("heading", "base")

    def __init__(
        self,
        heading: ast.AST,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.heading = heading
        self.base = base
        self._fields = ["heading", "base"]
