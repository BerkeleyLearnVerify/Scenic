import ast
from typing import Optional, Union


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
