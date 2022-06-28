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


# Operators
class RelativePositionOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self, target: ast.AST, base: ast.AST = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class RelativeHeadingOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self, target: ast.AST, base: ast.AST = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class ApparentHeadingOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self, target: ast.AST, base: ast.AST = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class DistanceFromOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self, target: ast.AST, base: Optional[ast.AST] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class DistancePastOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self, target: ast.AST, base: Optional[ast.AST] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class AngleFromOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self,
        target: Optional[ast.AST] = None,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class FollowOp(AST):
    __match_args__ = ("target", "base", "distance")

    def __init__(
        self,
        target: ast.AST,
        base: ast.AST,
        distance: ast.AST,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self.distance = distance
        self._fields = ["target", "base", "distance"]


class VisibleOp(AST):
    __match_args__ = ("region",)

    def __init__(self, region: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self._fields = ["region"]


class NotVisibleOp(AST):
    __match_args__ = ("region",)

    def __init__(self, region: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self._fields = ["region"]


class PositionOfOp(AST):
    __match_args__ = ("position", "target")

    def __init__(
        self,
        position: Union[
            "Front",
            "Back",
            "Left",
            "Right",
            "FrontLeft",
            "FrontRight",
            "BackLeft",
            "BackRight",
        ],
        target: ast.AST,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self.target = target


class Front(AST):
    "Represents position of `front of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Back(AST):
    "Represents position of `back of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Left(AST):
    "Represents position of `left of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Right(AST):
    "Represents position of `right of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontLeft(AST):
    "Represents position of `front left of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontRight(AST):
    "Represents position of `front right of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackLeft(AST):
    "Represents position of `back left of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackRight(AST):
    "Represents position of `back right of` operator"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class DegOp(AST):
    __match_args__ = ("operand",)

    def __init__(self, operand: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.operand = operand


class VectorOp(AST):
    __match_args__ = ("left", "right")

    def __init__(
        self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]


class FieldAtOp(AST):
    __match_args__ = ("left", "right")

    def __init__(
        self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right


class RelativeToOp(AST):
    __match_args__ = ("left", "right")

    def __init__(
        self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right


class OffsetAlongOp(AST):
    __match_args__ = ("left", "middle", "right")

    def __init__(
        self, left: ast.AST, middle: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.middle = middle
        self.right = right


class CanSeeOp(AST):
    __match_args__ = ("left", "right")

    def __init__(
        self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
