import ast
from typing import Optional, Union


class AST(ast.AST):
    "Scenic AST base class"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


# special statements


class TryInterrupt(AST):
    """Scenic AST node that represents try-interrupt statements"""

    __match_args__ = (
        "body",
        "interrupt_when_handlers",
        "except_handlers",
        "orelse",
        "finalbody",
    )

    def __init__(
        self,
        body: list[ast.stmt],
        interrupt_when_handlers: list["InterruptWhenHandler"],
        except_handlers: list[ast.ExceptHandler],
        orelse: list[ast.stmt],
        finalbody: list[ast.AST],
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.body = body
        self.interrupt_when_handlers = interrupt_when_handlers
        self.except_handlers = except_handlers
        self.orelse = orelse
        self.finalbody = finalbody
        self._fields = [
            "body",
            "interrupt_when_handlers",
            "except_handlers",
            "orelse",
            "finalbody",
        ]
        self._attributes = []


class InterruptWhenHandler(AST):
    __match_args__ = ("cond", "body")

    def __init__(
        self, cond: ast.AST, body: list[ast.AST], *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.body = body
        self._fields = ["cond", "body"]


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


class PropertyDef(AST):
    __match_args__ = ("property", "attributes", "value")

    def __init__(
        self,
        property: str,
        attributes: list[Union["Additive", "Dynamic"]],
        value=ast.AST,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.property = property
        self.attributes = attributes
        self.value = value
        self._fields = ["property", "attributes", "value"]


class Additive(AST):
    keyword = "additive"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Dynamic(AST):
    keyword = "dynamic"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


# behavior / monitor


class BehaviorDef(AST):
    __match_args__ = ("name", "args", "docstring", "header", "body")

    def __init__(
        self,
        name: str,
        args: ast.arguments,
        docstring: Optional[str],
        header: list[Union["Precondition", "Invariant"]],
        body: list[any],
        *_args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*_args, **kwargs)
        self.name = name
        self.args = args
        self.docstring = docstring
        self.header = header
        self.body = body
        self._fields = ["name", "args", "docstring", "header", "body"]


class MonitorDef(AST):
    __match_args__ = ("name", "docstring", "body")

    def __init__(
        self,
        name: str,
        docstring: Optional[str],
        body: list[ast.AST],
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.name = name
        self.docstring = docstring
        self.body = body
        self._fields = ["name", "docstring", "body"]


class Precondition(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]


class Invariant(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]


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
        self._fields = ["identifier", "value"]


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


class RequireAlways(AST):
    __match_args__ = ("cond", "name")

    def __init__(
        self, cond: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.name = name
        self._fields = ["cond", "name"]


class RequireEventually(AST):
    __match_args__ = ("cond", "name")

    def __init__(
        self, cond: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.name = name
        self._fields = ["cond", "name"]


class Mutate(AST):
    __match_args__ = ("elts", "scale")

    def __init__(
        self,
        elts: list[ast.Name],
        scale: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self.scale = scale
        self._fields = ["elts", "scale"]


class Abort(AST):
    pass


class Take(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: list[ast.AST], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class Wait(AST):
    pass


class Terminate(AST):
    pass


class DoFor(AST):
    __match_args__ = ("elts", "duration")

    def __init__(
        self,
        elts: list[ast.AST],
        duration: Union["Seconds", "Steps"],
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self.duration = duration
        self._fields = ["elts", "duration"]


class Seconds(AST):
    __match_args__ = ("value",)
    unitStr = "seconds"

    def __init__(self, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]


class Steps(AST):
    __match_args__ = ("value",)
    unitStr = "steps"

    def __init__(self, value: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]


class DoUntil(AST):
    __match_args__ = ("elts", "cond")

    def __init__(
        self, elts: list[ast.AST], cond: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self.cond = cond
        self._fields = ["value", "cond"]


class Do(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: list[ast.AST], *args: any, **kwargs: any) -> None:
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


class NotVisibleSpecifier(AST):
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
        self,
        # because `to` and `from` are symmetric, the first operand will be `target` and the second will be `base`
        target: ast.AST,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any
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
    functionName = "Front"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Back(AST):
    "Represents position of `back of` operator"
    functionName = "Back"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Left(AST):
    "Represents position of `left of` operator"
    functionName = "Left"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Right(AST):
    "Represents position of `right of` operator"
    functionName = "Right"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontLeft(AST):
    "Represents position of `front left of` operator"
    functionName = "FrontLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontRight(AST):
    "Represents position of `front right of` operator"
    functionName = "FrontRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackLeft(AST):
    "Represents position of `back left of` operator"
    functionName = "BackLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackRight(AST):
    "Represents position of `back right of` operator"
    functionName = "BackRight"

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
    __match_args__ = ("base", "direction", "offset")

    def __init__(
        self,
        base: ast.AST,
        direction: ast.AST,
        offset: ast.AST,
        *args: any,
        **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.base = base
        self.direction = direction
        self.offset = offset


class CanSeeOp(AST):
    __match_args__ = ("left", "right")

    def __init__(
        self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
