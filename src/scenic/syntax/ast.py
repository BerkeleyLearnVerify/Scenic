import ast
import typing
from typing import Optional, Union


class AST(ast.AST):
    "Scenic AST base class"

    _attributes = ("lineno", "col_offset", "end_lineno", "end_col_offset")

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
        body: typing.List[ast.stmt],
        interrupt_when_handlers: typing.List["InterruptWhenHandler"],
        except_handlers: typing.List[ast.ExceptHandler],
        orelse: typing.List[ast.stmt],
        finalbody: typing.List[ast.AST],
        *args: any,
        **kwargs: any,
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
        self, cond: ast.AST, body: typing.List[ast.AST], *args: any, **kwargs: any
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
        self, target: Union["Ego", "Workspace"], value: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.value = value
        self._fields = ["target", "value"]


class Ego(AST):
    "`ego` tracked assign target"

    functionName = "ego"


class Workspace(AST):
    ":term:`workspace` tracked assign target"

    functionName = "workspace"


class InitialScenario(AST):
    pass


class PropertyDef(AST):
    __match_args__ = ("property", "attributes", "value")

    def __init__(
        self,
        property: str,
        attributes: typing.List[Union["Additive", "Dynamic", "Final"]],
        value=ast.AST,
        *args: any,
        **kwargs: any,
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


class Final(AST):
    keyword = "final"

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
        header: typing.List[Union["Precondition", "Invariant"]],
        body: typing.List[any],
        *_args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*_args, **kwargs)
        self.name = name
        self.args = args
        self.docstring = docstring
        self.header = header
        self.body = body
        self._fields = ["name", "args", "docstring", "header", "body"]


class MonitorDef(AST):
    __match_args__ = ("name", "args", "docstring", "body")

    def __init__(
        self,
        name: str,
        args: ast.arguments,
        docstring: Optional[str],
        body: typing.List[ast.AST],
        *_args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*_args, **kwargs)
        self.name = name
        self.args = args
        self.docstring = docstring
        self.body = body
        self._fields = ["name", "args", "docstring", "body"]


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


# modular scenarios


class ScenarioDef(AST):
    __match_args__ = ("name", "args", "docstring", "header", "setup", "compose")

    def __init__(
        self,
        name: str,
        args: ast.arguments,
        docstring: Optional[str],
        header: Optional[typing.List[Union[Precondition, Invariant]]],
        setup: typing.List[ast.AST],
        compose: typing.List[ast.AST],
        *_args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*_args, **kwargs)
        self.name = name
        self.args = args
        self.docstring = docstring
        self.header = header
        self.setup = setup
        self.compose = compose
        self._fields = ["name", "args", "docstring", "header", "setup", "compose"]


# simple statements


class Model(AST):
    __match_args__ = ("name",)

    def __init__(self, name: str, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.name = name
        self._fields = ["name"]


class Param(AST):
    ":keyword:`param` statements"

    __match_args__ = ("elts",)

    def __init__(self, elts: typing.List["parameter"], *args: any, **kwargs: any) -> None:
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
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.prob = prob
        self.name = name
        self._fields = ["cond", "prob", "name"]


class RequireMonitor(AST):
    __match_args__ = ("monitor", "name")

    def __init__(
        self, monitor: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.monitor = monitor
        self.name = name
        self._fields = ["monitor", "name"]


class Always(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]

    def __reduce__(self):
        return (
            type(self),
            (self.value,),
            {
                "lineno": self.lineno,
                "end_lineno": self.end_lineno,
                "col_offset": self.col_offset,
                "end_col_offset": self.end_col_offset,
            },
        )


class Eventually(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]

    def __reduce__(self):
        return (
            type(self),
            (self.value,),
            {
                "lineno": self.lineno,
                "end_lineno": self.end_lineno,
                "col_offset": self.col_offset,
                "end_col_offset": self.end_col_offset,
            },
        )


class Next(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.value = value
        self._fields = ["value"]

    def __reduce__(self):
        return (
            type(self),
            (self.value,),
            {
                "lineno": self.lineno,
                "end_lineno": self.end_lineno,
                "col_offset": self.col_offset,
                "end_col_offset": self.end_col_offset,
            },
        )


class Record(AST):
    __match_args__ = ("value", "name")

    def __init__(
        self, value: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self.name = name
        self._fields = ["value", "name"]


class RecordInitial(AST):
    __match_args__ = ("value", "name")

    def __init__(
        self, value: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self.name = name
        self._fields = ["value", "name"]


class RecordFinal(AST):
    __match_args__ = ("value", "name")

    def __init__(
        self, value: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.value = value
        self.name = name
        self._fields = ["value", "name"]


class Mutate(AST):
    __match_args__ = ("elts", "scale")

    def __init__(
        self,
        elts: typing.List[ast.Name],
        scale: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self.scale = scale
        self._fields = ["elts", "scale"]


class Override(AST):
    __match_args__ = ("target", "specifiers")

    def __init__(
        self, target: ast.AST, specifiers: typing.List[ast.AST], *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.specifiers = specifiers
        self._fields = ["target", "specifiers"]


class Abort(AST):
    pass


class Take(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: typing.List[ast.AST], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class Wait(AST):
    pass


class Terminate(AST):
    pass


class TerminateSimulation(AST):
    pass


class TerminateSimulationWhen(AST):
    __match_args__ = (
        "cond",
        "name",
    )

    def __init__(
        self, cond: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.name = name
        self._fields = ["cond", "name"]


class TerminateWhen(AST):
    __match_args__ = (
        "cond",
        "name",
    )

    def __init__(
        self, cond: ast.AST, name: Optional[str] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.cond = cond
        self.name = name
        self._fields = ["cond", "name"]


class TerminateAfter(AST):
    __match_args__ = ("duration",)

    def __init__(
        self, duration: Union["Seconds", "Steps"], *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.duration = duration
        self._fields = ["duration"]


class DoFor(AST):
    __match_args__ = ("elts", "duration")

    def __init__(
        self,
        elts: typing.List[ast.AST],
        duration: Union["Seconds", "Steps"],
        *args: any,
        **kwargs: any,
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
        self, elts: typing.List[ast.AST], cond: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self.cond = cond
        self._fields = ["elts", "cond"]


class DoChoose(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: typing.List[ast.AST], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class DoShuffle(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: typing.List[ast.AST], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class Do(AST):
    __match_args__ = ("elts",)

    def __init__(self, elts: typing.List[ast.AST], *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.elts = elts
        self._fields = ["elts"]


class Simulator(AST):
    __match_args__ = ("value",)

    def __init__(self, value: ast.AST, *args: any, **kwargs: any) -> None:
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
        **kwargs: any,
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


class Above(AST):
    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Below(AST):
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
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self.offset = offset
        self.base = base
        self._fields = ["position", "offset", "base"]


class VisibleSpecifier(AST):
    __match_args__ = ("base",)

    def __init__(self, base: Optional[ast.AST] = None, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.base = base
        self._fields = ["base"]


class NotVisibleSpecifier(AST):
    __match_args__ = ("base",)

    def __init__(self, base: Optional[ast.AST] = None, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.base = base
        self._fields = ["base"]


class InSpecifier(AST):
    __match_args__ = ("region",)

    def __init__(self, region: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self._fields = ["region"]


class OnSpecifier(AST):
    __match_args__ = ("region",)

    def __init__(self, region: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self._fields = ["region"]


class ContainedInSpecifier(AST):
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
        **kwargs: any,
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


class FacingAwayFromSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]


class FacingDirectlyTowardSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]


class FacingDirectlyAwayFromSpecifier(AST):
    __match_args__ = ("position",)

    def __init__(self, position: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self._fields = ["position"]


class ApparentlyFacingSpecifier(AST):
    __match_args__ = ("heading", "base")

    def __init__(
        self, heading: ast.AST, base: Optional[ast.AST] = None, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.heading = heading
        self.base = base
        self._fields = ["heading", "base"]


# Operators


class ImpliesOp(AST):
    __match_args__ = ("hypothesis", "conclusion")

    def __init__(
        self, hypothesis: ast.AST, conclusion: ast.AST, *args: any, **kwargs: any
    ) -> None:
        super().__init__(*args, **kwargs)
        self.hypothesis = hypothesis
        self.conclusion = conclusion
        self._fields = ["hypothesis", "conclusion"]

    def __reduce__(self):
        return (
            type(self),
            (self.hypothesis, self.conclusion),
            {
                "lineno": self.lineno,
                "end_lineno": self.end_lineno,
                "col_offset": self.col_offset,
                "end_col_offset": self.end_col_offset,
            },
        )


class UntilOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]

    def __reduce__(self):
        return (
            type(self),
            (self.left, self.right),
            {
                "lineno": self.lineno,
                "end_lineno": self.end_lineno,
                "col_offset": self.col_offset,
                "end_col_offset": self.end_col_offset,
            },
        )


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
        **kwargs: any,
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
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class AltitudeFromOp(AST):
    __match_args__ = ("target", "base")

    def __init__(
        self,
        target: Optional[ast.AST] = None,
        base: Optional[ast.AST] = None,
        *args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.target = target
        self.base = base
        self._fields = ["target", "base"]


class FollowOp(AST):
    __match_args__ = ("target", "base", "distance")

    def __init__(
        self, target: ast.AST, base: ast.AST, distance: ast.AST, *args: any, **kwargs: any
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


class VisibleFromOp(AST):
    __match_args__ = ("region", "base")

    def __init__(self, region: ast.AST, base: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self.base = base
        self._fields = ["region", "base"]


class NotVisibleFromOp(AST):
    __match_args__ = ("region", "base")

    def __init__(self, region: ast.AST, base: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.region = region
        self.base = base
        self._fields = ["region", "base"]


class PositionOfOp(AST):
    __match_args__ = ("position", "target")

    def __init__(
        self,
        position: Union[
            "Front",
            "Back",
            "Left",
            "Right",
            "Top",
            "Bottom",
            "FrontLeft",
            "FrontRight",
            "BackLeft",
            "BackRight",
            "TopFrontLeft",
            "TopFrontRight",
            "TopBackLeft",
            "TopBackRight",
            "BottomFrontLeft",
            "BottomFrontRight",
            "BottomBackLeft",
            "BottomBackRight",
        ],
        target: ast.AST,
        *args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.position = position
        self.target = target
        self._fields = ["position", "target"]


class Front(AST):
    "Represents position of :scenic:`front of` operator"

    functionName = "Front"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Back(AST):
    "Represents position of :scenic:`back of` operator"

    functionName = "Back"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Left(AST):
    "Represents position of :scenic:`left of` operator"

    functionName = "Left"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Right(AST):
    "Represents position of :scenic:`right of` operator"

    functionName = "Right"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Top(AST):
    "Represents position of :scenic:`top of` operator"

    functionName = "Top"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class Bottom(AST):
    "Represents position of :scenic:`bottom of` operator"

    functionName = "Bottom"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontLeft(AST):
    "Represents position of :scenic:`front left of` operator"

    functionName = "FrontLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class FrontRight(AST):
    "Represents position of :scenic:`front right of` operator"

    functionName = "FrontRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackLeft(AST):
    "Represents position of :scenic:`back left of` operator"

    functionName = "BackLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BackRight(AST):
    "Represents position of :scenic:`back right of` operator"

    functionName = "BackRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class TopFrontLeft(AST):
    "Represents position of :scenic:`top front left of` operator"

    functionName = "TopFrontLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class TopFrontRight(AST):
    "Represents position of :scenic:`top front right of` operator"

    functionName = "TopFrontRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class TopBackLeft(AST):
    "Represents position of :scenic:`top back left of` operator"

    functionName = "TopBackLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class TopBackRight(AST):
    "Represents position of :scenic:`top back right of` operator"

    functionName = "TopBackRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BottomFrontLeft(AST):
    "Represents position of :scenic:`bottom front left of` operator"

    functionName = "BottomFrontLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BottomFrontRight(AST):
    "Represents position of :scenic:`bottom front right of` operator"

    functionName = "BottomFrontRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BottomBackLeft(AST):
    "Represents position of :scenic:`bottom back left of` operator"

    functionName = "BottomBackLeft"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class BottomBackRight(AST):
    "Represents position of :scenic:`bottom back right of` operator"

    functionName = "BottomBackRight"

    def __init__(self, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)


class DegOp(AST):
    __match_args__ = ("operand",)

    def __init__(self, operand: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.operand = operand
        self._fields = ["operand"]


class VectorOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]


class FieldAtOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]


class RelativeToOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]


class OffsetAlongOp(AST):
    __match_args__ = ("base", "direction", "offset")

    def __init__(
        self,
        base: ast.AST,
        direction: ast.AST,
        offset: ast.AST,
        *args: any,
        **kwargs: any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.base = base
        self.direction = direction
        self.offset = offset
        self._fields = ["base", "direction", "offset"]


class CanSeeOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]


class IntersectsOp(AST):
    __match_args__ = ("left", "right")

    def __init__(self, left: ast.AST, right: ast.AST, *args: any, **kwargs: any) -> None:
        super().__init__(*args, **kwargs)
        self.left = left
        self.right = right
        self._fields = ["left", "right"]
