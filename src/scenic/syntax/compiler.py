import ast
import itertools
from typing import Optional, Tuple, List, Union

import scenic.syntax.ast as s

# exposed functions


def compileScenicAST(
    scenicAST: ast.AST,
) -> Tuple[Union[ast.AST, list[ast.AST]], List[ast.AST]]:
    """Compiles Scenic AST to Python AST"""
    compiler = ScenicToPythonTransformer()
    tree = compiler.visit(scenicAST)
    if isinstance(tree, list):
        node = [ast.fix_missing_locations(n) for n in tree]
    else:
        node = ast.fix_missing_locations(tree)
    return node, compiler.requirements


# constants

temporaryName = "_Scenic_temporary_name"
behaviorArgName = "_Scenic_current_behavior"
checkPreconditionsName = "checkPreconditions"
checkInvariantsName = "checkInvariants"
interruptPrefix = "_Scenic_interrupt"

abortFlag = ast.Attribute(ast.Name("BlockConclusion", ast.Load()), "ABORT", ast.Load())
breakFlag = ast.Attribute(ast.Name("BlockConclusion", ast.Load()), "BREAK", ast.Load())
continueFlag = ast.Attribute(
    ast.Name("BlockConclusion", ast.Load()), "CONTINUE", ast.Load()
)
returnFlag = ast.Attribute(
    ast.Name("BlockConclusion", ast.Load()), "RETURN", ast.Load()
)
finishedFlag = ast.Attribute(
    ast.Name("BlockConclusion", ast.Load()), "FINISHED", ast.Load()
)

# shorthands for convenience

loadCtx = ast.Load()
ego = ast.Name("ego")

noArgs = ast.arguments(
    posonlyargs=[],
    args=[],
    vararg=None,
    kwonlyargs=[],
    kw_defaults=[],
    kwarg=None,
    defaults=[],
)
selfArg = ast.arguments(
    posonlyargs=[],
    args=[ast.arg(arg="self", annotation=None)],
    vararg=None,
    kwonlyargs=[],
    kw_defaults=[],
    kwarg=None,
    defaults=[],
)
initialBehaviorArgs = [
    ast.arg(arg=behaviorArgName, annotation=None),
    ast.arg(arg="self", annotation=None),
]
onlyBehaviorArgs = ast.arguments(
    posonlyargs=[],
    args=initialBehaviorArgs,
    vararg=None,
    kwonlyargs=[],
    kw_defaults=[],
    kwarg=None,
    defaults=[],
)

# helpers


class AttributeFinder(ast.NodeVisitor):
    """Utility class for finding all referenced attributes of a given name."""

    @staticmethod
    def find(target, node):
        af = AttributeFinder(target)
        af.visit(node)
        return af.attributes

    def __init__(self, target):
        super().__init__()
        self.target = target
        self.attributes = set()

    def visit_Attribute(self, node):
        val = node.value
        if isinstance(val, ast.Name) and val.id == self.target:
            self.attributes.add(node.attr)
        self.visit(val)


class LocalFinder(ast.NodeVisitor):
    """Utility class for finding all local variables of a code block."""

    @staticmethod
    def findIn(block):
        lf = LocalFinder()
        for statement in block:
            lf.visit(statement)
        names = lf.names
        return names - lf.globals - lf.nonlocals

    def __init__(self):
        self.names = set()
        self.globals = set()
        self.nonlocals = set()

    def visit_Global(self, node):
        self.globals.update(node.names)

    def visit_Nonlocal(self, node):
        self.nonlocals.update(node.names)

    def visit_FunctionDef(self, node):
        self.names.add(node.name)
        self.visit(node.args)
        for decorator in node.decorator_list:
            self.visit(decorator)
        if node.returns is not None:
            self.visit(node.returns)
        # do not visit body; it's another block

    def visit_Lambda(self, node):
        self.visit(node.args)
        # do not visit body; it's another block

    def visit_ClassDef(self, node):
        self.names.add(node.name)
        for child in itertools.chain(node.bases, node.keywords, node.decorator_list):
            self.visit(child)
        # do not visit body; it's another block

    def visit_Import(self, node):
        for alias in node.names:
            bound = alias.asname
            if bound is None:
                bound = alias.name
            self.names.add(bound)

    def visit_ImportFrom(self, node):
        self.visit_Import(node)

    def visit_Name(self, node):
        if isinstance(node.ctx, (ast.Store, ast.Del)):
            self.names.add(node.id)

    def visit_ExceptHandler(self, node):
        if node.name is not None:
            self.names.add(node.name)
        self.generic_visit(node)


def unquote(s: str) -> str:
    if (s[:3] == s[-3:]) and s.startswith(("'''", '"""')):
        return s[3:-3]
    if (s[0] == s[-1]) and s.startswith(("'", '"')):
        return s[1:-1]
    return s


# transformer


class ScenicToPythonTransformer(ast.NodeTransformer):
    def __init__(self) -> None:
        super().__init__()
        self.requirements = []

        self.inBehavior: bool = False
        "True if the transformer is processing behavior body"

        self.behaviorLocals: set = set()
        "Set of variable names on the local scope of the behavior"

        self.inCompose: bool = False
        "True if the transformer is processing scenario"

        self.inTryInterrupt = False
        self.inInterruptBlock = False
        self.inLoop = False
        self.usedBreak = False
        self.usedContinue = False

    def generic_visit(self, node):
        if isinstance(node, s.AST):
            raise Exception(
                f'Scenic AST node "{node.__class__.__name__}" needs visitor in compiler'
            )
        return super().generic_visit(node)

    # add support for list of nodes
    def visit(self, node: ast.AST | list[ast.AST]):
        if isinstance(node, ast.AST):
            return super().visit(node)
        elif isinstance(node, list):
            newStatements = []
            for statement in node:
                newStatement = self.visit(statement)
                if isinstance(newStatement, ast.AST):
                    newStatements.append(newStatement)
                else:
                    newStatements.extend(newStatement)
            return newStatements
        else:
            raise RuntimeError(f"unknown object {node} encountered during compilation")

    # helper functions
    def _register_requirement_syntax(self, syntax: ast.AST) -> int:
        self.requirements.append(syntax)
        return len(self.requirements) - 1

    def visit_Name(self, node: ast.Name) -> any:
        from scenic.syntax.translator import builtinNames, trackedNames, behaviorArgName

        if node.id in builtinNames:
            if not isinstance(node.ctx, ast.Load):
                raise SyntaxError(f'unexpected keyword "f{node.id}"')
        elif node.id in trackedNames:
            if not isinstance(node.ctx, ast.Load):
                raise SyntaxError(f'only simple assignments to "{node.id}" are allowed')
            node = ast.copy_location(ast.Call(ast.Name(node.id, loadCtx), [], []), node)
        elif node.id in self.behaviorLocals:
            lookup = ast.Attribute(
                ast.Name(behaviorArgName, loadCtx), node.id, node.ctx
            )
            return ast.copy_location(lookup, node)
        return node

    # hook control flow nodes to set appropriate flags for try-interrupt

    def visit_For(self, node):
        old = self.inLoop
        self.inLoop = True
        newNode = self.generic_visit(node)
        self.inLoop = old
        return newNode

    def visit_While(self, node):
        old = self.inLoop
        self.inLoop = True
        newNode = self.generic_visit(node)
        self.inLoop = old
        return newNode

    def visit_FunctionDef(self, node):
        oldInLoop, oldInInterruptBlock = self.inLoop, self.inInterruptBlock
        self.inLoop, self.inInterruptBlock = False, False
        newNode = self.generic_visit(node)
        self.inLoop, self.inInterruptBlock = oldInLoop, oldInInterruptBlock
        return newNode

    def visit_Break(self, node):
        if self.inInterruptBlock and not self.inLoop:
            if not self.usedBreak:
                self.usedBreak = node
            newNode = ast.Return(breakFlag)
            return ast.copy_location(newNode, node)
        else:
            return self.generic_visit(node)

    def visit_Continue(self, node):
        if self.inInterruptBlock and not self.inLoop:
            if not self.usedContinue:
                self.usedContinue = node
            newNode = ast.Return(continueFlag)
            return ast.copy_location(newNode, node)
        else:
            return self.generic_visit(node)

    def visit_Return(self, node):
        if self.inInterruptBlock:
            value = ast.Constant(None) if node.value is None else node.value
            ret = ast.Return(ast.Call(returnFlag, [value], []))
            return ast.copy_location(ret, node)
        else:
            return self.generic_visit(node)

    # Special Case

    def visit_TryInterrupt(self, node: s.TryInterrupt):
        statements = []
        oldInTryInterrupt = self.inTryInterrupt

        self.inTryInterrupt = True

        oldInInterruptBlock, oldInLoop = self.inInterruptBlock, self.inLoop
        self.inInterruptBlock = True
        self.inLoop = False
        self.usedBreak = False
        self.usedContinue = False

        def makeInterruptBlock(name, body):
            newBody = self.visit(body)
            allLocals = LocalFinder.findIn(newBody)
            if allLocals:
                newBody.insert(0, ast.Nonlocal(list(allLocals)))
            newBody.append(ast.Return(finishedFlag))
            return ast.FunctionDef(name, onlyBehaviorArgs, newBody, [], None)

        bodyName = f"{interruptPrefix}_body"
        statements.append(makeInterruptBlock(bodyName, node.body))

        handlerNames, conditionNames = [], []
        for i, handler in enumerate(node.interrupt_when_handlers):
            condition = handler.cond
            block = handler.body

            handlerName = f"{interruptPrefix}_handler_{i}"
            handlerNames.append(handlerName)
            conditionName = f"{interruptPrefix}_condition_{i}"
            conditionNames.append(conditionName)

            statements.append(makeInterruptBlock(handlerName, block))
            self.inGuard = True
            checker = ast.Lambda(noArgs, self.visit(condition))
            self.inGuard = False
            defChecker = ast.Assign([ast.Name(conditionName, ast.Store())], checker)
            statements.append(defChecker)

        self.inInterruptBlock, self.inLoop = oldInInterruptBlock, oldInLoop
        self.inTryInterrupt = oldInTryInterrupt

        # Prepare tuples of interrupt conditions and handlers
        # (in order from high priority to low, so reversed relative to the syntax)
        conditions = ast.Tuple(
            [ast.Name(n, ast.Load()) for n in reversed(conditionNames)], ast.Load()
        )
        handlers = ast.Tuple(
            [ast.Name(n, ast.Load()) for n in reversed(handlerNames)], ast.Load()
        )

        # Construct code to execute the try-interrupt statement
        args = [
            ast.Name(behaviorArgName, ast.Load()),
            ast.Name("self", ast.Load()),
            ast.Name(bodyName, ast.Load()),
            conditions,
            handlers,
        ]
        callRuntime = ast.Call(ast.Name("runTryInterrupt", ast.Load()), args, [])
        runTI = ast.Assign(
            [ast.Name(temporaryName, ast.Store())], ast.YieldFrom(callRuntime)
        )
        statements.append(runTI)
        result = ast.Name(temporaryName, ast.Load())
        if self.usedBreak:
            test = ast.Compare(result, [ast.Is()], [breakFlag])
            brk = ast.copy_location(ast.Break(), self.usedBreak)
            statements.append(ast.If(test, [brk], []))
        if self.usedContinue:
            test = ast.Compare(result, [ast.Is()], [continueFlag])
            cnt = ast.copy_location(ast.Continue(), self.usedContinue)
            statements.append(ast.If(test, [cnt], []))
        test = ast.Compare(result, [ast.Is()], [returnFlag])
        retCheck = ast.If(
            test, [ast.Return(ast.Attribute(result, "return_value", ast.Load()))], []
        )
        statements.append(retCheck)

        # Construct overall try-except statement
        if node.except_handlers or node.finalbody:
            newTry = ast.Try(
                statements,
                [self.visit(handler) for handler in node.except_handlers],
                self.visit(node.orelse),
                self.visit(node.finalbody),
            )
            return ast.copy_location(newTry, node)
        else:
            return statements

    def visit_TrackedAssign(self, node: s.TrackedAssign):
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id=node.target.functionName, ctx=loadCtx),
                args=[self.visit(node.value)],
                keywords=[],
            )
        )

    def visit_ClassDef(self, node: ast.ClassDef) -> any:
        # use `Object` as base if none is specified
        if not node.bases:
            node.bases = [ast.Name("Object", loadCtx)]

        # extract all property definitions
        propertyDefs: list[s.PropertyDef] = []
        newBody = []
        for stmt in node.body:
            if isinstance(stmt, s.PropertyDef):
                propertyDefs.append(stmt)
            else:
                newBody.append(stmt)

        # create dictionary from property name (str) to default values
        propertyDict = {}
        for propertyDef in propertyDefs:
            if propertyDef.property in propertyDict:
                raise SyntaxError(f'duplicated property "{propertyDef.property}"')
            propertyDict[propertyDef.property] = propertyDef

        newBody.insert(
            0,
            ast.Assign(
                targets=[ast.Name(id="_scenic_properties", ctx=ast.Store())],
                value=ast.Dict(
                    keys=[ast.Constant(value=p) for p in propertyDict.keys()],
                    values=[
                        self.transformPropertyDef(v) for v in propertyDict.values()
                    ],
                ),
            ),
        )

        node.body = newBody
        return self.generic_visit(node)

    def transformPropertyDef(self, node: s.PropertyDef):
        properties = AttributeFinder.find("self", node.value)
        return ast.Call(
            func=ast.Name(id="PropertyDefault", ctx=ast.Load()),
            args=[
                ast.Set(elts=[ast.Constant(value=p) for p in properties]),
                ast.Set(
                    elts=[ast.Constant(value=attr.keyword) for attr in node.attributes]
                ),
                ast.Lambda(
                    args=selfArg,
                    body=self.visit(node.value),
                ),
            ],
            keywords=[],
        )

    def visit_PropertyDef(self, _: s.PropertyDef) -> any:
        assert False, "PropertyDef should be handled in `visit_ClassDef`"

    def visit_BehaviorDef(self, node: s.BehaviorDef):
        # TODO(shun): assert not in behavior or compose

        return self.makeBehaviorLikeDef(
            baseClassName="Behavior",
            name=node.name,
            args=node.args,
            docstring=node.docstring,
            header=node.header,
            body=node.body,
        )

    def visit_MonitorDef(self, node: s.MonitorDef):
        # TODO(shun): assert not in behavior or compose

        return self.makeBehaviorLikeDef(
            baseClassName="Monitor",
            name=node.name,
            args=ast.arguments(
                posonlyargs=[],
                args=[],
                vararg=None,
                kwonlyargs=[],
                kw_defaults=[],
                kwarg=None,
                defaults=[],
            ),
            docstring=node.docstring,
            header=[],
            body=node.body,
        )

    def visit_ScenarioDef(self, node: s.ScenarioDef):
        # TODO(shun): assert not in behavior or compose

        # Set up arguments for setup and compose blocks
        args: ast.arguments = self.visit(node.args)
        args.posonlyargs = initialBehaviorArgs + args.posonlyargs

        # Get preconditions and invariants
        preconditions, invariants = self.classifyPreconditionsAndInvariants(node.header)

        setup = node.setup
        compose = node.compose

        # Find all locals of the scenario, which will be shared amongst the various blocks
        allLocals = set()
        if node.compose:
            allLocals.update(LocalFinder.findIn(node.compose))
        if node.setup:
            allLocals.update(LocalFinder.findIn(node.setup))
        oldBL = self.behaviorLocals
        self.behaviorLocals = allLocals

        guardCheckers = self.makeGuardCheckers(args, preconditions, invariants)

        # Construct compose block
        self.inCompose = self.inBehavior = True
        guardCheckers = self.makeGuardCheckers(args, preconditions, invariants)
        if node.compose or preconditions or invariants:
            if compose:
                body = self.visit(node.compose)
            else:
                # generate no-op compose block to ensure invariants are checked
                wait = self.generateInvocation(node, ast.Constant(()))
                body = [ast.While(ast.Constant(True), wait, [])]
            compose = ast.FunctionDef("_compose", args, body, [], None)
        else:
            compose = ast.Assign(
                [ast.Name("_compose", ast.Store())], ast.Constant(None)
            )
        self.inCompose = self.inBehavior = False

        # Construct setup block
        if setup:
            setup = ast.FunctionDef("_setup", args, self.visit(node.setup), [], None)
        else:
            setup = ast.Assign([ast.Name("_setup", ast.Store())], ast.Constant(None))

        self.behaviorLocals = oldBL

        # Assemble scenario definition
        name = node.name
        saveLocals = ast.Assign(
            [ast.Name("_locals", ast.Store())], ast.Constant(frozenset(allLocals))
        )
        body = guardCheckers + [saveLocals, setup, compose]
        return ast.ClassDef(name, [ast.Name("DynamicScenario", loadCtx)], [], body, [])

    def makeGuardCheckers(
        self,
        args: ast.arguments,
        preconditions: list[s.Precondition],
        invariants: list[s.Invariant],
    ) -> list[ast.AST]:
        """Create a list of statements that defines precondition and invariant checker"""

        # Statements that check preconditions are satisfied
        preconditionChecks = []
        for precondition in preconditions:
            call = ast.Call(
                ast.Name("PreconditionViolation", loadCtx),
                [
                    ast.Name(behaviorArgName, loadCtx),
                    ast.Constant(precondition.lineno),
                ],
                [],
            )
            throw = ast.Raise(exc=call, cause=None)
            check = ast.If(
                test=ast.UnaryOp(ast.Not(), self.visit(precondition.value)),
                body=[throw],
                orelse=[],
            )
            preconditionChecks.append(ast.copy_location(check, precondition))
        definePreconditionChecker = ast.FunctionDef(
            checkPreconditionsName, args, preconditionChecks or [ast.Pass()], [], None
        )

        # Statements that check invariants are satisfied
        invariantChecks = []
        for invariant in invariants:
            call = ast.Call(
                ast.Name("InvariantViolation", loadCtx),
                [ast.Name(behaviorArgName, loadCtx), ast.Constant(invariant.lineno)],
                [],
            )
            throw = ast.Raise(exc=call, cause=None)
            check = ast.If(
                test=ast.UnaryOp(ast.Not(), self.visit(invariant.value)),
                body=[throw],
                orelse=[],
            )
            invariantChecks.append(ast.copy_location(check, invariant))
        defineInvariantChecker = ast.FunctionDef(
            checkInvariantsName, args, invariantChecks or [ast.Pass()], [], None
        )

        # assemble function body preamble
        preamble = [
            definePreconditionChecker,
            defineInvariantChecker,
        ]
        return preamble

    def classifyPreconditionsAndInvariants(
        self, header: list[Union[s.Precondition, s.Invariant]]
    ) -> tuple[list[s.Precondition], list[s.Invariant]]:
        """Given mixed list of preconditions and invariants, classify items into list of preconditions and list of invariants

        Args:
            header (list[Union[s.Precondition, s.Invariant]]): List of preconditions and invariants

        Returns:
            tuple[list[s.Precondition], list[s.Invariant]]: Tuple of precondition list and invariant list
        """
        preconditions: list[s.Precondition] = []
        invariants: list[s.Invariant] = []
        for n in header:
            if isinstance(n, s.Precondition):
                preconditions.append(n)
            elif isinstance(n, s.Invariant):
                invariants.append(n)
            else:
                assert False, f"Unexpected node type {n.__class__.__name__}"
        return (preconditions, invariants)

    def makeBehaviorLikeDef(
        self,
        baseClassName: str,
        name: str,
        args: Optional[ast.arguments],
        docstring: Optional[str],
        header: list[Union[s.Precondition, s.Invariant]],
        body: list[ast.AST],
    ):
        # --- Extract preconditions and invariants ---
        preconditions, invariants = self.classifyPreconditionsAndInvariants(header)

        # --- Copy arguments to the behavior object's namespace ---
        # list of all arguments
        allArgs = itertools.chain(args.posonlyargs, args.args, args.kwonlyargs)
        # statements that create argument variables
        copyArgs: list[ast.AST] = []
        for arg in allArgs:
            dest = ast.Attribute(
                ast.Name(behaviorArgName, loadCtx), arg.arg, ast.Store()
            )
            copyArgs.append(
                ast.copy_location(ast.Assign([dest], ast.Name(arg.arg, loadCtx)), arg)
            )

        # --- Create a new `arguments` ---
        newArgs: ast.arguments = self.visit(args)
        # add private current behavior argument and implicit `self` argument
        newArgs.posonlyargs = initialBehaviorArgs + newArgs.posonlyargs

        # --- Process body ---
        self.inBehavior = True
        oldBehaviorLocals = self.behaviorLocals

        self.behaviorLocals = allLocals = LocalFinder.findIn(body)

        # handle docstring
        newBody = body
        docstringNode = []
        if docstring is not None:
            docstringNode = [ast.Expr(ast.Constant(unquote(docstring)))]

        # process body
        statements = self.visit(newBody)

        generatorBody = copyArgs + statements
        generatorDefinition = ast.FunctionDef(
            name="makeGenerator",
            args=newArgs,
            body=generatorBody,
            decorator_list=[],
            returns=None,
        )

        # --- Save local variables ---
        saveLocals = ast.Assign(
            [ast.Name("_locals", ast.Store())], ast.Constant(frozenset(allLocals))
        )

        # --- Guards ---
        guardCheckers = self.makeGuardCheckers(newArgs, preconditions, invariants)

        # --- Create class definition ---
        classBody = docstringNode + guardCheckers + [saveLocals, generatorDefinition]
        classDefinition = ast.ClassDef(
            name, [ast.Name(baseClassName, loadCtx)], [], classBody, []
        )

        self.inBehavior = False
        self.behaviorLocals = oldBehaviorLocals

        return classDefinition

    def visit_Call(self, node: ast.Call) -> any:
        newArgs = []
        wrappedStar = False
        for arg in node.args:
            if isinstance(arg, ast.Starred):  # TODO(shun): check not in behavior?
                wrappedStar = True
                checkedVal = ast.Call(
                    ast.Name("wrapStarredValue", ast.Load()),
                    [self.visit(arg.value), ast.Constant(arg.value.lineno)],
                    [],
                )
                newArgs.append(ast.Starred(checkedVal, ast.Load()))
            else:
                newArgs.append(self.visit(arg))
        newKeywords = [self.visit(kwarg) for kwarg in node.keywords]
        newFunc = self.visit(node.func)
        if wrappedStar:
            newNode = ast.Call(
                ast.Name("callWithStarArgs", ast.Load()),
                [newFunc] + newArgs,
                newKeywords,
            )
        else:
            newNode = ast.Call(newFunc, newArgs, newKeywords)
        newNode = ast.copy_location(newNode, node)
        return newNode

    def visit_Model(self, node: s.Model):
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id="model", ctx=loadCtx),
                args=[
                    ast.Name(id="_Scenic_module_namespace", ctx=loadCtx),
                    ast.Constant(value=node.name),
                ],
                keywords=[],
            )
        )

    def visit_Mutate(self, node: s.Mutate):
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id="mutate", ctx=loadCtx),
                args=[self.visit(el) for el in node.elts],
                keywords=[ast.keyword(arg="scale", value=self.visit(node.scale))]
                if node.scale is not None
                else [],
            )
        )

    def visit_Param(self, node: s.Param):
        d = dict()
        for parameter in node.elts:
            if parameter.identifier in d:
                raise SyntaxError(f'Duplicated param "{parameter.identifier}"')
            d[parameter.identifier] = self.visit(parameter.value)
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(id="param", ctx=loadCtx),
                args=[
                    ast.Dict(
                        [ast.Constant(k) for k in d.keys()],
                        list(d.values()),
                    )
                ],
                keywords=[],
            )
        )

    def visit_Require(self, node: s.Require):
        condition = self.visit(node.cond)
        return self.createRequirementLike(
            "require", condition, node.lineno, node.name, node.prob
        )

    def visit_Abort(self, node: s.Abort):
        return ast.copy_location(
            ast.Return(abortFlag),
            node,
        )

    def visit_Take(self, node: s.Take):
        action = ast.Tuple(self.visit(node.elts), loadCtx)
        return self.generateInvocation(node, ast.copy_location(action, node))

    def visit_Wait(self, node: s.Wait):
        return self.generateInvocation(node, ast.Constant(()))

    def visit_Terminate(self, node: s.Terminate):
        termination = ast.Call(
            ast.Name("makeTerminationAction", loadCtx), [ast.Constant(node.lineno)], []
        )
        return self.generateInvocation(node, termination)

    def visit_Do(self, node: s.Do):
        subHandler = ast.Attribute(
            ast.Name(behaviorArgName, loadCtx), "_invokeSubBehavior", loadCtx
        )
        # TODO(shun): Check node has no more than one element inside behavior/monitors
        subArgs = [
            ast.Name("self", loadCtx),
            ast.Tuple([self.visit(e) for e in node.elts], loadCtx),
        ]
        subRunner = ast.Call(subHandler, subArgs, [])
        return self.generateInvocation(node, subRunner, ast.YieldFrom)

    def visit_DoFor(self, node: s.DoFor):
        subHandler = ast.Attribute(
            ast.Name(behaviorArgName, loadCtx), "_invokeSubBehavior", loadCtx
        )
        # TODO(shun): Check node has no more than one element inside behavior/monitors
        subArgs = [
            ast.Name("self", loadCtx),
            ast.Tuple([self.visit(e) for e in node.elts], loadCtx),
            ast.Call(
                func=ast.Name("Modifier", loadCtx),
                args=[
                    ast.Constant("for"),
                    self.visit(node.duration.value),
                    ast.Constant(node.duration.unitStr),
                ],
                keywords=[],
            ),
        ]
        subRunner = ast.Call(subHandler, subArgs, [])
        return self.generateInvocation(node, subRunner, ast.YieldFrom)

    def visit_DoUntil(self, node: s.DoUntil):
        subHandler = ast.Attribute(
            ast.Name(behaviorArgName, loadCtx), "_invokeSubBehavior", loadCtx
        )
        # TODO(shun): Check node has no more than one element inside behavior/monitors
        subArgs = [
            ast.Name("self", loadCtx),
            ast.Tuple([self.visit(e) for e in node.elts], loadCtx),
            ast.Call(
                func=ast.Name("Modifier", loadCtx),
                args=[
                    ast.Constant("until"),
                    ast.Lambda(noArgs, self.visit(node.cond)),
                ],
                keywords=[],
            ),
        ]
        subRunner = ast.Call(subHandler, subArgs, [])
        return self.generateInvocation(node, subRunner, ast.YieldFrom)

    def generateInvocation(self, node: ast.AST, actionlike, invoker=ast.Yield):
        """Generate an invocation of an action, behavior, or scenario."""
        invokeAction = ast.Expr(invoker(actionlike))
        checker = ast.Attribute(
            ast.Name(behaviorArgName, loadCtx), checkInvariantsName, loadCtx
        )
        args = ast.Starred(
            ast.Attribute(ast.Name(behaviorArgName, loadCtx), "_args", loadCtx), loadCtx
        )
        kwargs = ast.keyword(
            None, ast.Attribute(ast.Name(behaviorArgName, loadCtx), "_kwargs", loadCtx)
        )
        checkInvariants = ast.Expr(
            ast.Call(checker, [ast.Name("self", loadCtx), args], [kwargs])
        )
        ast.copy_location(invokeAction, node)
        ast.copy_location(checkInvariants, node)
        return [
            invokeAction,
            checkInvariants,
        ]

    def visit_RequireAlways(self, node: s.RequireAlways):
        condition = self.visit(node.cond)
        return self.createRequirementLike(
            "require_always", condition, node.lineno, node.name
        )

    def visit_RequireEventually(self, node: s.RequireEventually):
        condition = self.visit(node.cond)
        return self.createRequirementLike(
            "require_eventually", condition, node.lineno, node.name
        )

    def visit_Record(self, node: s.Record):
        value = self.visit(node.value)
        return self.createRequirementLike("record", value, node.lineno, node.name)

    def visit_RecordInitial(self, node: s.RecordInitial):
        value = self.visit(node.value)
        return self.createRequirementLike(
            "record_initial", value, node.lineno, node.name
        )

    def visit_RecordFinal(self, node: s.RecordFinal):
        value = self.visit(node.value)
        return self.createRequirementLike("record_final", value, node.lineno, node.name)

    def visit_TerminateWhen(self, node: s.TerminateWhen):
        condition = self.visit(node.cond)
        return self.createRequirementLike(
            "terminate_when", condition, node.lineno, None
        )

    def visit_TerminateSimulationWhen(self, node: s.TerminateSimulationWhen):
        condition = self.visit(node.cond)
        return self.createRequirementLike(
            "terminate_simulation_when", condition, node.lineno
        )

    def createRequirementLike(
        self,
        functionName: str,
        syntax: ast.AST,
        lineno: int,
        name: Optional[str] = None,
        prob: Optional[float] = None,
    ):
        """Create a call to a function that implements requirement-like features, such as `record` and `terminate when`.

        Args:
            functionName (str): Name of the requirement-like function to call. Its signature must be `(reqId: int, body: () -> bool, lineno: int, name: str | None)`
            syntax (ast.AST): AST node to evaluate for checking the condition
            lineno (int): Line number in the source code
            name (Optional[str], optional): Optional name for requirements. Defaults to None.
            prob (Optional[float], optional): Optional probability for requirements. Defaults to None.
        """
        syntax_id = self._register_requirement_syntax(syntax)
        return ast.Expr(
            value=ast.Call(
                func=ast.Name(functionName, loadCtx),
                args=[
                    ast.Constant(syntax_id),  # requirement ID
                    ast.Lambda(args=noArgs, body=syntax),  # body
                    ast.Constant(lineno),  # line number
                    ast.Constant(name),  # requirement name
                ],
                keywords=[ast.keyword(arg="prob", value=ast.Constant(prob))]
                if prob is not None
                else [],
            )
        )

    # Instance & Specifier

    def visit_New(self, node: s.New):
        return ast.Call(
            func=ast.Name(id="new", ctx=loadCtx),
            args=[
                ast.Name(id=node.className, ctx=loadCtx),
                ast.List(elts=[self.visit(s) for s in node.specifiers], ctx=ast.Load()),
            ],
            keywords=[],
        )

    def visit_WithSpecifier(self, node: s.WithSpecifier):
        return ast.Call(
            func=ast.Name(id="With", ctx=loadCtx),
            args=[
                ast.Constant(value=node.prop),
                self.visit(node.value),
            ],
            keywords=[],
        )

    def visit_AtSpecifier(self, node: s.AtSpecifier):
        return ast.Call(
            func=ast.Name(id="At", ctx=loadCtx),
            args=[self.visit(node.position)],
            keywords=[],
        )

    def visit_OffsetBySpecifier(self, node: s.OffsetBySpecifier):
        return ast.Call(
            func=ast.Name(id="OffsetBy", ctx=loadCtx),
            args=[
                self.visit(node.offset),
            ],
            keywords=[],
        )

    def visit_OffsetAlongSpecifier(self, node: s.OffsetAlongSpecifier):
        return ast.Call(
            func=ast.Name(id="OffsetAlongSpec", ctx=loadCtx),
            args=[
                self.visit(node.direction),
                self.visit(node.offset),
            ],
            keywords=[],
        )

    def visit_DirectionOfSpecifier(self, node: s.DirectionOfSpecifier):
        if isinstance(node.direction, s.LeftOf):
            fn = "LeftSpec"
        elif isinstance(node.direction, s.RightOf):
            fn = "RightSpec"
        elif isinstance(node.direction, s.AheadOf):
            fn = "Ahead"
        elif isinstance(node.direction, s.Behind):
            fn = "Behind"
        else:
            assert False, f"impossible direction {node.direction} in PositionSpecifier"
        return ast.Call(
            func=ast.Name(id=fn, ctx=loadCtx),
            args=[
                self.visit(node.position),
            ],
            keywords=(
                []
                if node.distance is None
                else [ast.keyword(arg="dist", value=self.visit(node.distance))]
            ),
        )

    def visit_BeyondSpecifier(self, node: s.BeyondSpecifier):
        return ast.Call(
            func=ast.Name(id="Beyond", ctx=loadCtx),
            args=[self.visit(node.position), self.visit(node.offset)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )

    def visit_VisibleSpecifier(self, node: s.VisibleSpecifier):
        if node.base is not None:
            return ast.Call(
                func=ast.Name(id="VisibleFrom", ctx=loadCtx),
                args=[self.visit(node.base)],
                keywords=[],
            )
        return ast.Call(
            func=ast.Name(id="VisibleSpec", ctx=loadCtx),
            args=[],
            keywords=[],
        )

    def visit_NotVisibleSpecifier(self, node: s.NotVisibleSpecifier):
        if node.base is not None:
            return ast.Call(
                func=ast.Name(id="NotVisibleFrom", ctx=loadCtx),
                args=[self.visit(node.base)],
                keywords=[],
            )
        return ast.Call(
            func=ast.Name(id="NotVisibleSpec", ctx=loadCtx),
            args=[],
            keywords=[],
        )

    def visit_InSpecifier(self, node: s.InSpecifier):
        return ast.Call(
            func=ast.Name(id="In", ctx=loadCtx),
            args=[self.visit(node.region)],
            keywords=[],
        )

    def visit_FollowingSpecifier(self, node: s.FollowingSpecifier):
        return ast.Call(
            func=ast.Name(id="Following", ctx=loadCtx),
            args=[self.visit(node.field), self.visit(node.distance)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )

    def visit_FacingSpecifier(self, node: s.FacingSpecifier):
        return ast.Call(
            func=ast.Name(id="Facing", ctx=loadCtx),
            args=[self.visit(node.heading)],
            keywords=[],
        )

    def visit_FacingTowardSpecifier(self, node: s.FacingTowardSpecifier):
        return ast.Call(
            func=ast.Name(id="FacingToward", ctx=loadCtx),
            args=[self.visit(node.position)],
            keywords=[],
        )

    def visit_ApparentlyFacingSpecifier(self, node: s.ApparentlyFacingSpecifier):
        return ast.Call(
            func=ast.Name(id="ApparentlyFacing", ctx=loadCtx),
            args=[self.visit(node.heading)],
            keywords=[ast.keyword(arg="fromPt", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )

    # Operators

    def visit_RelativePositionOp(self, node: s.RelativePositionOp):
        return ast.Call(
            func=ast.Name(id="RelativePosition", ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[]
            if node.base is None
            else [ast.keyword(arg="Y", value=self.visit(node.base))],
        )

    def visit_RelativeHeadingOp(self, node: s.RelativeHeadingOp):
        return ast.Call(
            func=ast.Name(id="RelativeHeading", ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[]
            if node.base is None
            else [ast.keyword(arg="Y", value=self.visit(node.base))],
        )

    def visit_ApparentHeadingOp(self, node: s.ApparentHeadingOp):
        return ast.Call(
            func=ast.Name(id="ApparentHeading", ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[]
            if node.base is None
            else [ast.keyword(arg="Y", value=self.visit(node.base))],
        )

    def visit_DistanceFromOp(self, node: s.DistanceFromOp):
        return ast.Call(
            func=ast.Name(id="DistanceFrom", ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[ast.keyword(arg="Y", value=self.visit(node.base))]
            if node.base is not None
            else [],
        )

    def visit_DistancePastOp(self, node: s.DistancePastOp):
        return ast.Call(
            func=ast.Name(id="DistancePast", ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[]
            if node.base is None
            else [ast.keyword(arg="Y", value=self.visit(node.base))],
        )

    def visit_AngleFromOp(self, node: s.AngleFromOp):
        assert (
            node.base is not None or node.target is not None
        ), "neither target nor base were specified in AngleFromOp"
        keywords = []
        if node.base is not None:
            keywords.append(ast.keyword("X", self.visit(node.base)))
        if node.target is not None:
            keywords.append(ast.keyword("Y", self.visit(node.target)))
        return ast.Call(
            func=ast.Name(id="AngleFrom", ctx=loadCtx),
            args=[],
            keywords=keywords,
        )

    def visit_FollowOp(self, node: s.FollowOp):
        return ast.Call(
            func=ast.Name(id="Follow", ctx=loadCtx),
            args=[
                self.visit(node.target),
                self.visit(node.base),
                self.visit(node.distance),
            ],
            keywords=[],
        )

    def visit_VisibleOp(self, node: s.VisibleOp):
        return ast.Call(
            func=ast.Name(id="Visible", ctx=loadCtx),
            args=[self.visit(node.region)],
            keywords=[],
        )

    def visit_NotVisibleOp(self, node: s.VisibleOp):
        return ast.Call(
            func=ast.Name(id="NotVisible", ctx=loadCtx),
            args=[self.visit(node.region)],
            keywords=[],
        )

    def visit_PositionOfOp(self, node: s.PositionOfOp):
        return ast.Call(
            func=ast.Name(id=node.position.functionName, ctx=loadCtx),
            args=[self.visit(node.target)],
            keywords=[],
        )

    def visit_DegOp(self, node: s.DegOp):
        return ast.BinOp(
            left=self.visit(node.operand), op=ast.Mult(), right=ast.Constant(0.01745329)
        )

    def visit_VectorOp(self, node: s.VectorOp):
        return ast.Call(
            func=ast.Name(id="Vector", ctx=loadCtx),
            args=[self.visit(node.left), self.visit(node.right)],
            keywords=[],
        )

    def visit_FieldAtOp(self, node: s.FieldAtOp):
        return ast.Call(
            func=ast.Name(id="FieldAt", ctx=loadCtx),
            args=[self.visit(node.left), self.visit(node.right)],
            keywords=[],
        )

    def visit_RelativeToOp(self, node: s.RelativeToOp):
        return ast.Call(
            func=ast.Name(id="RelativeTo", ctx=loadCtx),
            args=[self.visit(node.left), self.visit(node.right)],
            keywords=[],
        )

    def visit_OffsetAlongOp(self, node: s.OffsetAlongOp):
        return ast.Call(
            func=ast.Name(id="OffsetAlong", ctx=loadCtx),
            args=[
                self.visit(node.base),
                self.visit(node.direction),
                self.visit(node.offset),
            ],
            keywords=[],
        )

    def visit_CanSeeOp(self, node: s.CanSeeOp):
        return ast.Call(
            func=ast.Name(id="CanSee", ctx=loadCtx),
            args=[
                self.visit(node.left),
                self.visit(node.right),
            ],
            keywords=[],
        )
