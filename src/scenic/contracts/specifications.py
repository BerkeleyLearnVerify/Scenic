from abc import ABC, abstractmethod
import ast
from copy import copy, deepcopy
from itertools import chain, zip_longest

import scenic.core.propositions as propositions
import scenic.syntax.ast as scenic_ast
from scenic.syntax.compiler import NameFinder


class ASTSpecTransformer:
    def __init__(self, defSyntaxTrees):
        self.defSyntaxTrees = defSyntaxTrees
        self.defSpecs = {}

        self.defInfo = (self.defSyntaxTrees, self.defSpecs)

        for name, node in self.defSyntaxTrees.items():
            self.defSpecs = copy(self.defSpecs)
            self.defSpecs[name] = self.convert(node)
            self.defInfo = (self.defSyntaxTrees, self.defSpecs)

    def convert(self, node):
        assert isinstance(node, ast.AST)

        if isinstance(node, scenic_ast.Always):
            return Always(self.convert(node.value))
        if isinstance(node, scenic_ast.Eventually):
            return Eventually(self.convert(node.value))
        if isinstance(node, scenic_ast.ContractNext):
            return Next(self.convert(node.target))
        if isinstance(node, scenic_ast.UntilOp):
            return Until(self.convert(node.left), self.convert(node.right))
        if isinstance(node, scenic_ast.ImpliesOp):
            return Implies(self.convert(node.hypothesis), self.convert(node.conclusion))

        if isinstance(node, ast.UnaryOp):
            if isinstance(node.op, ast.USub):
                return Neg(self.convert(node.operand))
            if isinstance(node.op, ast.Not):
                return Not(self.convert(node.operand))

        if isinstance(node, ast.BoolOp):
            if isinstance(node.op, ast.And):
                return And([self.convert(v) for v in node.values])
            if isinstance(node.op, ast.Or):
                return Or([self.convert(v) for v in node.values])

        if isinstance(node, ast.Compare):
            if len(node.ops) == 1:
                Op = None
                if isinstance(node.ops[0], ast.Eq):
                    Op = Equal
                elif isinstance(node.ops[0], ast.Lt):
                    Op = LT
                elif isinstance(node.ops[0], ast.LtE):
                    Op = LE
                elif isinstance(node.ops[0], ast.Gt):
                    Op = GT
                elif isinstance(node.ops[0], ast.GtE):
                    Op = GE

                if Op:
                    p1 = self.convert(node.left)
                    p2 = self.convert(node.comparators[0])

                    return Op(p1, p2)

            if len(node.ops) == 2:
                Op1 = None
                if isinstance(node.ops[0], ast.Eq):
                    Op1 = Equal
                elif isinstance(node.ops[0], ast.Lt):
                    Op1 = LT
                elif isinstance(node.ops[0], ast.LtE):
                    Op1 = LE
                elif isinstance(node.ops[0], ast.Gt):
                    Op1 = GT
                elif isinstance(node.ops[0], ast.GtE):
                    Op1 = GE

                Op2 = None
                if isinstance(node.ops[1], ast.Eq):
                    Op2 = Equal
                elif isinstance(node.ops[1], ast.Lt):
                    Op2 = LT
                elif isinstance(node.ops[1], ast.LtE):
                    Op2 = LE
                elif isinstance(node.ops[1], ast.Gt):
                    Op2 = GT
                elif isinstance(node.ops[1], ast.GtE):
                    Op2 = GE

                if Op1 and Op2:
                    p1 = self.convert(node.left)
                    p2 = self.convert(node.comparators[0])
                    p3 = self.convert(node.comparators[1])

                    return And([Op1(p1, p2), Op2(p2, p3)])

        if isinstance(node, ast.BinOp):
            Op = None
            if isinstance(node.op, ast.Add):
                Op = Add
            elif isinstance(node.op, ast.Sub):
                Op = Sub
            elif isinstance(node.op, ast.Mult):
                Op = Mul
            elif isinstance(node.op, ast.Div):
                Op = Div

            if Op:
                p1 = self.convert(node.left)
                p2 = self.convert(node.right)

                return Op(p1, p2)

        if isinstance(node, ast.Call):
            if (
                isinstance(node.func, ast.Name)
                and node.func.id == "ceil"
                and len(node.args) == 1
            ):
                return Ceil(self.convert(node.args[0]))

            if (
                isinstance(node.func, ast.Name)
                and node.func.id == "min"
                and len(node.args) == 2
            ):
                return Min(self.convert(node.args[0]), self.convert(node.args[1]))

            if (
                isinstance(node.func, ast.Name)
                and node.func.id == "max"
                and len(node.args) == 2
            ):
                return Max(self.convert(node.args[0]), self.convert(node.args[1]))

        if isinstance(node, ast.Constant):
            return ConstantSpecNode(node.value)

        if isinstance(node, ast.Name) and node.id in self.defSpecs:
            return DefSpecNode(node.id, self.defInfo)

        return Atomic(node, self.defInfo)


class SpecNode(ABC):
    @abstractmethod
    def applyAtomicTransformer(self, transformer):
        pass

    @abstractmethod
    def getAtomicNames(self):
        pass

    @abstractmethod
    def getDefs(self):
        pass

    @abstractmethod
    def toPACTIStr(self, syntaxMappings):
        pass

    @abstractmethod
    def toPACTITemp(self, syntaxMappings):
        # TODO: Remove this temp function once PACTI supports nested functions/temporal ops.
        pass

    def getContractVars(self):
        return self.extractTempVars(self.getAtomicNames())

    @staticmethod
    def syntaxTreeToSyntaxVal(targetTree, syntaxMappings):
        # Check if there's a tree in the mappings that's equivalent to this one
        for existingTree in syntaxMappings:
            if SpecNode.equivalentAST(targetTree, existingTree):
                return syntaxMappings[existingTree]

    @abstractmethod
    def getAtomics(self):
        pass

    @abstractmethod
    def toLean(self):
        pass


class Atomic(SpecNode):
    def __init__(self, ast, defInfo):
        self.ast = ast
        self.defSyntaxTrees, _ = defInfo

    def applyAtomicTransformer(self, transformer):
        self.ast = transformer.visit(self.ast)

    def getAtomicNames(self):
        nf = NameFinder()
        nf.visit(self.ast)
        return tuple(nf.names)

    def getDefs(self):
        return tuple()

    def getAtomics(self, ctx=bool):
        return ((self, ctx),)

    @staticmethod
    def equivalentAST(node1, node2, node1_defSyntaxTrees, node2_defSyntaxTrees):
        # Check if either name references a def
        if isinstance(node1, ast.Name) and node1.id in node1_defSyntaxTrees:
            node1 = node1_defSyntaxTrees[node1.id]
        if isinstance(node2, ast.Name) and node2.id in node2_defSyntaxTrees:
            node2 = node2_defSyntaxTrees[node2.id]

        if type(node1) is not type(node2):
            return False

        if isinstance(node1, ast.AST):
            for name, val in vars(node1).items():
                if name in {
                    "lineno",
                    "end_lineno",
                    "col_offset",
                    "end_col_offset",
                    "ctx",
                }:
                    continue
                if not Atomic.equivalentAST(
                    val, getattr(node2, name), node1_defSyntaxTrees, node2_defSyntaxTrees
                ):
                    return False
            return True

        elif isinstance(node1, list) and isinstance(node2, list):
            return all(
                Atomic.equivalentAST(n1, n2, node1_defSyntaxTrees, node2_defSyntaxTrees)
                for n1, n2 in zip_longest(node1, node2)
            )
        else:
            return node1 == node2

    @staticmethod
    def extractTempVars(var_iterable):
        from scenic.contracts.composition import Composition

        return {var for var in var_iterable if Composition.isTempVar(var)}

    def toLean(self, ctx=bool):
        # TODO: Better name replacement
        str_map = {
            "(": "",
            ")": "",
            "[": "_",
            "]": "_",
            " ": "_",
            '"': "_",
            "'": "_",
            "==": "_EQ_",
        }

        self_str = str(self)
        for m in str_map.items():
            self_str = self_str.replace(m[0], m[1])

        return self_str

    def __eq__(self, other):
        return type(self) is type(other) and self.equivalentAST(
            self.ast, other.ast, self.defSyntaxTrees, other.defSyntaxTrees
        )

    def __str__(self):
        if (
            isinstance(self.ast, ast.Slice)
            and isinstance(self.ast.slice, ast.Name)
            and ast.slice.id == SCENIC_INTERNAL_TIME
        ):
            return ast.unparse(self.ast.value)

        return ast.unparse(self.ast)


class DefSpecNode(SpecNode):
    def __init__(self, name, defInfo):
        self.name = name
        _, self.defSpecs = defInfo

    def applyAtomicTransformer(self, transformer):
        self.defSpecs[self.name].applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return self.defSpecs[self.name].getAtomicNames()

    def getDefs(self):
        return (
            (
                list(self.defSpecs.keys()).index(self.name),
                (self.name, self.defSpecs[self.name]),
            ),
        ) + self.defSpecs[self.name].getDefs()

    def getAtomics(self, ctx=bool):
        return self.defSpecs[self.name].getAtomics(ctx)

    def toLean(self, ctx=bool):
        return self.name

    def __eq__(self, other):
        return (
            type(self) is type(other)
            and self.defSpecs[self.name] == other.defSpecs[other.name]
        )

    def __str__(self):
        return self.name


class ConstantSpecNode(SpecNode):
    def __init__(self, value):
        self.value = value

    def applyAtomicTransformer(self, transformer):
        pass

    def getAtomicNames(self):
        return tuple()

    def getDefs(self):
        return tuple()

    def getAtomics(self, ctx=bool):
        return tuple()

    def toLean(self, ctx=bool):
        return str(self.value)

    def __eq__(self, other):
        return type(self) is type(other) and self.value == other.value

    def __str__(self):
        return str(self.value)


class UnarySpecNode(SpecNode):
    def __init__(self, sub):
        assert isinstance(sub, SpecNode)
        self.sub = sub

    def applyAtomicTransformer(self, transformer):
        self.sub.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return self.sub.getAtomicNames()

    def getDefs(self):
        return self.sub.getDefs()

    def getAtomics(self, ctx=bool):
        return self.sub.getAtomics(self.ctx)

    def __eq__(self, other):
        return type(self) is type(other) and self.sub == other.sub


class BinarySpecNode(SpecNode):
    def __init__(self, sub1, sub2):
        assert isinstance(sub1, SpecNode) and isinstance(sub2, SpecNode)
        self.sub1, self.sub2 = sub1, sub2

    def applyAtomicTransformer(self, transformer):
        self.sub1.applyAtomicTransformer(transformer)
        self.sub2.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return self.sub1.getAtomicNames() + self.sub2.getAtomicNames()

    def getDefs(self):
        return self.sub1.getDefs() + self.sub2.getDefs()

    def getAtomics(self, ctx=bool):
        return self.sub1.getAtomics(self.ctx) + self.sub2.getAtomics(self.ctx)

    def __eq__(self, other):
        return (
            type(self) is type(other)
            and self.sub1 == other.sub1
            and self.sub2 == other.sub2
        )


class NarySpecNode(SpecNode):
    def __init__(self, subs):
        assert all(isinstance(sub, SpecNode) for sub in subs)
        assert len(subs) > 1
        self.subs = subs

    def applyAtomicTransformer(self, transformer):
        for sub in self.subs:
            sub.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return tuple(chain(*(sub.getAtomicNames() for sub in self.subs)))

    def getDefs(self):
        return tuple(chain(*(sub.getDefs() for sub in self.subs)))

    def getAtomics(self, ctx=bool):
        return tuple(chain(*(sub.getAtomics(self.ctx) for sub in self.subs)))

    def __eq__(self, other):
        return type(self) is type(other) and self.subs == other.subs


class Atomic(SpecNode):
    def __init__(self, ast, source_str=None):
        self.ast = ast
        self.source_str = source_str

    def applyAtomicTransformer(self, transformer):
        self.ast = transformer.visit(self.ast)

    def getAtomicNames(self):
        nf = NameFinder()
        nf.visit(self.ast)
        return nf.names

    def toPACTIStr(self, syntaxMappings):
        func_name = self.syntaxTreeToSyntaxVal(self.ast, syntaxMappings)
        func_vars = ", ".join(self.getContractVars())
        return f"Atomic_{func_name}({func_vars})"

    def toPACTITemp(self, syntaxMappings):
        return f"Atomic_{self.syntaxTreeToSyntaxVal(self.ast, syntaxMappings)}"

    def clearSourceStrings(self):
        self.source_str = None

    def __eq__(self, other):
        return type(self) is type(other) and self.equivalentAST(self.ast, other.ast)

    def __str__(self):
        return self.source_str if self.source_str else ast.unparse(self.ast)


class Always(UnarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return f"G ({self.sub.toLean()})"

    def toPACTIStr(self, syntaxMappings):
        return f"{self.toPACTITemp(syntaxMappings)}({', '.join(self.getContractVars())})"

    def toPACTITemp(self, syntaxMappings):
        return f"ALW_{self.sub.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"always ({self.sub})"


class Eventually(UnarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return f"F ({self.sub.toLean()})"

    def toPACTIStr(self, syntaxMappings):
        return f"{self.toPACTITemp(syntaxMappings)}({', '.join(self.getContractVars())})"

    def toPACTITemp(self, syntaxMappings):
        return f"EVN_{self.sub.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"eventually ({self.sub})"


class Next(UnarySpecNode):
    def toLean(self, ctx=bool):
        if ctx is bool:
            return f"Xʷ ({self.sub.toLean(ctx)})"
        else:
            return f"X ({self.sub.toLean(ctx)})"

    def getAtomics(self, ctx=bool):
        return self.sub.getAtomics(ctx)

    def toPACTIStr(self, syntaxMappings):
        return f"{self.toPACTITemp(syntaxMappings)}({', '.join(self.getContractVars())})"

    def toPACTITemp(self, syntaxMappings):
        return f"NXT_{self.sub.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"next ({self.sub})"


class Not(UnarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return f"¬({self.sub.toLean()})"

    def toPACTIStr(self, syntaxMappings):
        return f"~({self.sub.toPACTIStr(syntaxMappings)})"

    def toPACTITemp(self, syntaxMappings):
        return f"NOT_{self.sub.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"not ({self.sub})"


class Neg(UnarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"-({self.sub.toLean(float)})"

    def __str__(self):
        return f"-({self.sub})"


class Ceil(UnarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"⌈{self.sub.toLean(float)}⌉"

    def __str__(self):
        return f"ceil({self.sub})"


class Until(BinarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean()}) U ({self.sub2.toLean()})"

    def toPACTIStr(self, syntaxMappings):
        return f"{self.toPACTITemp(syntaxMappings)}({', '.join(self.getContractVars())})"

    def toPACTITemp(self, syntaxMappings):
        return f"UNTIL_{self.sub1.toPACTITemp(syntaxMappings)}_{self.sub2.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"({self.sub1}) until ({self.sub2})"


class Implies(BinarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean()}) → ({self.sub2.toLean()})"

    def toPACTIStr(self, syntaxMappings):
        return f"({self.sub1.toPACTIStr(syntaxMappings)}) => ({self.sub2.toPACTIStr(syntaxMappings)})"

    def toPACTITemp(self, syntaxMappings):
        return f"IMP_{self.sub1.toPACTITemp(syntaxMappings)}_{self.sub2.toPACTITemp(syntaxMappings)}"

    def __str__(self):
        return f"({self.sub1}) implies ({self.sub2})"


class Equal(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) == ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) == ({self.sub2})"


class GT(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) > ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) > ({self.sub2})"


class GE(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) ≥ ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) >= ({self.sub2})"


class LT(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) < ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) < ({self.sub2})"


class LE(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) ≤ ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) <= ({self.sub2})"


class Add(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) + ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) + ({self.sub2})"


class Sub(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) - ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) - ({self.sub2})"


class Mul(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) * ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) * ({self.sub2})"


class Div(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) / ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"({self.sub1}) / ({self.sub2})"


class Min(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) ⊓ ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"min(({self.sub1}), ({self.sub2}))"


class Max(BinarySpecNode):
    ctx = float

    def toLean(self, ctx=bool):
        return f"({self.sub1.toLean(float)}) ⊔ ({self.sub2.toLean(float)})"

    def __str__(self):
        return f"max(({self.sub1}), ({self.sub2}))"


class And(NarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return " ∧ ".join(f"({sub.toLean()})" for sub in self.subs)

    def toPACTIStr(self, syntaxMappings):
        pacti_str = f"({self.sub1.toPACTIStr(syntaxMappings)}) & ({self.sub2.toPACTIStr(syntaxMappings)})"
        for sub in self.subs[2:]:
            pacti_str = "(" + pacti_str + f" & {self.sub.toPACTIStr(syntaxMappings)})"

        return pacti_str

    def toPACTITemp(self, syntaxMappings):
        return f"AND_{'_'.join(sub.toPACTITemp(syntaxMappings) for sub in self.subs)}"

    def __str__(self):
        return " and ".join(f"({str(sub)})" for sub in self.subs)


class Or(NarySpecNode):
    ctx = bool

    def toLean(self, ctx=bool):
        return " ∨ ".join(f"({sub.toLean()})" for sub in self.subs)
        
    def toPACTIStr(self, syntaxMappings):
        pacti_str = f"({self.sub1.toPACTIStr(syntaxMappings)}) | ({self.sub2.toPACTIStr(syntaxMappings)})"
        for sub in self.subs[2:]:
            pacti_str = "(" + pacti_str + f" | {self.sub.toPACTIStr(syntaxMappings)})"

        return pacti_str

    def toPACTITemp(self, syntaxMappings):
        return f"OR_{'_'.join(sub.toPACTITemp(syntaxMappings) for sub in self.subs)}"

    def __str__(self):
        return " or ".join(f"({str(sub)})" for sub in self.subs)
