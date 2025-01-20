from abc import ABC, abstractmethod
import ast
from copy import copy, deepcopy
from itertools import zip_longest

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
            return Always(self.convert(node.value), self.defInfo)
        if isinstance(node, scenic_ast.Eventually):
            return Eventually(self.convert(node.value), self.defInfo)
        if isinstance(node, scenic_ast.Next):
            return Next(self.convert(node.value), self.defInfo)
        if isinstance(node, scenic_ast.UntilOp):
            return Until(self.convert(node.left), self.convert(node.right), self.defInfo)
        if isinstance(node, scenic_ast.ImpliesOp):
            return Implies(
                self.convert(node.hypothesis), self.convert(node.conclusion), self.defInfo
            )

        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
            return Not(self.convert(node.operand), self.defInfo)

        if isinstance(node, ast.BoolOp):
            if isinstance(node.op, ast.And):
                return And([self.convert(v) for v in node.values], self.defInfo)
            if isinstance(node.op, ast.Or):
                return Or([self.convert(v) for v in node.values], self.defInfo)

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

                    return Op(p1, p2, self.defInfo)

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

                    return And(
                        [Op1(p1, p2, self.defInfo), Op2(p2, p3, self.defInfo)],
                        self.defInfo,
                    )

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

                return Op(p1, p2, self.defInfo)

        if isinstance(node, ast.Constant):
            return ConstantSpecNode(node.value, self.defInfo)

        return Atomic(node, self.defInfo)


class SpecNode(ABC):
    def __init__(self, defInfo):
        self.defSyntaxTrees, self.defSpecs = defInfo

    @abstractmethod
    def applyAtomicTransformer(self, transformer):
        pass

    @abstractmethod
    def getAtomicNames(self):
        pass

    @staticmethod
    def syntaxTreeToSyntaxVal(targetTree, syntaxMappings):
        # Check if there's a tree in the mappings that's equivalent to this one
        for existingTree in syntaxMappings:
            if equivalentAST(targetTree, existingTree):
                return syntaxMappings[existingTree]

        syntaxMappings[targetTree] = len(syntaxMappings)
        return syntaxMappings[targetTree]

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
                if not SpecNode.equivalentAST(
                    val, getattr(node2, name), node1_defSyntaxTrees, node2_defSyntaxTrees
                ):
                    return False
            return True

        elif isinstance(node1, list) and isinstance(node2, list):
            return all(
                SpecNode.equivalentAST(n1, n2, node1_defSyntaxTrees, node2_defSyntaxTrees)
                for n1, n2 in zip_longest(node1, node2)
            )
        else:
            return node1 == node2


class Atomic(SpecNode):
    def __init__(self, ast, defInfo):
        self.ast = ast
        super().__init__(defInfo)

    def applyAtomicTransformer(self, transformer):
        self.ast = transformer.visit(self.ast)

    def getAtomicNames(self):
        nf = NameFinder()
        nf.visit(self.ast)
        return nf.names

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


class ConstantSpecNode(SpecNode):
    def __init__(self, value, defInfo):
        self.value = value
        super().__init__(defInfo)

    def applyAtomicTransformer(self, transformer):
        pass

    def getAtomicNames(self):
        return set()

    def clearSourceStrings(self):
        pass

    def __eq__(self, other):
        return type(self) is type(other) and self.value == other.value

    def __str__(self):
        return str(self.value)


class UnarySpecNode(SpecNode):
    def __init__(self, sub, defInfo):
        assert isinstance(sub, SpecNode)
        self.sub = sub
        super().__init__(defInfo)

    def applyAtomicTransformer(self, transformer):
        self.sub.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return self.sub.getAtomicNames()

    def clearSourceStrings(self):
        self.sub.clearSourceStrings()

    def __eq__(self, other):
        return type(self) is type(other) and self.sub == other.sub


class BinarySpecNode(SpecNode):
    def __init__(self, sub1, sub2, defInfo):
        assert isinstance(sub1, SpecNode) and isinstance(sub2, SpecNode)
        self.sub1, self.sub2 = sub1, sub2
        super().__init__(defInfo)

    def applyAtomicTransformer(self, transformer):
        self.sub1.applyAtomicTransformer(transformer)
        self.sub2.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return self.sub1.getAtomicNames() | self.sub2.getAtomicNames()

    def clearSourceStrings(self):
        self.sub1.clearSourceStrings()
        self.sub2.clearSourceStrings()

    def __eq__(self, other):
        return (
            type(self) is type(other)
            and self.sub1 == other.sub1
            and self.sub2 == other.sub2
        )


class NarySpecNode(SpecNode):
    def __init__(self, subs, defInfo):
        assert all(isinstance(sub, SpecNode) for sub in subs)
        self.subs = subs
        super().__init__(defInfo)

    def applyAtomicTransformer(self, transformer):
        for sub in self.subs:
            sub.applyAtomicTransformer(transformer)

    def getAtomicNames(self):
        return set().union(*(sub.getAtomicNames() for sub in self.subs))

    def clearSourceStrings(self):
        for sub in self.subs:
            self.sub.clearSourceStrings()

    def __eq__(self, other):
        return type(self) is type(other) and self.subs == other.subs


class Always(UnarySpecNode):
    def __str__(self):
        return f"always ({self.sub})"


class Eventually(UnarySpecNode):
    def __str__(self):
        return f"eventually ({self.sub})"


class Next(UnarySpecNode):
    def __str__(self):
        return f"next ({self.sub})"


class Not(UnarySpecNode):
    def __str__(self):
        return f"not ({self.sub})"


class Until(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) until ({self.sub2})"


class Implies(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) implies ({self.sub2})"


class Equal(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) == ({self.sub2})"


class GT(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) > ({self.sub2})"


class GE(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) >= ({self.sub2})"


class LT(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) < ({self.sub2})"


class LE(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) <= ({self.sub2})"


class Add(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) + ({self.sub2})"


class Sub(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) - ({self.sub2})"


class Mul(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) * ({self.sub2})"


class Div(BinarySpecNode):
    def __str__(self):
        return f"({self.sub1}) / ({self.sub2})"


class Min(BinarySpecNode):
    def __str__(self):
        return f"Min(({self.sub1}), ({self.sub2}))"


class Max(BinarySpecNode):
    def __str__(self):
        return f"Max(({self.sub1}), ({self.sub2}))"


class And(NarySpecNode):
    def __str__(self):
        return " and ".join(f"({str(sub)})" for sub in self.subs)


class Or(NarySpecNode):
    def __str__(self):
        return " or ".join(f"({str(sub)})" for sub in self.subs)
