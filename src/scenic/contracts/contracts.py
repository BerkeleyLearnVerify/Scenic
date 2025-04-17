from abc import ABC, abstractmethod
from copy import deepcopy
from functools import cached_property

from scenic.contracts.specifications import ASTSpecTransformer, SpecNode
from scenic.syntax.compiler import NameConstantTransformer


class Contract:
    def __init__(self, **kwargs):
        # Create and store lambdas for definitions, assumptions, and guarantees
        props = self.prop_factory(**kwargs)
        import scenic.contracts.veneer as contract_veneer

        self.definitions = props[0]
        self.assumptions_props = props[1]
        self.guarantees_props = props[2]

        offset = self.def_id_offset
        def_names = list(self.definitions.keys())
        self.def_syntaxTrees = {
            def_names[i]: deepcopy(contract_veneer._syntaxTrees[offset + i])
            for i in range(len(self.definitions))
        }
        offset += len(self.definitions)
        self.assumption_syntaxTrees = [
            deepcopy(contract_veneer._syntaxTrees[offset + i])
            for i in range(len(self.assumptions_props))
        ]
        offset += len(self.assumptions_props)
        self.guarantee_syntaxTrees = [
            deepcopy(contract_veneer._syntaxTrees[offset + i])
            for i in range(len(self.guarantees_props))
        ]

        self.kwargs = kwargs
        constant_transformer = NameConstantTransformer(self.kwargs)
        for tree in (
            list(self.def_syntaxTrees.values())
            + self.assumption_syntaxTrees
            + self.guarantee_syntaxTrees
        ):
            constant_transformer.visit(tree)

        spec_transformer = ASTSpecTransformer(self.def_syntaxTrees)
        self.assumptions = [
            spec_transformer.convert(a) for a in self.assumption_syntaxTrees
        ]
        self.guarantees = [
            spec_transformer.convert(g) for g in self.guarantee_syntaxTrees
        ]

        # TODO: Handle contracts w/ more than one object
        assert len(self.objects) <= 1


class VerificationTechnique(ABC):
    @property
    @abstractmethod
    def assumptions(self):
        pass

    @property
    @abstractmethod
    def guarantees(self):
        pass

    @abstractmethod
    def verify(self, generateBatchApprox):
        pass


class ContractResult(ABC):
    def __init__(self, assumptions, guarantees, component):
        self.assumptions = assumptions
        self.guarantees = guarantees
        self.component = component

    def __str__(self):
        is_probabilistic = self.correctness != 1 or self.confidence != 1
        empty_clause = "    None\n"

        string = (
            "Probabilistic Contract Result:\n"
            if is_probabilistic
            else "Contract Result:\n"
        )
        string += f"  Component: {self.component}\n"
        if is_probabilistic:
            string += f"  Minimum {100*self.correctness:.2f}% Correctness with {100*self.confidence:.2f}% Confidence\n"
        string += f"  Assumptions:\n{self.assumptionsSummary if self.assumptionsSummary else empty_clause}"
        string += f"  Guarantees:\n{self.guaranteesSummary if self.guaranteesSummary else empty_clause}"

        evidenceSummary = self.evidenceSummary.replace("\n", "\n    ")
        string += f"  Evidence:\n    {evidenceSummary}"
        return string

    @property
    @abstractmethod
    def correctness(self):
        pass

    @property
    @abstractmethod
    def confidence(self):
        pass

    @property
    @abstractmethod
    def evidenceSummary(self):
        pass

    @property
    def assumptionsSummary(self):
        return "".join(f"    {a}\n" for a in self.assumptions)

    @property
    def guaranteesSummary(self):
        return "".join(f"    {g}\n" for g in self.guarantees)
