from abc import ABC, abstractmethod
from functools import cached_property

from scenic.contracts.specifications import SpecNode


class Contract:
    def __init__(self, **kwargs):
        # Create and store lambdas for definitions, assumptions, and guarantees
        props = self.prop_factory(**kwargs)
        import scenic.contracts.veneer as contract_veneer

        self.definitions = props[0]
        self.assumptions_props = props[1]
        self.guarantees_props = props[2]
        self.assumptions = [
            SpecNode.propToSpec(a, contract_veneer._syntaxTrees)
            for a in self.assumptions_props
        ]
        self.guarantees = [
            SpecNode.propToSpec(g, contract_veneer._syntaxTrees)
            for g in self.guarantees_props
        ]

        self.kwargs = kwargs

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
    def verify(self):
        pass


class ContractResult(ABC):
    def __init__(self, assumptions, guarantees, component):
        self.assumptions = assumptions
        self.guarantees = guarantees
        self.component = component

    def __str__(self):
        is_probabilistic = self.correctness != 1 or self.confidence != 1

        string = (
            "Probabilistic Contract Result:\n"
            if is_probabilistic
            else "Contract Result:\n"
        )
        string += f"  Component: {self.component}\n"
        if is_probabilistic:
            string += f"  Minimum {100*self.correctness:.2f}% Correctness with {100*self.confidence:.2f}% Confidence\n"
        string += f"  Assumptions:\n{self.assumptionsSummary}"
        string += f"  Guarantees:\n{self.guaranteesSummary}"

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
    def assumptionsSummary(self):
        return "".join(f"    {a}\n" for a in self.assumptions)

    @property
    def guaranteesSummary(self):
        return "".join(f"    {g}\n" for g in self.assumptions)

    @property
    @abstractmethod
    def evidenceSummary(self):
        pass
