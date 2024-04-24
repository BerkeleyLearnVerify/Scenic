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
    @cached_property
    @abstractmethod
    def assumptions(self):
        pass

    @cached_property
    @abstractmethod
    def guarantees(self):
        pass

    @abstractmethod
    def verify(self):
        pass


class _ContractResultBase:
    def __init__(self, assumptions, guarantees, component, evidence):
        self.assumptions = assumptions
        self.guarantees = guarantees
        self.component = component
        self.evidence = evidence


class ContractResult(_ContractResultBase):
    def __str__(self):
        string = "ContractResult:\n"
        string += f"  Component: {self.component}\n"

        string += "  Assumptions:\n"
        for a in self.assumptions:
            string += f"    {a}\n"

        string += "  Guarantees:\n"
        for g in self.guarantees:
            string += f"    {g}\n"

        string += f"  Evidence: \n"
        string += "    " + str(self.evidence).replace("\n", "\n    ")
        return string

    @property
    def correctness(self):
        return 1

    @property
    def confidence(self):
        return 1


class ProbabilisticContractResult(_ContractResultBase):
    def __init__(self, assumptions, guarantees, component, evidence):
        from scenic.contracts.testing import ProbabilisticEvidence

        super().__init__(assumptions, guarantees, component, evidence)

    @property
    def correctness(self):
        return self.evidence.correctness

    @property
    def confidence(self):
        return self.evidence.confidence

    def __str__(self):
        string = "ProbabilisticContractResult:\n"
        string += f"  Minimum {100*self.correctness:.2f}% Correctness with {100*self.confidence:.2f}% Confidence\n"
        string += f"  Component: {self.component}\n"
        from scenic.contracts.testing import ProbabilisticEvidence, TestResult

        if isinstance(self.evidence, ProbabilisticEvidence):
            string += "  Assumptions:\n"
            for ai, a in enumerate(self.assumptions):
                if self.evidence.a_count == 0:
                    percent_violated = 0
                else:
                    percent_violated = (
                        sum(
                            1 / len(at.violations)
                            for at in self.evidence.testData
                            if at.result == TestResult.A and ai in at.violations
                        )
                        / self.evidence.a_count
                    )

                string += f"    ({percent_violated*100:6.2f}%) {a}\n"

            string += "  Guarantees:\n"

            for gi, g in enumerate(self.guarantees):
                if self.evidence.g_count == 0:
                    percent_violated = 0
                else:
                    percent_violated = (
                        sum(
                            1 / len(gt.violations)
                            for gt in self.evidence.testData
                            if gt.result == TestResult.G and gi in gt.violations
                        )
                        / self.evidence.g_count
                    )

                string += f"    ({percent_violated*100:6.2f}%) {g}\n"
        else:
            string += "  Assumptions:\n"
            for a in self.assumptions:
                string += f"    {a}\n"

            string += "  Guarantees:\n"
            for g in self.guarantees:
                string += f"    {g}\n"

        string += f"  Evidence: \n"
        string += "    " + str(self.evidence).replace("\n", "\n    ")
        return string


class ContractEvidence:
    pass
