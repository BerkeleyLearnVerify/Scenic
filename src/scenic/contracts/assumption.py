from functools import cached_property

from scenic.contracts.contracts import ContractResult, VerificationTechnique


class Assumption(VerificationTechnique):
    def __init__(
        self,
        ## Compiler Provided ##
        contract,
        component,
        correctness,
        confidence,
    ):
        self.contract = contract
        self.component = component
        self.correctness = correctness
        self.confidence = confidence

    @property
    def assumptions(self):
        return self.contract.assumptions

    @property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self):
        return AssumptionContractResult(
            self.contract.assumptions,
            self.contract.guarantees,
            self.component,
            self.correctness,
            self.confidence,
        )


class AssumptionContractResult(ContractResult):
    def __init__(self, assumptions, guarantees, component, correctness, confidence):
        super().__init__(assumptions, guarantees, component)
        self._correctness = correctness
        self._confidence = confidence

    @property
    def correctness(self):
        return self._correctness

    @property
    def confidence(self):
        return self._confidence

    @property
    def evidenceSummary(self):
        return "Assumed"
