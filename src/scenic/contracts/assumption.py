from functools import cached_property

from scenic.contracts.contracts import ContractResult, VerificationTechnique


class Assumption(VerificationTechnique):
    def __init__(
        self,
        ## Compiler Provided ##
        contract,
        component,
    ):
        self.contract = contract
        self.component = component

    @property
    def assumptions(self):
        return self.contract.assumptions

    @property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self):
        return AssumptionContractResult(
            self.contract.assumptions, self.contract.guarantees, self.component
        )


class AssumptionContractResult(ContractResult):
    @property
    def correctness(self):
        return 1

    @property
    def confidence(self):
        return 1

    @property
    def evidenceSummary(self):
        return "Assumed"
