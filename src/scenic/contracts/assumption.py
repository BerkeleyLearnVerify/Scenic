from functools import cached_property

from scenic.contracts.contracts import (
    ContractEvidence,
    ContractResult,
    VerificationTechnique,
)


class Assumption(VerificationTechnique):
    def __init__(
        self,
        ## Compiler Provided ##
        contract,
        component,
    ):
        self.contract = contract
        self.component = component

    @cached_property
    def assumptions(self):
        return self.contract.assumptions

    @cached_property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self):
        return ContractResult(
            self.contract.assumptions,
            self.contract.guarantees,
            self.component,
            AssumptionEvidence(),
        )


class AssumptionEvidence(ContractEvidence):
    def __str__(self):
        return "Assumed"
