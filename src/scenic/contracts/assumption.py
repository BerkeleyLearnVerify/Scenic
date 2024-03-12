from scenic.contracts.contracts import ContractEvidence, ContractResult


class Assumption:
    def __init__(
        self,
        ## Compiler Provided ##
        contract,
        component,
    ):
        self.contract = contract
        self.component = component

    def verify(self):
        return ContractResult(self.contract, self.component, AssumptionEvidence())


class AssumptionEvidence(ContractEvidence):
    def __str__(self):
        return "Assumed"
