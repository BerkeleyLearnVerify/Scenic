from functools import cached_property

from scenic.contracts.contracts import ContractResult, VerificationTechnique


class Refinement(VerificationTechnique):
    def __init__(self, stmt, contract, method):
        self.stmt = stmt
        self.contract = contract
        self.method = method

        self.method.check(self.stmt, self.contract)

    @cached_property
    def assumptions(self):
        return self.contract.assumptions

    @cached_property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self):
        return RefinementContractResult(
            self.assumptions,
            self.guarantees,
            self.stmt.component,
            self.stmt.verify(),
            self.method,
        )


class RefinementContractResult(ContractResult):
    def __init__(self, assumptions, guarantees, component, sub_result, method):
        super().__init__(assumptions, guarantees, component)
        self.sub_result = sub_result
        self.method = method

    @cached_property
    def correctness(self):
        return self.sub_result.correctness

    @cached_property
    def confidence(self):
        return self.sub_result.confidence

    @property
    def evidenceSummary(self):
        string = ""
        string += f"Refinement Method: {self.method}\n"
        string += self.sub_result.evidenceSummary
        return string


class LeanRefinementProof:
    def __init__(self, proof_loc):
        self.proof_loc = proof_loc

    def check(self, stmt, contract):
        # breakpoint()
        #
        pass

    def __str__(self):
        return f"LeanProof ('{self.proof_loc}')"
