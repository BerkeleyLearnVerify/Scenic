class Contract:
    def __init__(self, **kwargs):
        # Create and store lambdas for definitions, assumptions, and guarantees
        props = self.prop_factory(**kwargs)
        self.definitions = props[0]
        self.assumptions = props[1]
        self.guarantees = props[2]

        self.kwargs = kwargs

        # TODO: Handle contracts w/ more than one object
        assert len(self.objects) <= 1


class _ContractResultBase:
    def __init__(self, contract, component, evidence):
        self.contract = contract
        self.component = component
        self.assumptions = contract.assumptions
        self.guarantees = contract.guarantees
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


class ProbabilisticContractResult(_ContractResultBase):
    def __init__(self, contract, component, evidence):
        from scenic.contracts.testing import ProbabilisticEvidence

        if not isinstance(evidence, ProbabilisticEvidence):
            raise ValueError("Evidence provided is not ProbabilisticEvidence")

        super().__init__(contract, component, evidence)

    def __str__(self):
        string = "ContractResult:\n"
        string += f"  Component: {self.component}\n"
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

        string += f"  Evidence: \n"
        string += "    " + str(self.evidence).replace("\n", "\n    ")
        return string


class ContractEvidence:
    pass
