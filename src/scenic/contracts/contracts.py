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


class ContractResult:
    def __init__(self, assumptions, guarantees, evidence):
        self.assumptions = assumptions
        self.guarantees = guarantees
        self.evidence = evidence

    def __str__(self):
        string = "ContractResult:\n"
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
