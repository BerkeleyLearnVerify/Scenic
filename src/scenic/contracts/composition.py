from scenic.contracts.contracts import (
    ContractEvidence,
    ContractResult,
    VerificationTechnique,
)


class Composition(VerificationTechnique):
    def __init__(
        self,
        ## Compiler Provided ##
        component,
        sub_stmts,
    ):
        # Store parameters
        self.component = component
        self.sub_stmts = sub_stmts

        # Ensure all sub statements are of a valid form
        for stmt in self.sub_stmts:
            assert isinstance(stmt, VerificationTechnique)

        # TODO: Proper assumption/guaranteee checking
        import scenic.contracts.veneer as contracts_veneer

        print(contracts_veneer.getSyntaxTrees())
