from scenic.contracts.components import ActionComponent, BaseComponent, ComposeComponent
from scenic.contracts.contracts import (
    ContractEvidence,
    ContractResult,
    VerificationTechnique,
)
from scenic.syntax.compiler import NameFinder, NameSwapTransformer


def nodeNames(node):
    nf = NameFinder()
    nf.visit(node)
    return nf.names


class Composition(VerificationTechnique):
    def __init__(
        self,
        ## Compiler Provided ##
        component,
        sub_stmts,
    ):
        ## Initialization ##
        # Store parameters
        self.component = component
        self.sub_stmts = sub_stmts

        # Ensure all sub statements are of a valid form
        for stmt in self.sub_stmts:
            assert isinstance(stmt, VerificationTechnique)

        # Extract subcomponents and connections if applicable
        subcomponents = (
            component.subcomponents if isinstance(component, ComposeComponent) else {}
        )
        connections = (
            component.connections if isinstance(component, ComposeComponent) else []
        )

        # Check that all sub-statements are applied to this component
        # or one of it's direct sub-components.
        for stmt in sub_stmts:
            assert stmt.component is component or stmt.component in subcomponents.values()

        ## Composition checks ##
        import scenic.contracts.veneer as contracts_veneer

        # Compute general port-variable mapping
        encoding_map = {}
        for var_num, mapping in enumerate(connections):
            # Unpack mapping and come up with a new intermediate variable name
            source, dest = mapping
            temp_var_name = self.tempVarName(var_num)

            # Map both variables to the new name
            encoding_map[source] = temp_var_name
            encoding_map[dest] = temp_var_name

        # Compute encoding visitor for each subcomponent
        # The encoding visitor should encode all connected port variables to the appropriate
        # unifying temp variable.
        encoding_visitors = {}
        for subcomponent in subcomponents:
            name_map = {}

            for source_info, target_name in encoding_map.items():
                if source_info[1] == subcomponent:
                    name_map[source_info[0]] = target_name

            encoding_visitors[subcomponent] = NameSwapTransformer(name_map)

        # Compute general decoding visitor
        decoding_map = {
            target: source[0]
            for source, target in encoding_map.items()
            if source[1] is None
        }
        assert sum(1 for source, _ in encoding_map.items() if source[1] is None) == len(
            decoding_map
        )
        decoding_visitor = NameSwapTransformer(decoding_map)

        # Move through sub_stmts linearly, checking assumptions and accumulating guarantees
        tl_assumptions = []
        tl_guarantees = []

        for sub_stmt in sub_stmts:
            pass
            ## Encode assumptions and guarantees ##

            ## Decode assumptions and guarantees ##
            # If any assumptions are still using temporary variable names after decoding, then they are
            # internal assumptions which are are not being satisfied, and this is not a valid IO contract
            # and we should raise an error.
            # If any guarantees are still using temporary variable names after decoding, then they're
            # internal guarantees which aren't relevant to our composition and we can drop them.

        # TODO: Clear atomic source syntax strings?

    @staticmethod
    def tempVarName(temp_var_name):
        return f"SCENIC_INTERNAL_VAR_{temp_var_name}"
