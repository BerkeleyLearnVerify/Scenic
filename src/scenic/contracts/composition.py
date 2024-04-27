from copy import deepcopy
from functools import cached_property
from math import prod

from scenic.contracts.components import ActionComponent, BaseComponent, ComposeComponent
from scenic.contracts.contracts import (
    ContractEvidence,
    ContractResult,
    ProbabilisticContractResult,
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
        for stmt in self.sub_stmts:
            assert stmt.component is component or stmt.component in subcomponents.values()

        ## Composition checks ##
        import scenic.contracts.veneer as contracts_veneer

        # Compute general port-variable mapping
        # Encoding map is a dictionary mapping tuples (port_name, subcomponent_name)
        # to a temporary variable. Top level ports have None as subcomponent name.
        encoding_map = {}
        self.var_num = 0

        # Assign variables to all connections
        for source, dest in connections:
            # Check if we already have a temporary variable name and if not
            # come up with a new intermediate variable name.
            temp_var_name = (
                encoding_map[source] if source in encoding_map else self.tempVarName()
            )

            # Map both variables to the new name
            encoding_map[source] = temp_var_name
            encoding_map[dest] = temp_var_name

        # Assign any remaining subcomponent port variables to temp names
        for sc_name, sc_obj in subcomponents.items():
            ports = list(sc_obj.inputs_types.keys()) + list(sc_obj.outputs_types.keys())

            for port in ports:
                key = (port, sc_name)

                if key not in encoding_map:
                    encoding_map[key] = self.tempVarName()

        # Assign any remaining top level port variables to temp names
        tl_ports = list(self.component.inputs_types.keys()) + list(
            self.component.outputs_types.keys()
        )
        for port in tl_ports:
            key = (port, None)

            if key not in encoding_map:
                encoding_map[key] = self.tempVarName()

        # Compute which temporary variables are internal/external.
        input_temp_vars = {
            var
            for key, var in encoding_map.items()
            if key[1] is None and key[0] in self.component.inputs_types
        }
        output_temp_vars = {
            var
            for key, var in encoding_map.items()
            if key[1] is None and key[0] in self.component.outputs_types
        }
        internal_temp_vars = set(encoding_map.values()) - (
            input_temp_vars | output_temp_vars
        )

        # Compute encoding transformer for each subcomponent and the top level component
        # The encoding transformer should encode all port variables to the appropriate
        # temp variable.
        encoding_transformers = {}

        # Subcomponents
        for subcomponent in subcomponents:
            name_map = {}

            for source_info, target_name in encoding_map.items():
                if source_info[1] == subcomponent:
                    name_map[source_info[0]] = target_name

            encoding_transformers[
                self.component.subcomponents[subcomponent]
            ] = NameSwapTransformer(name_map)

        # Top level component
        name_map = {}
        for source_info, target_name in encoding_map.items():
            if source_info[1] == None:
                name_map[source_info[0]] = target_name

        encoding_transformers[self.component] = NameSwapTransformer(name_map)

        # Compute decoding transformer for assumptions and guarantees
        # We need two decoding transformers because temp variables can get mapped to different
        # final variables depending on whether or not they're in an assumption or guarantee
        # (specifically in the case where a variable is passed through a component unchanged).
        assumption_decoding_map = {}
        for port in self.component.inputs_types.keys():
            assumption_decoding_map[encoding_map[((port, None))]] = port
        assumption_decoding_transformer = NameSwapTransformer(assumption_decoding_map)

        guarantee_decoding_map = {}
        for port in self.component.outputs_types.keys():
            guarantee_decoding_map[encoding_map[((port, None))]] = port
        guarantee_decoding_transformer = NameSwapTransformer(guarantee_decoding_map)

        # Move through sub_stmts linearly, checking assumptions and accumulating guarantees
        tl_assumptions = []
        tl_guarantees = []

        for sub_stmt in self.sub_stmts:
            encoding_transformer = encoding_transformers[sub_stmt.component]

            ## Copy and encode assumptions and guarantees ##
            assumptions = [deepcopy(spec) for spec in sub_stmt.assumptions]
            guarantees = [deepcopy(spec) for spec in sub_stmt.guarantees]

            for spec in assumptions + guarantees:
                spec.applyAtomicTransformer(encoding_transformer)

            #### START TODO: REPLACE THIS BLOCK WITH PACTI

            ## Split out purely external assumptions.
            # External assumptions can be moved to the top level contract
            internal_assumptions = []

            for assumption in assumptions:
                names = assumption.getAtomicNames()
                temp_var_names = self.extractTempVars(names)
                if temp_var_names <= input_temp_vars:
                    tl_assumptions.append(assumption)
                else:
                    internal_assumptions.append(assumption)

            ## Attempt to discharge all internal assumptions using accumulated top-level
            ## assumptions and guarantees.
            for assumption in internal_assumptions:
                # TACTIC 1: Discharge if we already have this assumption in our accumulated
                # assumptions and guarantees.
                if any(assumption == spec for spec in tl_assumptions + tl_guarantees):
                    continue

                # We couldn't prove this assumption :(
                ## DEBUG ##
                print("Assumptions:")
                for a in tl_assumptions:
                    print(f"    {a}")

                print("Guarantees:")
                for g in tl_guarantees:
                    print(f"    {g}")
                breakpoint()

            ## Add guarantees to accumulated top-level guarantees
            tl_guarantees += guarantees

            ## Simplify assumptions and guarantees, removing duplicates
            new_tl_assumptions = []
            for spec in tl_assumptions:
                if not any(spec == e_spec for e_spec in new_tl_assumptions):
                    new_tl_assumptions.append(spec)
            tl_assumptions = new_tl_assumptions

            new_tl_guarantees = []
            for spec in tl_guarantees:
                if not any(spec == e_spec for e_spec in new_tl_guarantees) and not any(
                    spec == e_spec for e_spec in new_tl_assumptions
                ):
                    new_tl_guarantees.append(spec)
            tl_guarantees = new_tl_guarantees

            #### END TODO: REPLACE THIS BLOCK WITH PACTI

        ## Decode top level assumptions and guarantees ##
        # If any assumptions are still using internal temporary variable names after decoding, then
        # they are internal assumptions which are are not being satisfied, and this is not a valid
        # IO contract and we should raise an assertion, as PACTI should handle this.
        # If any guarantees are still using internal temporary variable names after decoding, then
        # they're internal guarantees which aren't relevant to our composition and we should raise
        # an assertion, as PACTI should handle this.
        for spec in tl_assumptions:
            spec.applyAtomicTransformer(assumption_decoding_transformer)
        for spec in tl_guarantees:
            spec.applyAtomicTransformer(guarantee_decoding_transformer)

        for spec in tl_assumptions:  # tl_assumptions + tl_guarantees
            assert len(self.extractTempVars(spec.getAtomicNames())) == 0

        # TODO: Replace this with assertion above when PACTI implemented
        tl_guarantees = [
            spec
            for spec in tl_guarantees
            if len(self.extractTempVars(spec.getAtomicNames())) == 0
        ]

        ## Store assumptions and guarantees
        self._assumptions = tl_assumptions
        self._guarantees = tl_guarantees

        # TODO: Clear atomic source syntax strings?

    @cached_property
    def assumptions(self):
        return self._assumptions

    @cached_property
    def guarantees(self):
        return self._guarantees

    def verify(self):
        # Compute sub results and package as evidence
        sub_results = [stmt.verify() for stmt in self.sub_stmts]
        evidence = CompositionEvidence(sub_results)

        # Check what kind of contract result we want
        if any(isinstance(result, ProbabilisticContractResult) for result in sub_results):
            result = ProbabilisticContractResult(
                self.assumptions, self.guarantees, self.component, evidence
            )
        else:
            result = ContractResult(
                self.assumptions, self.guarantees, self.component, evidence
            )

        return result

    def tempVarName(self):
        var_name = f"SCENIC_INTERNAL_VAR_{self.var_num}"
        self.var_num += 1
        return var_name

    @staticmethod
    def extractTempVars(var_iterable):
        return {var for var in var_iterable if var.startswith("SCENIC_INTERNAL_VAR_")}


class CompositionEvidence:
    def __init__(self, sub_results):
        self.sub_results = sub_results

    @cached_property
    def correctness(self):
        correctness_gaps = [1 - result.correctness for result in self.sub_results]
        overall_correctness_gap = sum(correctness_gaps)
        return max(0, 1 - overall_correctness_gap)

    @cached_property
    def confidence(self):
        return prod(result.confidence for result in self.sub_results)

    def __str__(self):
        return "\n".join(str(result) for result in self.sub_results)
