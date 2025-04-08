import ast
from copy import copy, deepcopy
from functools import cached_property, reduce
import itertools
from math import prod

from pacti.contracts import PropositionalIoContract

from scenic.contracts.components import ActionComponent, BaseComponent, ComposeComponent
from scenic.contracts.contracts import ContractResult, VerificationTechnique
import scenic.contracts.specifications as specs
from scenic.core.distributions import Options, Range
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
        environment,
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
        self.encoding_map = {}
        self.var_num = 0

        # Assign variables to all connections
        for source, dest in connections:
            # Check if we already have a temporary variable name and if not
            # come up with a new intermediate variable name.
            temp_var_name = (
                self.encoding_map[source]
                if source in self.encoding_map
                else self.tempVarName()
            )

            # Map both variables to the new name
            self.encoding_map[source] = temp_var_name
            self.encoding_map[dest] = temp_var_name

        # Assign any remaining subcomponent port variables to temp names
        for sc_name, sc_obj in subcomponents.items():
            ports = list(sc_obj.inputs_types.keys()) + list(sc_obj.outputs_types.keys())

            for port in ports:
                key = (port, sc_name)

                if key not in self.encoding_map:
                    self.encoding_map[key] = self.tempVarName()

        # Assign any remaining top level port variables to temp names
        tl_ports = list(self.component.inputs_types.keys()) + list(
            self.component.outputs_types.keys()
        )
        for port in tl_ports:
            key = (port, None)

            if key not in self.encoding_map:
                self.encoding_map[key] = self.tempVarName()

        # Compute which temporary variables are internal/external.
        input_temp_vars = {
            var
            for key, var in self.encoding_map.items()
            if key[1] is None and key[0] in self.component.inputs_types
        }
        output_temp_vars = {
            var
            for key, var in self.encoding_map.items()
            if key[1] is None and key[0] in self.component.outputs_types
        }
        internal_temp_vars = set(self.encoding_map.values()) - (
            input_temp_vars | output_temp_vars
        )

        # Compute encoding transformer for each subcomponent and the top level component
        # The encoding transformer should encode all port variables to the appropriate
        # temp variable.
        self.encoding_transformers = {}

        # Subcomponents
        for subcomponent in subcomponents:
            name_map = {}

            for source_info, target_name in self.encoding_map.items():
                if source_info[1] == subcomponent:
                    name_map[source_info[0]] = target_name

            self.encoding_transformers[
                self.component.subcomponents[subcomponent]
            ] = NameSwapTransformer(name_map)

        # Top level component
        name_map = {}
        for source_info, target_name in self.encoding_map.items():
            if source_info[1] == None:
                name_map[source_info[0]] = target_name

        self.encoding_transformers[self.component] = NameSwapTransformer(name_map)

        # Compute decoding transformer for assumptions and guarantees
        # We need two decoding transformers because temp variables can get mapped to different
        # final variables depending on whether or not they're in an assumption or guarantee
        # (specifically in the case where a variable is passed through a component unchanged).
        assumption_decoding_map = {}
        for port in self.component.inputs_types.keys():
            assumption_decoding_map[self.encoding_map[((port, None))]] = port
        assumption_decoding_transformer = NameSwapTransformer(assumption_decoding_map)

        guarantee_decoding_map = {}
        for port in itertools.chain(
            self.component.outputs_types.keys(), self.component.inputs_types.keys()
        ):
            guarantee_decoding_map[self.encoding_map[((port, None))]] = port
        guarantee_decoding_transformer = NameSwapTransformer(guarantee_decoding_map)

        # Initialize syntaxMappings
        syntaxMappings = {}

        # Convert all sub_stmts to PACTI contracts and order/cluster according to sub-component
        stmt_groups = [[] for _ in range(len(self.sub_stmts))]
        stmt_group_loc = {stmt: pos for pos, stmt in enumerate(self.sub_stmts)}

        for sub_stmt in self.sub_stmts:
            encoding_transformer = self.encoding_transformers[sub_stmt.component]

            ## Copy and encode assumptions and guarantees ##
            assumptions = [deepcopy(spec) for spec in sub_stmt.assumptions]
            guarantees = [deepcopy(spec) for spec in sub_stmt.guarantees]

            for spec in assumptions + guarantees:
                spec.applyAtomicTransformer(encoding_transformer)

            # Convert all assumptions and guarantees to PACTI-compatible strings
            pstring_assumptions = [a.toPACTIStr(syntaxMappings) for a in assumptions]
            pstring_guarantees = [g.toPACTIStr(syntaxMappings) for g in guarantees]

            # Encode IO variables, create PACTI contract, and store it in the appropriate group
            encoded_input_vars = [
                encoding_transformer.name_map[i]
                for i in sub_stmt.component.inputs_types.keys()
            ]
            encoded_output_vars = [
                encoding_transformer.name_map[o]
                for o in sub_stmt.component.outputs_types.keys()
            ]

            pacti_contract = PropositionalIoContract.from_strings(
                input_vars=encoded_input_vars,
                output_vars=encoded_output_vars,
                assumptions=pstring_assumptions,
                guarantees=pstring_guarantees,
            )

            stmt_groups[stmt_group_loc[sub_stmt]].append(pacti_contract)

        # Merge all groups, then compose in order.
        merged_contracts = [
            reduce(lambda x, y: x.merge(y), group) for group in stmt_groups
        ]
        composed_contract = reduce(lambda x, y: x.compose(y), merged_contracts)

        breakpoint()
        pass

        """
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
            assert False
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
        """

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
        valid_guarantees = [
            spec
            for spec in tl_guarantees
            if len(self.extractTempVars(spec.getAtomicNames())) == 0
        ]

        ## TODO: Deprecate below with PACTI integration
        ## Try to salvage other guarantees
        dropped_guarantees = [
            spec
            for spec in tl_guarantees
            if len(self.extractTempVars(spec.getAtomicNames())) != 0
        ]

        while dropped_guarantees:
            g1 = dropped_guarantees.pop(0)

            if isinstance(g1, specs.Always) and isinstance(g1.sub, specs.Implies):
                for g2 in filter(
                    lambda x: isinstance(x, specs.Always)
                    and isinstance(x.sub, specs.Implies),
                    dropped_guarantees,
                ):
                    g1_a, g1_b = g1.sub.sub1, g1.sub.sub2
                    g2_a, g2_b = g2.sub.sub1, g2.sub.sub2

                    if g1_b == g2_a:
                        new_g = specs.Always(specs.Implies(g1_a, g2_b))

                        if len(self.extractTempVars(new_g.getAtomicNames())) == 0:
                            valid_guarantees.append(new_g)

        ## Store assumptions and guarantees
        self._assumptions = tl_assumptions
        self._guarantees = valid_guarantees

        # TODO: Clear atomic source syntax strings?

    @cached_property
    def assumptions(self):
        return self._assumptions

    @cached_property
    def guarantees(self):
        return self._guarantees

    def verify(self):
        sub_results = [stmt.verify() for stmt in self.sub_stmts]
        return CompositionContractResult(
            self.assumptions, self.guarantees, self.component, sub_results
        )

    def tempVarName(self):
        var_name = f"SCENIC_INTERNAL_VAR_{self.var_num}"
        self.var_num += 1
        return var_name

    @staticmethod
    def isTempVar(var):
        return var.startswith("SCENIC_INTERNAL_VAR_")

    @staticmethod
    def extractTempVars(var_iterable):
        return {var for var in var_iterable if Composition.isTempVar(var)}


class CompositionContractResult(ContractResult):
    def __init__(self, assumptions, guarantees, component, sub_results):
        super().__init__(assumptions, guarantees, component)
        self.sub_results = sub_results

    @cached_property
    def correctness(self):
        correctness_gaps = [1 - result.correctness for result in self.sub_results]
        overall_correctness_gap = sum(correctness_gaps)
        return max(0, 1 - overall_correctness_gap)

    @cached_property
    def confidence(self):
        return prod(result.confidence for result in self.sub_results)

    @property
    def evidenceSummary(self):
        return "\n".join(str(result) for result in self.sub_results)


class Merge(VerificationTechnique):
    def __init__(self, sub_stmts, component, environment):
        ## Initialization ##
        # Store parameters
        self.component = component
        self.sub_stmts = sub_stmts
        self.environment = environment

        # Ensure all sub statements are of a valid form
        for stmt in self.sub_stmts:
            assert isinstance(stmt, VerificationTechnique)

        # Check that all sub-statements are applied to this component
        for stmt in self.sub_stmts:
            assert stmt.component is self.component

        # TODO: Handle more than two statements
        assert len(self.sub_stmts) == 2

        assert (
            sub_stmts[0].guarantees == sub_stmts[1].guarantees
        ), "Merged guarantees are not equivalent"

        assumptions_1 = copy(sub_stmts[0].assumptions)
        assumptions_2 = copy(sub_stmts[1].assumptions)

        def standardize_spec(spec):
            if isinstance(spec, specs.Not):
                return (True, spec.sub)

            return (False, spec)

        split_specs = None

        for a1, a2 in itertools.product(assumptions_1, assumptions_2):
            sa1 = standardize_spec(a1)
            sa2 = standardize_spec(a2)

            if (sa1[0] != sa2[0]) and (sa1[1] == sa2[1]):
                # Found a split
                split_specs = (a1, a2)
                break

        assert (
            split_specs is not None
        ), "Couldn't find a pair of assumptions that split the space"

        # Determine probability of each side of the split
        prob1, _ = self.getSpecProb(split_specs[0])
        prob2, _ = self.getSpecProb(split_specs[1])
        assert (prob1 is not None) or (prob2 is not None)
        assert prob1 + prob2 == 1.0

        self.result_weights = (prob1, prob2)
        self.guarantees = deepcopy(sub_stmts[0].guarantees)

        assumptions_1.remove(split_specs[0])
        assumptions_2.remove(split_specs[1])

        assert assumptions_1 == assumptions_2

        self.assumptions = deepcopy(assumptions_1)

    # TODO: Better handling for this. Right now we have a pretty brittle way of asserting
    # probs are computed properly.
    def getSpecProb(self, spec):
        def extractConstant(spec):
            if isinstance(spec, specs.ConstantSpecNode):
                return spec.value

        def extractDist(spec):
            if (
                isinstance(spec, specs.Atomic)
                and isinstance(spec.ast, ast.Subscript)
                and isinstance(spec.ast.value, ast.Name)
                and spec.ast.value.id == "params"
            ):
                assert isinstance(spec.ast.slice.value, str)
                param_name = spec.ast.slice.value

                return param_name, self.environment.params[param_name]

        if isinstance(spec, specs.Not):
            sub_call = self.getSpecProb(spec.sub)
            return 1 - sub_call[0], sub_call[1]
        elif isinstance(spec, specs.And):
            sub_calls = [self.getSpecProb(sub) for sub in spec.subs]
            deps = list(itertools.chain(*[sub[1] for sub in sub_calls]))
            assert len(deps) == len(set(deps))
            return prod(sub[0] for sub in sub_calls), set(deps)
        elif isinstance(spec, specs.Or):
            sub_calls = [self.getSpecProb(sub) for sub in spec.subs]
            deps = set(itertools.chain(*[sub[1] for sub in sub_calls]))
            assert set(sub_calls[0][1]) == deps
            return 1 - prod((1 - sub[0]) for sub in sub_calls), deps
        elif isinstance(spec, specs.Equal) or isinstance(spec, specs.GE):
            if (extractConstant(spec.sub1) is not None) and (
                extractDist(spec.sub2) is not None
            ):
                val = extractConstant(spec.sub1)
                dist = extractDist(spec.sub2)
            elif (extractConstant(spec.sub2) is not None) and (
                extractDist(spec.sub1) is not None
            ):
                val = extractConstant(spec.sub2)
                dist = extractDist(spec.sub1)
            else:
                assert False

            deps, dist = dist

            if isinstance(dist, Options):
                assert isinstance(spec, specs.Equal)
                for opt, weight in dist.optWeights.items():
                    if opt == val:
                        return weight, (deps,)
                assert False
            elif isinstance(dist, Range):
                assert isinstance(spec, specs.GE)
                if dist.high <= val:
                    return 0, (deps,)
                else:
                    prob = (dist.high - val) / (dist.high - dist.low)
                    return prob, (deps,)

        breakpoint()

        assert False

    @cached_property
    def assumptions(self):
        return self._assumptions

    @cached_property
    def guarantees(self):
        return self._guarantees

    def verify(self):
        sub_results = [stmt.verify() for stmt in self.sub_stmts]
        return MergeContractResult(
            self.assumptions,
            self.guarantees,
            self.component,
            sub_results,
            self.result_weights,
        )


class MergeContractResult(ContractResult):
    def __init__(self, assumptions, guarantees, component, sub_results, result_weights):
        super().__init__(assumptions, guarantees, component)
        self.sub_results = sub_results
        self.result_weights = result_weights

    @cached_property
    def correctness(self):
        overall_correctness = 0
        for i, stmt in enumerate(self.sub_results):
            weight = self.result_weights[i]
            overall_correctness += stmt.correctness * weight
        return overall_correctness

    @cached_property
    def confidence(self):
        return prod(result.confidence for result in self.sub_results)

    @property
    def evidenceSummary(self):
        summary = "Result Merge:\n"
        for i, result in enumerate(self.sub_results):
            summary += f"Sub Result (Weight={self.result_weights[i]:.2f}, Sub-correctness={result.correctness*self.result_weights[i]:.2f}):\n{result}\n"
        return summary
