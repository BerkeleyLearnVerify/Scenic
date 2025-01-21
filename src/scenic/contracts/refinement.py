from copy import deepcopy
from functools import cached_property
from pathlib import Path

from scenic.contracts.composition import Composition
from scenic.contracts.contracts import ContractResult, VerificationTechnique
from scenic.syntax.compiler import NameSwapTransformer


class Refinement(VerificationTechnique):
    def __init__(self, stmt, contract, method):
        assert isinstance(stmt, Composition)
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
        ## Extract and standardize intermediate assumptions and guarantees
        i_assumptions = []
        i_guarantees = []
        for sub_stmt in stmt.sub_stmts:
            encoding_transformer = stmt.encoding_transformers[sub_stmt.component]

            ## Copy and encode assumptions and guarantees ##
            assumptions = [deepcopy(spec) for spec in sub_stmt.assumptions]
            guarantees = [deepcopy(spec) for spec in sub_stmt.guarantees]

            for spec in assumptions + guarantees:
                spec.applyAtomicTransformer(encoding_transformer)

            ## Add specs to accumulated i specs
            i_guarantees += guarantees

            ## Simplify assumptions and guarantees, removing duplicates
            new_i_assumptions = []
            for spec in i_assumptions:
                if not any(spec == e_spec for e_spec in new_i_assumptions):
                    new_i_assumptions.append(spec)
            i_assumptions = new_i_assumptions

            new_i_guarantees = []
            for spec in i_guarantees:
                if not (
                    any(spec == e_spec for e_spec in new_i_guarantees)
                    and not any(spec == e_spec for e_spec in new_i_assumptions)
                ):
                    new_i_guarantees.append(spec)
            i_guarantees = new_i_guarantees

        ## Extract and standardize refinement assumptions and guarantees
        tl_assumptions = [deepcopy(spec) for spec in contract.assumptions]
        tl_guarantees = [deepcopy(spec) for spec in contract.guarantees]

        for spec in tl_assumptions + tl_guarantees:
            spec.applyAtomicTransformer(stmt.encoding_transformers[stmt.component])

        ## Extract atomics from all specs, which will form the signals in our trace
        atomics = []
        atomics_types = {}
        for spec in i_assumptions + i_guarantees + tl_assumptions + tl_guarantees:
            for na_name, na_type in spec.getAtomics():
                if all(na_name != a[0] for a in atomics):
                    atomics.append((na_name, na_type))

        prop_atomics = [a for a, _ in filter(lambda x: x[1] is bool, atomics)]
        real_atomics = [a for a, _ in filter(lambda x: x[1] is float, atomics)]

        ## Extract defs from all specs
        defs = {}
        for spec in i_assumptions + i_guarantees + tl_assumptions + tl_guarantees:
            for d_name, d_spec in spec.getDefs():
                if d_name not in defs:
                    defs[d_name] = d_spec
                else:
                    if defs[d_name] != d_spec:
                        # TODO: Handle renames
                        assert False

        print("Atomics:")
        for a in atomics:
            print(f"    {a[0]}: {a[1]}")
        print()

        print("Defs:")
        for d_name, d_spec in defs.items():
            print(f"    {d_name}: {d_spec}")
        print()

        ## Setup proof directory
        Path(self.proof_loc).mkdir(parents=True, exist_ok=True)

        with open(self.proof_loc / "Lib.lean", "w") as f:
            # Imports
            f.write("import Sceniclean.Basic\n\n")

            # TraceState structure
            f.write("structure TraceState where\n")
            f.write("  -- Props\n")
            for i, a in enumerate(prop_atomics):
                f.write(f"  P{i}: Prop\n")
            f.write("  -- Reals\n")
            for i, a in enumerate(real_atomics):
                f.write(f"  N{i}: Real\n")
            f.write("\n")

            # Prop Signals
            f.write("-- Prop Signals\n")
            for i, a in enumerate(prop_atomics):
                f.write(
                    f"  abbrev {a.toLean()} : TraceSet TraceState := TraceSet.of (·.P{i})\n"
                )
            f.write("\n")

            # Real Signals
            f.write("-- Real Signals\n")
            for i, a in enumerate(real_atomics):
                f.write(
                    f"  abbrev {a.toLean()} : TraceSet TraceState := TraceSet.of (·.N{i})\n"
                )
            f.write("\n")

            # Defs
            f.write("-- Defs\n")
            for d_name, d_spec in defs.items():
                f.write(f"abbrev {d_name} := FLTL[{d_spec.toLean()}]\n")
            f.write("\n")

        breakpoint()

        ## Output Lean Library File

        breakpoint()

    def __str__(self):
        return f"LeanProof ('{self.proof_loc}')"
