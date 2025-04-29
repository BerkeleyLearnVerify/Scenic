from copy import deepcopy
from functools import cached_property
import json
import os
from pathlib import Path
import subprocess

from scenic.contracts.composition import Composition
from scenic.contracts.contracts import ContractResult, VerificationTechnique
from scenic.syntax.compiler import NameSwapTransformer


class Refinement(VerificationTechnique):
    def __init__(self, stmt, contract, method):
        self.stmt = stmt
        self.component = self.stmt.component
        self.contract = contract
        self.method = method

    @cached_property
    def assumptions(self):
        return self.contract.assumptions

    @cached_property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self, generateBatchApprox):
        self.method.check(self.stmt, self.contract)

        return RefinementContractResult(
            self.assumptions,
            self.guarantees,
            self.stmt.component,
            self.stmt.verify(generateBatchApprox),
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
    def __init__(self, proof_path, proof_loc, repl_loc):
        self.proof_path = Path(proof_path)
        self.proof_loc = Path(proof_loc)
        self.proof_dir = self.proof_loc / self.proof_path
        self.repl_loc = Path(repl_loc)

    def check(self, stmt, contract):
        ## Extract and standardize internal assumptions and guarantees
        if isinstance(stmt, Composition):
            i_assumptions = []
            i_guarantees = []
            for sub_stmt in stmt.sub_stmts:
                ## Copy and encode assumptions and guarantees ##
                assumptions = [deepcopy(spec) for spec in sub_stmt.assumptions]
                guarantees = [deepcopy(spec) for spec in sub_stmt.guarantees]

                encoding_transformer = stmt.encoding_transformers[sub_stmt.component]
                for spec in assumptions + guarantees:
                    spec.applyAtomicTransformer(encoding_transformer)

                ## Add specs to accumulated i specs
                i_assumptions += assumptions
                i_guarantees += guarantees
        else:
            i_assumptions = [deepcopy(spec) for spec in stmt.assumptions]
            i_guarantees = [deepcopy(spec) for spec in stmt.guarantees]

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

        if isinstance(stmt, Composition):
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

        # Sort atomics lists to keep signal names from swapping
        prop_atomics.sort(key=str)
        real_atomics.sort(key=str)

        ## Extract defs from all specs
        defs = {}
        for spec in i_assumptions + i_guarantees + tl_assumptions + tl_guarantees:
            raw_new_defs = spec.getDefs()

            for d_name, d_spec in [
                d for _, d in sorted(raw_new_defs, key=lambda x: x[0])
            ]:
                if d_name not in defs:
                    defs[d_name] = d_spec
                else:
                    if defs[d_name] != d_spec:
                        # TODO: Handle renames
                        assert False

        ## Setup proof directory
        Path(self.proof_dir).mkdir(
            parents=True, exist_ok=True
        )

        ## Output Lean Data File
        with open(self.proof_dir / "Lib.lean", "w") as f:
            # Imports
            f.write("import LeanLTL\n\n")
            f.write("open LeanLTL\nopen scoped LeanLTL.Notation\n\n")
            f.write(f"namespace {self.proof_dir.parts[-1]}\n\n")

            # TraceState structure
            f.write("structure TraceState where\n")
            f.write("  -- Props\n")
            for i, a in enumerate(prop_atomics):
                f.write(f"  P{i}: Prop\n")
            f.write("  -- Numbers\n")
            for i, a in enumerate(real_atomics):
                f.write(f"  N{i}: ℚ\n")
            f.write("deriving Inhabited\n")
            f.write("\n")

            # Prop Signals
            f.write("-- Prop Signals\n")
            for i, a in enumerate(prop_atomics):
                f.write(
                    f"abbrev {a.toLeanName()} : TraceSet TraceState := TraceSet.of (·.P{i})\n"
                )
            f.write("\n")

            # Numerical Signals
            f.write("-- Numerical Signals\n")
            for i, a in enumerate(real_atomics):
                f.write(
                    f"abbrev {a.toLeanName()} : TraceFun TraceState ℚ := TraceFun.of (·.N{i})\n"
                )
            f.write("\n")

            # Defs
            f.write("-- Defs\n")
            for d_name, d_spec in defs.items():
                f.write(f"abbrev {d_name} := LLTLV[{d_spec.toLean(includeGets=False)}]\n")
            f.write("\n")

            TOP = "\u22a4"

            # Top Level Assumptions
            f.write("-- Top Level Assumptions \n")
            for i, a in enumerate(tl_assumptions):
                f.write(f"abbrev A{i} := LLTL[{a.toLean()}]\n")
            f.write("\n")
            f.write(
                f"abbrev assumptions : TraceSet TraceState := LLTL[{' ∧ '.join(f'A{i}' for i in range(len(tl_assumptions))) if tl_assumptions else TOP}]\n"
            )
            f.write("\n")

            # Internal Assumptions
            f.write("-- Internal Assumptions \n")
            for i, a in enumerate(i_assumptions):
                f.write(f"abbrev IA{i} := LLTL[{a.toLean()}]\n")
            f.write("\n")
            f.write(
                f"abbrev i_assumptions : TraceSet TraceState := LLTL[{' ∧ '.join(f'IA{i}' for i in range(len(i_assumptions))) if i_assumptions else TOP}]\n"
            )
            f.write("\n")

            # Internal Guarantees
            f.write("-- Internal Guarantees \n")
            for i, g in enumerate(i_guarantees):
                f.write(f"abbrev IG{i} := LLTL[{g.toLean()}]\n")
            f.write("\n")
            f.write(
                f"abbrev i_guarantees : TraceSet TraceState := LLTL[{' ∧ '.join(f'IG{i}' for i in range(len(i_guarantees))) if i_guarantees else TOP}]\n"
            )
            f.write("\n")

            # Guarantees
            f.write("-- Top Level Guarantees \n")
            for i, g in enumerate(tl_guarantees):
                f.write(f"abbrev G{i} := LLTL[{g.toLean()}]\n")
            f.write("\n")
            f.write(
                f"abbrev guarantees : TraceSet TraceState := LLTL[{' ∧ '.join(f'G{i}' for i in range(len(tl_guarantees))) if tl_guarantees else TOP}]\n"
            )
            f.write("\n")

        ## Output Lean Proof Files
        if not os.path.exists(
            self.proof_dir / "AssumptionProof.lean"
        ):
            with open(
                self.proof_dir / "AssumptionProof.lean",
                "w",
            ) as f:
                f.write(f"import {'.'.join(self.proof_path.parts)}.Lib\n\n")
                f.write("open LeanLTL\nopen scoped LeanLTL.Notation\n\n")
                f.write(f"namespace {self.proof_dir.parts[-1]}\n\n")

                f.write("\n")
                f.write(
                    "theorem imp_assumptions : LLTL[(assumptions)] ⇒ LLTL[i_assumptions] := by\n"
                )
                f.write("  sorry\n")

        if not os.path.exists(
            self.proof_dir / "GuaranteesProof.lean"
        ):
            with open(
                self.proof_dir / "GuaranteesProof.lean",
                "w",
            ) as f:
                f.write(f"import {'.'.join(self.proof_path.parts)}.Lib\n\n")
                f.write("open LeanLTL\nopen scoped LeanLTL.Notation\n\n")
                f.write(f"namespace {self.proof_dir.parts[-1]}\n\n")

                f.write("\n")
                f.write(
                    "theorem imp_guarantees : LLTL[(assumptions ∧ i_guarantees)] ⇒ LLTL[guarantees] := by\n"
                )
                f.write("  sorry\n")

        # Check validity of proofs
        assert self.checkProof(
            self.proof_dir / "AssumptionProof.lean"
        )
        assert self.checkProof(
            self.proof_dir / "GuaranteesProof.lean"
        )

    def checkProof(self, file):
        subprocess.run(
            ["lake", "build"],
            capture_output=True,
            text=True,
            cwd=str(self.proof_loc)
        )
        input_cmd = '{"path": "' + str(file) + '", "allTactics": false}'
        repl_loc_full = str(self.repl_loc) + "/.lake/build/bin/repl"
        result = subprocess.run(
            ["lake", "env", repl_loc_full],
            input=input_cmd,
            capture_output=True,
            text=True,
            cwd=str(self.proof_loc),
        )
        result_dict = json.loads(result.stdout)

        assert result_dict["env"] == 0

        for msg in result_dict.get("messages", []):
            assert msg["severity"] != "error", f"Error in Lean proof: {file}"

        assert "sorries" not in result_dict, "Sorry used in Lean proof"

        return True

    def __str__(self):
        return f"LeanProof: ('{self.proof_loc}')"
