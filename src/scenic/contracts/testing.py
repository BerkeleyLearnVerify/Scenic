from abc import ABC, abstractmethod
import enum
from functools import cached_property
from pathlib import Path
import time

import rv_ltl
import scipy

from scenic.contracts.contracts import ContractResult, VerificationTechnique
from scenic.contracts.utils import linkSetBehavior, lookuplinkedObject
from scenic.core.distributions import RejectionException
from scenic.core.dynamics import GuardViolation, RejectSimulationException
import scenic.core.object_types
from scenic.core.scenarios import Scenario


class Testing(VerificationTechnique):
    def __init__(
        self,
        ## Testing Specific ##
        confidence,
        batchSize,
        verbose,
        ## Compiler Provided ##
        contract,
        component,
        obj,
        termConditions,
        reqConditions,
    ):
        # Store general args
        self.confidence = confidence
        self.batchSize = batchSize
        self.verbose = verbose
        self.contract = contract
        self.component = component
        self.obj = obj
        self.termConditions = termConditions
        self.reqConditions = reqConditions

    @cached_property
    def assumptions(self):
        return self.contract.assumptions

    @cached_property
    def guarantees(self):
        return self.contract.guarantees

    def verify(self, generateBatchApprox):
        result = self._newContractResult()

        activeTermConditions = (
            self.termConditions if self.termConditions else self.reqConditions
        )

        while not any(
            cond.check(result) for cond in self.termConditions + self.reqConditions
        ):
            result.addTests(self.runTests(self.batchSize, generateBatchApprox))

            if self.verbose:
                print(result)
                print()

        if self.verbose and self.termConditions:
            print("Termination Conditions:")
            for cond in self.termConditions:
                print(f"  {cond} = {cond.check(result)}")
            print()

        # Check requirements
        result.requirementsMet = all(cond.check(result) for cond in self.reqConditions)

        if self.verbose and self.reqConditions:
            print("Requirement Conditions:")
            for cond in self.reqConditions:
                print(f"  {cond} = {cond.check(result)}")
            print()

        return result

    @abstractmethod
    def _newContractResult(self):
        raise NotImplementedError()


class SimulationTesting(Testing):
    def __init__(
        self,
        scenario,
        maxSteps,
        *,
        ## Testing Specific ##
        confidence=0.95,
        batchSize=1,
        verbose=False,
        ## Compiler Provided ##
        contract,
        component,
        obj,
        termConditions,
        reqConditions,
    ):
        super().__init__(
            confidence=confidence,
            batchSize=batchSize,
            verbose=verbose,
            contract=contract,
            component=component,
            obj=obj,
            termConditions=termConditions,
            reqConditions=reqConditions,
        )

        # Store technique specific args
        self.scenario = scenario
        self.maxSteps = maxSteps

        assert len(termConditions) + len(reqConditions) > 0

    def _newContractResult(self):
        return SimulationTestingContractResult(
            self.contract.assumptions,
            self.contract.guarantees,
            self.component,
            self.confidence,
            self.scenario,
        )

    @staticmethod
    def _createTestData(result, violations, scenario, scene, simulation, start_time):
        return SimulationTestData(
            result,
            violations,
            scenario.sceneToBytes(scene),
            simulation.getReplay(),
            time.time() - start_time,
        )

    def runTests(self, num_tests, generateBatchApprox):
        # Generate scenes
        scenes = generateBatchApprox(scenario=self.scenario, numScenes=num_tests)

        # Evaluate each scene
        tests = []
        for scene in scenes:
            tests.append(self.testScene(scene))

        return tests

    def testScene(self, scene):
        start_time = time.time()

        # Link object
        linkSetBehavior(scene, [self.obj])

        ## Create and link EagerValueWindows ##
        base_value_windows = {}
        # Objects
        assert len(self.contract.objects) == 1
        object_name = self.contract.objects[0]
        obj_ptr = lookuplinkedObject(scene, self.component.linkedObjectName)
        base_value_windows[object_name] = EagerValueWindow((lambda: obj_ptr))

        # Globals
        for global_name in self.contract.globals:
            if global_name == "objects":
                objects_ptr = scene.objects
                base_value_windows[global_name] = EagerValueWindow((lambda: objects_ptr))
            elif global_name == "WORKSPACE":
                workspace_ptr = scene.workspace
                base_value_windows[global_name] = EagerValueWindow(
                    (lambda: workspace_ptr)
                )
            elif global_name == "params":
                params_ptr = scene.params
                base_value_windows[global_name] = EagerValueWindow((lambda: params_ptr))
            else:
                raise ValueError(f"Unrecognized global value '{global_name}'")

        # Inputs
        for input_name in self.contract.inputs_types.keys():
            base_value_windows[input_name] = EagerValueWindow(
                (lambda: self.component.last_inputs[input_name])
            )

        # Outputs
        for output_name in self.contract.outputs_types.keys():
            base_value_windows[output_name] = EagerValueWindow(
                (lambda: self.component.last_outputs[output_name])
            )

        value_windows = {**base_value_windows}

        # Definitions
        for def_name, def_lambda in self.contract.definitions.items():
            def_closure = lambda l, x: lambda t: l(t, *x.values())
            value_windows[def_name] = LazyValueWindow(
                def_closure(def_lambda, value_windows.copy())
            )

        ## Create Monitors for Assumptions/Guarantees
        assumptions_monitors = [
            a.create_monitor() for a in self.contract.assumptions_props
        ]
        assumptions_values = [
            rv_ltl.B4.PRESUMABLY_TRUE for a in self.contract.assumptions_props
        ]
        guarantees_monitors = [g.create_monitor() for g in self.contract.guarantees_props]
        guarantees_values = [
            rv_ltl.B4.PRESUMABLY_TRUE for g in self.contract.guarantees_props
        ]

        ## Evaluate Contract on Simulation ##
        sim_step = 0
        eval_step = 0

        # Instantiate simulator
        simulator = self.scenario.getSimulator()

        def vw_update_hook():
            for vw_name, vw in base_value_windows.items():
                vw.update()

        # Step contract till termination
        with simulator.simulateStepped(scene, maxSteps=self.maxSteps) as simulation:
            while not simulation.result or eval_step < sim_step:
                # If simulation not terminated, advance simulation one time step, catching any rejections
                if not simulation.result:
                    try:
                        simulation.advance(vw_update_hook)
                    except (
                        RejectSimulationException,
                        RejectionException,
                        GuardViolation,
                    ) as e:
                        return self._createTestData(
                            TestResult.R, [], self.scenario, scene, simulation, start_time
                        )

                    # If the simulation finished, no need to update value windows.
                    if simulation.result:
                        continue

                    # If the simulation didn't finish, update all base value windows

                    # Increment simulation step
                    sim_step += 1

                    # If the sim_step isn't sufficiently ahead of eval_step, hold off on assumption/guarantee
                    # evaluation for now.
                    if sim_step - eval_step <= self.contract.max_lookahead:
                        continue

                # Check all assumptions
                prop_params = [eval_step] + list(value_windows.values())
                for a_iter, assumption in enumerate(assumptions_monitors):
                    try:
                        a_val = assumption.update(prop_params)
                        assumptions_values[a_iter] = a_val
                    except InvalidTimeException as e:
                        if e.time < sim_step and simulation.result:
                            raise

                # If we've definitely violated an assumption, we can terminate early.
                violated_assumptions = [
                    ai
                    for ai, av in enumerate(assumptions_values)
                    if av == rv_ltl.B4.FALSE
                ]

                if violated_assumptions and False:
                    return self._createTestData(
                        TestResult.A,
                        violated_assumptions,
                        self.scenario,
                        scene,
                        simulation,
                        start_time,
                    )

                # Check all guarantees
                for g_iter, guarantee in enumerate(guarantees_monitors):
                    try:
                        r_val = guarantee.update(prop_params)
                        guarantees_values[g_iter] = r_val
                    except InvalidTimeException as e:
                        if e.time < sim_step and simulation.result:
                            raise

                # Increment evaluation step
                eval_step += 1

        # Check final status assumptions and guarantees
        violated_assumptions = [
            ai
            for ai, av in enumerate(assumptions_values)
            if av == rv_ltl.B4.PRESUMABLY_FALSE or av == rv_ltl.B4.FALSE
        ]
        if violated_assumptions:
            return self._createTestData(
                TestResult.A,
                violated_assumptions,
                self.scenario,
                scene,
                simulation,
                start_time,
            )

        violated_guarantees = [
            gi
            for gi, gv in enumerate(guarantees_values)
            if gv == rv_ltl.B4.PRESUMABLY_FALSE or gv == rv_ltl.B4.FALSE
        ]
        if len(violated_guarantees) > 0:
            return self._createTestData(
                TestResult.G,
                violated_guarantees,
                self.scenario,
                scene,
                simulation,
                start_time,
            )
        else:
            return self._createTestData(
                TestResult.V, [], self.scenario, scene, simulation, start_time
            )


class EagerValueWindow:
    def __init__(self, get_val):
        self._elapsed_time = 0
        self.get_val = get_val
        self.window = []

    def update(self):
        new_val = self.get_val()

        # Try to replace objects with their concrete proxies.
        # TODO: Find a better approach for this?
        if isinstance(new_val, scenic.core.object_types.Object):
            new_val = new_val._copyWith()
        elif isinstance(new_val, list):
            new_val = [
                v._copyWith() if isinstance(v, scenic.core.object_types.Object) else v
                for v in new_val
            ]
        elif isinstance(new_val, tuple):
            new_val = tuple(
                v._copyWith() if isinstance(v, scenic.core.object_types.Object) else v
                for v in new_val
            )

        self._elapsed_time += 1
        self.window.append(new_val)

    def __getitem__(self, time):
        if not time < self._elapsed_time:
            raise InvalidTimeException(time)

        return self.window[time]


class LazyValueWindow:
    def __init__(self, get_val):
        self.get_val = get_val
        self.window = {}

    def update(self, time):
        self.window[time] = self.get_val(time)

    def __getitem__(self, time):
        if time not in self.window:
            self.update(time)

        return self.window[time]


class InvalidTimeException(Exception):
    def __init__(self, time):
        self.time = time


## Termination/Requirement Conditions
class Condition(ABC):
    @abstractmethod
    def check(self, evidence):
        raise NotImplementedError()

    def __str__(self):
        return repr(self)


class TimeTerminationCondition(Condition):
    def __init__(self, timeout):
        self.timeout = timeout

    def check(self, evidence):
        return evidence.elapsed_time >= self.timeout

    def __repr__(self):
        return f"{self.__class__.__name__}({self.timeout})"


class CountTerminationCondition(Condition):
    def __init__(self, count):
        self.count = count

    def check(self, evidence):
        return len(evidence) >= self.count

    def __repr__(self):
        return f"{self.__class__.__name__}({self.count})"


class GapTerminationCondition(Condition):
    def __init__(self, gap):
        self.gap = gap

    def check(self, evidence):
        return evidence.confidenceGap <= self.gap

    def __repr__(self):
        return f"{self.__class__.__name__}({self.gap})"


class CorrectnessRequirementCondition(Condition):
    def __init__(self, correctness):
        self.correctness = correctness

    def check(self, evidence):
        return evidence.correctness >= self.correctness

    def __repr__(self):
        return f"{self.__class__.__name__}({self.correctness})"


## Contract Results and Test Data Classes
class TestingContractResult(ContractResult):
    def __init__(self, assumptions, guarantees, component, confidence):
        super().__init__(assumptions, guarantees, component)
        self._confidence = confidence
        self.testData = []
        self.requirementsMet = None

        # Initialize metrics
        self.elapsed_time = 0
        self.v_count = 0
        self.r_count = 0
        self.a_count = 0
        self.g_count = 0

    def addTests(self, newTests):
        # Update metrics
        self.elapsed_time += sum(t.elapsed_time for t in newTests)
        self.v_count += len(list(filter(lambda t: t.result == TestResult.V, newTests)))
        self.r_count += len(list(filter(lambda t: t.result == TestResult.R, newTests)))
        self.a_count += len(list(filter(lambda t: t.result == TestResult.A, newTests)))
        self.g_count += len(list(filter(lambda t: t.result == TestResult.G, newTests)))

        # Add new tests
        self.testData += newTests

    @property
    def correctness(self):
        if self.v_count + self.g_count == 0:
            return 0

        bt = scipy.stats.binomtest(
            k=self.v_count, n=self.v_count + self.g_count, alternative="greater"
        )
        ci = bt.proportion_ci(confidence_level=self.confidence)
        return ci.low

    @property
    def confidence(self):
        return self._confidence

    @property
    def confidenceGap(self):
        if len(self) == 0 or (self.v_count + self.g_count) == 0:
            return 1

        bt = scipy.stats.binomtest(k=self.v_count, n=self.v_count + self.g_count)
        ci = bt.proportion_ci(confidence_level=self.confidence)
        return ci.high - ci.low

    @property
    def meanCorrectness(self):
        if self.v_count + self.g_count == 0:
            return float("nan")

        return self.v_count / (self.v_count + self.g_count)

    @property
    @abstractmethod
    def _source_info(self):
        pass

    def __len__(self):
        return len(self.testData)

    def __iter__(self):
        return self.testData

    @property
    def assumptionsSummary(self):
        string = ""
        for ai, a in enumerate(self.assumptions):
            if self.a_count == 0:
                percent_violated = 0
            else:
                percent_violated = (
                    sum(
                        1 / len(at.violations)
                        for at in self.testData
                        if at.result == TestResult.A and ai in at.violations
                    )
                    / self.a_count
                )

            string += f"    ({percent_violated*100:6.2f}%) {a}\n"

        return string

    @property
    def guaranteesSummary(self):
        string = ""
        for gi, g in enumerate(self.guarantees):
            if self.g_count == 0:
                percent_violated = 0
            else:
                percent_violated = (
                    sum(
                        1 / len(gt.violations)
                        for gt in self.testData
                        if gt.result == TestResult.G and gi in gt.violations
                    )
                    / self.g_count
                )

            string += f"    ({percent_violated*100:6.2f}%) {g}\n"

        return string

    @property
    def evidenceSummary(self):
        string = (
            f"Simulation-Based Testing\n"
            f"Sampled from {self._source_info}\n"
            f"{self.v_count} Verified,  {self.r_count} Rejected,  "
            f"{self.a_count} A-Violated,  {self.g_count} G-Violated\n"
            f"{len(self.testData)} Samples, {self.elapsed_time:.2f} Seconds\n"
            f"Mean Correctness: {100*self.meanCorrectness:.2f}%\n"
            f"Confidence Gap: {self.confidenceGap:.4f}"
        )
        return string


class SimulationTestingContractResult(TestingContractResult):
    def __init__(self, assumptions, guarantees, component, confidence, source):
        # Validate and store source
        if not (isinstance(source, Scenario) or source is None):
            raise ValueError("SimulationEvidence must have a Scenario object as a source")

        self.source = source

        super().__init__(assumptions, guarantees, component, confidence)

    def addTests(self, newTests):
        if any(not isinstance(t, SimulationTestData) for t in newTests):
            raise ValueError(
                "SimulationEvidence can only accept tests of class SimulationTestData"
            )

        super().addTests(newTests)

    @property
    def _source_info(self):
        return (f"Scenario '{Path(self.source.filename).name if self.source else 'NONE'}"
                f" (Hash={int.from_bytes(self.source.astHash) if self.source else 'NONE'})")

@enum.unique
class TestResult(enum.Enum):
    V = "Valid: The contract was successfully validated"
    R = "Rejected: The scenario was rejected or a guard was violated"
    A = "Assumptions: An assumption was violated"
    G = "Guarantees: A guarantee was violated"


class TestData:
    def __init__(self, result, violations, elapsed_time):
        self.result = result
        self.violations = violations
        self.elapsed_time = elapsed_time


class SimulationTestData(TestData):
    def __init__(self, result, violations, scene_bytes, sim_replay, elapsed_time):
        super().__init__(result, violations, elapsed_time)
        self.scene_bytes = scene_bytes
        self.sim_replay = sim_replay
