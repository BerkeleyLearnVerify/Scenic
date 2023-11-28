from abc import ABC, abstractmethod
import enum
import time

import rv_ltl
import scipy

from scenic.contracts.contracts import ContractEvidence, ContractResult
from scenic.contracts.utils import linkSetBehavior, lookuplinkedObject
from scenic.core.distributions import RejectionException
from scenic.core.dynamics import GuardViolation, RejectSimulationException


class SimulationTesting:
    def __init__(
        self,
        scenario,
        maxSteps,
        batchSize=1,
        verbose=False,
        *,
        confidence,
        contract,
        component,
        obj,
        termConditions,
        reqConditions,
    ):
        # Store technique specific args
        self.scenario = scenario
        self.maxSteps = maxSteps
        self.batchSize = batchSize
        self.verbose = verbose

        # Store general args
        self.confidence = confidence
        self.contract = contract
        self.component = component
        self.obj = obj
        self.termConditions = termConditions
        self.reqConditions = reqConditions

        assert len(termConditions) + len(reqConditions) > 0

    def verify(self):
        self.evidence = ProbabilisticEvidence(confidence=self.confidence)

        while not any(cond.check(self.evidence) for cond in self.termConditions):
            self.evidence.addTests(self.runTests(self.batchSize))

            if self.verbose:
                print(f"{len(self.evidence)} Tests Accumulated")
                print(f"Elaped Time: {self.evidence.elapsed_time}")
                print(
                    f"{self.evidence.v_count} Verified,  {self.evidence.r_count} Rejected,  "
                    f"{self.evidence.a_count} A-Violated,  {self.evidence.g_count} G-Violated"
                )
                print(
                    f"Mean Correctness: {self.evidence.v_count/(self.evidence.v_count+self.evidence.g_count)*100:.2f}%"
                )
                print(f"Confidence Gap: {self.evidence.confidenceGap}")
                print(
                    f"Correctness Guarantee (Low): {self.evidence.correctness*100:.2f}%"
                )
                print()

        if self.verbose:
            print("Terminating:")
            for cond in self.termConditions:
                print(f"  {cond} = {cond.check(self.evidence)}")

        # Check requirements
        self.evidence.requirementsMet = all(
            cond.check(self.evidence) for cond in self.reqConditions
        )

        # Package contract result and return
        result = ContractResult(
            self.contract.assumptions, self.contract.guarantees, self.evidence
        )
        return result

    def runTests(self, num_tests):
        # Generate scenes
        scenes, _ = self.scenario.generateBatch(numScenes=num_tests)

        # Evaluate each scene
        tests = []
        for scene in scenes:
            tests.append(self.testScene(scene))

        return tests

    def testScene(self, scene):
        start_time = time.time()

        # Link object
        assert self.obj
        linkSetBehavior(scene, [self.obj])

        ## Create and link ValueWindows ##
        value_windows = {}
        # Objects
        assert len(self.contract.objects) == 1
        object_name = self.contract.objects[0]
        obj_ptr = lookuplinkedObject(scene, self.component.linkedObjectName)
        value_windows[object_name] = ValueWindow((lambda t: obj_ptr))

        # Globals
        for global_name in self.contract.globals:
            if global_name == "objects":
                objects_ptr = scene.objects
                value_windows[global_name] = ValueWindow((lambda t: objects_ptr))
            elif global_name == "workspace":
                workspace_ptr = scene.workspace
                value_windows[global_name] = ValueWindow((lambda t: workspace_ptr))
            else:
                raise ValueError(f"Unrecognized global value '{global_name}'")

        # Inputs
        for input_name in self.contract.inputs_types.keys():
            value_windows[input_name] = ValueWindow(
                (lambda t: self.component.last_inputs[input_name])
            )

        # Outputs
        for output_name in self.contract.outputs_types.keys():
            value_windows[output_name] = ValueWindow(
                (lambda t: self.component.last_outputs[output_name])
            )

        # Definitions
        for def_name, def_lambda in self.contract.definitions.items():
            def_closure = lambda l, x: lambda t: l(t, *x.values())
            value_windows[def_name] = ValueWindow(
                def_closure(def_lambda, value_windows.copy())
            )

        ## Create Monitors for Assumptions/Guarantees
        assumptions_monitors = [a.create_monitor() for a in self.contract.assumptions]
        guarantees_monitors = [g.create_monitor() for g in self.contract.guarantees]
        guarantees_values = [rv_ltl.B4.PRESUMABLY_TRUE for g in self.contract.guarantees]

        ## Evaluate Contract ##
        # Instantiate simulator
        simulator = self.scenario.getSimulator()

        # Step contract till termination
        step = 0
        prop_params = [step] + list(value_windows.values())
        with simulator.simulateStepped(scene, maxSteps=self.maxSteps) as simulation:
            while True:
                # Advance simulation one time step, catching any rejections
                try:
                    simulation.advance()
                except (
                    RejectSimulationException,
                    RejectionException,
                    GuardViolation,
                ) as e:
                    return self.createTestData(
                        TestResult.R, self.scenario, scene, simulation, start_time
                    )

                # Update all value windows
                for vw_name, vw in value_windows.items():
                    vw.update(step)

                # Check all assumptions and guarantees
                for assumption in assumptions_monitors:
                    a_val = assumption.update(prop_params)
                    if a_val == rv_ltl.B4.FALSE:
                        return self.createTestData(
                            TestResult.A, self.scenario, scene, simulation, start_time
                        )

                for g_iter, guarantee in enumerate(guarantees_monitors):
                    r_val = guarantee.update(prop_params)
                    guarantees_values[g_iter] = r_val

                # Check if the simulation has ended, and if so break
                if simulation.result:
                    break

                # Increment time
                step += 1

        if any(
            g == rv_ltl.B4.PRESUMABLY_FALSE or g == rv_ltl.B4.FALSE
            for g in guarantees_values
        ):
            return self.createTestData(
                TestResult.G, self.scenario, scene, simulation, start_time
            )
        else:
            return self.createTestData(
                TestResult.V, self.scenario, scene, simulation, start_time
            )

    @staticmethod
    def createTestData(result, scenario, scene, simulation, start_time):
        return SimulationTestData(
            result, scenario.sceneToBytes(scene), simulation.getReplay(), start_time
        )


class ValueWindow:
    def __init__(self, get_val):
        self.get_val = get_val
        self.window = []

    def update(self, time):
        self.window.append(self.get_val(time))

    def __getitem__(self, key):
        return self.window[key]


## Test Data Classes
@enum.unique
class TestResult(enum.Enum):
    V = "Valid: The contract was successfully validated"
    R = "Rejected: The scenario was rejected or a guard was violated"
    A = "Assumptions: An assumption was violated"
    G = "Guarantees: A guarantee was violated"


class TestData:
    def __init__(self, result, elapsed_time):
        self.result = result
        self.elapsed_time = elapsed_time


class SimulationTestData(TestData):
    def __init__(self, result, scene_bytes, sim_replay, start_time):
        super().__init__(result, time.time() - start_time)
        self.scene_bytes = scene_bytes
        self.sim_replay = sim_replay


class ProbabilisticEvidence(ContractEvidence):
    def __init__(self, confidence):
        self.confidence = confidence
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
    def confidenceGap(self):
        if len(self) == 0:
            return 1

        bt = scipy.stats.binomtest(k=self.v_count, n=self.v_count + self.g_count)
        ci = bt.proportion_ci(confidence_level=self.confidence)
        return ci.high - ci.low

    @property
    def correctness(self):
        if len(self) == 0:
            return 0

        bt = scipy.stats.binomtest(
            k=self.v_count, n=self.v_count + self.g_count, alternative="greater"
        )
        ci = bt.proportion_ci(confidence_level=self.confidence)
        return ci.low

    def __len__(self):
        return len(self.testData)

    def __iter__(self):
        return self.testData

    def __str__(self):
        return f"Probabilistic ({100*self.correctness:.2f}% Correctness with {100*self.confidence:.2f}% Confidence)"


## Termination/Requirement Conditions
class TermReqCondition(ABC):
    @abstractmethod
    def check(self, evidence):
        raise NotImplementedError()

    def __str__(self):
        return repr(self)


class TimeTerminationCondition(TermReqCondition):
    def __init__(self, timeout):
        self.timeout = timeout

    def check(self, evidence):
        return evidence.elapsed_time >= self.timeout

    def __repr__(self):
        return f"{self.__class__.__name__}({self.timeout})"


class CountTerminationCondition(TermReqCondition):
    def __init__(self, count):
        self.count = count

    def check(self, evidence):
        return len(evidence) >= self.count

    def __repr__(self):
        return f"{self.__class__.__name__}({self.count})"


class GapTerminationCondition(TermReqCondition):
    def __init__(self, gap):
        self.gap = gap

    def check(self, evidence):
        return evidence.confidenceGap <= self.gap

    def __repr__(self):
        return f"{self.__class__.__name__}({self.gap})"


class CorrectnessRequirementCondition(TermReqCondition):
    def __init__(self, correctness):
        self.correctness = correctness

    def check(self, evidence):
        return evidence.correctness >= self.correctness

    def __repr__(self):
        return f"{self.__class__.__name__}({self.correctness})"
