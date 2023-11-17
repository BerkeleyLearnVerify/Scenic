import enum

import rv_ltl

from scenic.contracts.utils import linkSetBehavior, lookuplinkedObject
from scenic.core.distributions import RejectionException
from scenic.core.dynamics import GuardViolation, RejectSimulationException


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


class ScenicTesting:
    def __init__(self, contract, component, time, scenario=None, obj=None):
        # Store attributes
        self.contract = contract
        self.component = component
        self.scenario = scenario
        self.obj = obj

        # Ensure that a scenario is provided if the contract references
        # any globals.
        if len(self.contract.globals) > 0:
            assert scenario is not None

        # Ensure that a component is provided if the contract references
        # any input/output
        if len(self.contract.inputs_types) + len(self.contract.outputs_types) > 0:
            assert component is not None

    def run_test(self):
        # Generate scene
        scene, _ = self.scenario.generate()

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
        simulation = simulator.simulate(scene, maxSteps=50, manual=True)

        # Step contract till termination
        t = 0
        prop_params = [t] + list(value_windows.values())
        while True:
            # Advance simulation one time step, catching any rejections
            try:
                simulation.advance()
            except (RejectSimulationException, RejectionException, GuardViolation) as e:
                return TestResult.R

            # Update all value windows
            for vw_name, vw in value_windows.items():
                vw.update(t)

            # Check all assumptions and guarantees
            for assumption in assumptions_monitors:
                a_val = assumption.update(prop_params)
                if a_val == rv_ltl.B4.FALSE:
                    return TestResult.A

            for g_iter, guarantee in enumerate(guarantees_monitors):
                r_val = guarantee.update(prop_params)
                guarantees_values[g_iter] = r_val

            # Check if the simulation has ended, and if so break
            if simulation.result:
                break

            # Increment time
            t += 1

        if any(
            g == rv_ltl.B4.PRESUMABLY_FALSE or g == rv_ltl.B4.FALSE
            for g in guarantees_values
        ):
            return TestResult.G
        else:
            return TestResult.V


class ValueWindow:
    def __init__(self, get_val):
        self.get_val = get_val
        self.window = []

    def update(self, time):
        self.window.append(self.get_val(time))

    def __getitem__(self, key):
        return self.window[key]


@enum.unique
class TestResult(enum.Enum):
    V = "Valid: The contract was successfully validated"
    R = "Rejected: The scenario was rejected or a guard was violated"
    A = "Assumptions: An assumption was violated"
    G = "Guarantees: A guarantee was violated"
