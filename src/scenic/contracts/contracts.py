from scenic.contracts.utils import lookuplinkedObject


class Contract:
    def __init__(self):
        # Create and store lambdas for definitions, assumptions, and guarantees
        props = self.prop_factory()
        self.definitions = props[0]
        self.assumptions = props[1]
        self.guarantees = props[2]

        # TODO: Handle contracts w/ more than one object
        assert len(self.objects) <= 1


class ScenicTesting:
    def __init__(self, contract, time, scenario=None, component=None):
        # Store attributes
        self.contract = contract
        self.scenario = scenario
        self.component = component

        # Ensure that a scenario is provided if the contract references
        # any globals.
        if len(self.globals) > 0:
            assert scenario is not None

        # Ensure that a component is provided if the contract references
        # any input/output
        if len(self.input_types) + len(self.output_types) > 0:
            assert component is not None

    def run_test(self):
        # Generate scene
        scene = self.scenario.generate()

        ## Create and link ValueWindows ##
        value_windows = {}
        # Objects
        assert len(self.contract.objects) == 1
        object_name = self.contract.objects[0]
        obj_ptr = lookuplinkedObject(scene, object_name)
        value_windows[object_name] = ValueWindow((lambda: obj_ptr))

        # Globals
        for global_name in self.contract.globals:
            if global_name == "objects":
                objects_ptr = scene.objects
                value_windows[global_name] = ValueWindow((lambda: objects_ptr))
            elif global_name == "workspace":
                workspace_ptr = scene.workspace
                value_windows[global_name] = ValueWindow((lambda: workspace_ptr))
            else:
                raise ValueError(f"Unrecognized global value '{global_name}'")

        # Inputs
        for input_name in contract.input_types.keys():
            value_windows[input_name] = ValueWindow(
                (lambda: self.contract.last_inputs[input_name])
            )

        # Outputs
        for output_name in contract.output_types.keys():
            value_windows[output_name] = ValueWindow(
                (lambda: self.contract.last_outputs[output_name])
            )

        # Definitions
        for def_name, def_lambda in contract.definitions:
            def_closure = lambda x: lambda: def_lambda(**x)
            value_windows[def_name] = ValueWindow(def_closure(value_windows.copy()))

        ## Evaluate Contract ##
        # Instantiate simulator
        simulator = scenario.getSimulator()
        simulation = simulator.simulate(scene, maxSteps=time)


class ValueWindow:
    def __init__(self, get_val):
        self.get_val = get_val
        self.window = []

    def update(self):
        self.window.append(get_val())

    def __getitem__(self, key):
        return self.window[key]
