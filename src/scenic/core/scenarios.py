"""Scenario and scene objects."""

import dataclasses
import io
import random
import time
import sys

import rv_ltl

from scenic.core.distributions import (Samplable, ConstantSamplable, RejectionException,
                                       needsSampling)
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.external_params import ExternalSampler
from scenic.core.regions import EmptyRegion, convertToFootprint
from scenic.core.vectors import Vector
from scenic.core.errors import (InvalidScenarioError, RuntimeParseError,
                                optionallyDebugRejection)
from scenic.core.dynamics import Behavior, Monitor
from scenic.core.requirements import BoundRequirement
from scenic.core.serialization import Serializer, dumpAsScenicCode

# Pickling support

class _ScenarioPickleMixin:
    def __getstate__(self):
        # Start the pickle with an object storing our compile options and activating
        # the veneer with them; this will ensure they are consistent during import of
        # any needed Scenic modules (which might have been purged earlier, and in any
        # case won't already exist during unpickling). Similarly, tack a dummy object
        # on the end of the pickle which will deactivate the veneer and clean up.
        oldModules = []
        options = dataclasses.replace(
            self.compileOptions,
            paramOverrides=self.params, # save all params, not just those from --param
        )
        elements = (
            _Activator(options, oldModules),
            self.__dict__,
            _Deactivator(oldModules)
        )
        return elements

    def __setstate__(self, state):
        self.__dict__.update(state[1])

class _Activator:
    def __init__(self, compileOptions, oldModules):
        self.compileOptions = compileOptions
        # Save all modules already imported prior to pickling
        oldModules.extend(sys.modules.keys())

    def activate(self):
        import scenic.syntax.veneer as veneer
        assert not veneer.isActive()
        veneer.activate(self.compileOptions)

    def __getstate__(self):
        # Step 1 (during pickling)
        self.activate()
        return self.__dict__

    def __setstate__(self, state):
        # Step 3 (during unpickling)
        self.__dict__.update(state)
        self.activate()

class _Deactivator:
    def __init__(self, oldModules):
        self.oldModules = oldModules

    def deactivate(self):
        import scenic.syntax.veneer as veneer
        veneer.deactivate()
        assert not veneer.isActive(), 'nested pickle of Scene/Scenario'
        # Purge Scenic modules imported during pickling
        from scenic.syntax.translator import purgeModulesUnsafeToCache
        purgeModulesUnsafeToCache(self.oldModules)

    def __getstate__(self):
        # Step 2 (during pickling)
        self.deactivate()
        return self.__dict__

    def __setstate__(self, state):
        # Step 4 (during unpickling)
        self.__dict__.update(state)
        self.deactivate()

# Scenes and scenarios

class Scene(_ScenarioPickleMixin):
    """Scene()

    A scene generated from a Scenic scenario.

    To run a dynamic simulation from a scene, create an instance of `Simulator` for the
    simulator you want to use, and pass the scene to its `simulate` method.

    Attributes:
        objects (tuple of :obj:`~scenic.core.object_types.Object`): All objects in the
          scene. The ``ego`` object is first, if there is one.
        egoObject (:obj:`~scenic.core.object_types.Object` or `None`): The ``ego`` object, if any.
        params (dict): Dictionary mapping the name of each global parameter to its value.
        workspace (:obj:`~scenic.core.workspaces.Workspace`): The :term:`workspace` for the scenario.

    .. versionchanged:: 3.0

        The ``egoObject`` attribute can now be `None`.
    """
    def __init__(self, workspace, objects, egoObject, params,
                 temporalReqs=(),terminationConds=(), termSimulationConds=(),
                 recordedExprs=(), recordedInitialExprs=(), recordedFinalExprs=(),
                 monitors=(), behaviorNamespaces={}, dynamicScenario=None,
                 sample={}, compileOptions={}):
        self.workspace = workspace
        self.objects = tuple(objects)
        self.egoObject = egoObject
        self.params = params
        self.temporalRequirements = tuple(temporalReqs)
        self.terminationConditions = tuple(terminationConds)
        self.terminateSimulationConditions = tuple(termSimulationConds)
        self.recordedExprs = tuple(recordedExprs)
        self.recordedInitialExprs = tuple(recordedInitialExprs)
        self.recordedFinalExprs = tuple(recordedFinalExprs)
        self.monitors = tuple(monitors)
        self.behaviorNamespaces = behaviorNamespaces
        self.dynamicScenario = dynamicScenario
        self.sample = sample
        self.compileOptions = compileOptions

    def dumpAsScenicCode(self, stream=sys.stdout):
        """Dump Scenic code reproducing this scene to the given stream.

        For non-human-readable but complete serialization of scenes see
        `Scenario.sceneToBytes` and `Scenario.sceneFromBytes`.

        .. note::

            This function does not currently reproduce parts of the original Scenic
            program defining behaviors, functions, etc. used in the scene. Also, if
            the scene involves any user-defined types, they must provide a suitable
            :obj:`~object.__repr__` for this function to print them properly.

        Args:
            stream (:term:`text file`): Where to print the code (default `sys.stdout`).
        """
        for name, value in self.params.items():
            if str.isidentifier(name):
                stream.write(f'param {name} = ')
            else:
                stream.write(f'param "{name}" = ')
            dumpAsScenicCode(value, stream)
            stream.write('\n')
        stream.write('ego = ')
        for obj in self.objects:
            dumpAsScenicCode(obj, stream)
            stream.write('\n')

    def show(self, zoom=None, block=True):
        self.show_3d()

    def show_3d(self):
        """Render a 3D schematic of the scene for debugging."""
        import trimesh

        # Create a new trimesh scene to contain meshes
        render_scene = trimesh.scene.Scene()

        # display map
        self.workspace.show_3d(render_scene)
        # draw objects
        for obj in self.objects:
            obj.show_3d(render_scene, highlight=(obj is self.egoObject))

        # If nothing else is in the viewer, add some constructs
        # to avoid a crash
        if render_scene.is_empty:
            render_scene.add_geometry(trimesh.points.PointCloud([(0.1,0,0),(0,0.1,0), (0,0,.1)], colors=[255,255,255,0]))

        render_scene.show(flags={'axis': 'world'})

    def show_2d(self, zoom=None, block=True):
        """Render a 2D schematic of the scene for debugging."""
        import matplotlib.pyplot as plt
        plt.gca().set_aspect('equal')
        # display map
        self.workspace.show_2d(plt)
        # draw objects
        for obj in self.objects:
            obj.show_2d(self.workspace, plt, highlight=(obj is self.egoObject))
        # zoom in if requested
        if zoom:
            self.workspace.zoomAround(plt, self.objects, expansion=zoom)
        plt.show(block=block)

class Scenario(_ScenarioPickleMixin):
    """Scenario()

    A compiled Scenic scenario, from which scenes can be sampled.
    """
    def __init__(self, workspace, simulator,
                 instances, objects, egoObject,
                 params, externalParams,
                 requirements, requirementDeps,
                 monitors, behaviorNamespaces,
                 dynamicScenario, astHash, compileOptions):
        self.workspace = workspace
        self.simulator = simulator      # simulator for dynamic scenarios
        # make ego the first object, while otherwise preserving order
        ordered = []
        for obj in objects:
            if obj is not egoObject:
                ordered.append(obj)
        assert set(objects).issubset(set(instances))
        self._instances = tuple(instances)
        self.objects = (egoObject,) + tuple(ordered) if egoObject else tuple(ordered)
        self.egoObject = egoObject
        self.params = dict(params)
        self.externalParams = tuple(externalParams)
        self.externalSampler = ExternalSampler.forParameters(self.externalParams, self.params)
        self.monitors = tuple(monitors)
        self.behaviorNamespaces = behaviorNamespaces
        self.dynamicScenario = dynamicScenario
        self.astHash = astHash
        self.compileOptions = compileOptions

        staticReqs, alwaysReqs, terminationConds = [], [], []
        self.requirements = tuple(dynamicScenario._requirements)    # TODO clean up
        self.terminationConditions = tuple(dynamicScenario._terminationConditions)
        self.terminateSimulationConditions = tuple(dynamicScenario._terminateSimulationConditions)
        self.initialRequirements = self.requirements
        assert all(req.constrainsSampling for req in self.initialRequirements)
        self.recordedExprs = tuple(dynamicScenario._recordedExprs)
        self.recordedInitialExprs = tuple(dynamicScenario._recordedInitialExprs)
        self.recordedFinalExprs = tuple(dynamicScenario._recordedFinalExprs)
        # dependencies must use fixed order for reproducibility
        paramDeps = tuple(p for p in self.params.values() if isinstance(p, Samplable))
        behaviorDeps = []
        for namespace in self.behaviorNamespaces.values():
            for value in namespace.values():
                if isinstance(value, Samplable):
                    behaviorDeps.append(value)
        self.dependencies = self._instances + paramDeps + tuple(requirementDeps) + tuple(behaviorDeps)

        self.validate()

    def containerOfObject(self, obj):
        if hasattr(obj, 'regionContainedIn') and obj.regionContainedIn is not None:
            return obj.regionContainedIn
        else:
            return convertToFootprint(self.workspace.region)

    def validate(self):
        """Make some simple static checks for inconsistent built-in requirements.

        :meta private:
        """
        objects = self.objects
        staticVisibility = self.egoObject and not needsSampling(self.egoObject.visibleRegion)
        staticBounds = [obj._hasStaticBounds for obj in objects]
        for i in range(len(objects)):
            oi = objects[i]
            container = self.containerOfObject(oi)
            # Trivial case where container is empty
            if isinstance(container, EmptyRegion):
                raise InvalidScenarioError(f'Container region of {oi} is empty')
            # skip objects with unknown positions or bounding boxes
            if not staticBounds[i]:
                continue
            # Require object to be contained in the workspace/valid region
            if not needsSampling(container) and not container.containsObject(oi):
                raise InvalidScenarioError(f'Object at {oi.position} does not fit in container')
            # Require object to be visible from the ego object
            if staticVisibility and oi.requireVisible is True and oi is not self.egoObject:
                if not self.egoObject.canSee(oi):
                    raise InvalidScenarioError(f'Object at {oi.position} is not visible from ego')
            if not oi.allowCollisions:
                # Require object to not intersect another object
                for j in range(i):
                    oj = objects[j]
                    if oj.allowCollisions or not staticBounds[j]:
                        continue
                    if oi.intersects(oj):
                        raise InvalidScenarioError(f'Object at {oi.position} intersects'
                                                   f' object at {oj.position}')

    def generate(self, maxIterations=2000, verbosity=0, feedback=None):
        """Sample a `Scene` from this scenario.

        For a description of how scene generation is done, see `scene generation`.

        Args:
            maxIterations (int): Maximum number of rejection sampling iterations.
            verbosity (int): Verbosity level.
            feedback (float): Feedback to pass to external samplers doing active sampling.
                See :mod:`scenic.core.external_params`.

        Returns:
            A pair with the sampled `Scene` and the number of iterations used.

        Raises:
            `RejectionException`: if no valid sample is found in **maxIterations** iterations.
        """
        objects = self.objects

        # choose which custom requirements will be enforced for this sample
        activeReqs = [req for req in self.initialRequirements if random.random() <= req.prob]

        # do rejection sampling until requirements are satisfied
        rejection = True
        iterations = 0
        while rejection is not None:

            if iterations > 0:  # rejected the last sample
                if verbosity >= 2:
                    print(f'  Rejected sample {iterations} because of: {rejection}')
                if self.externalSampler is not None:
                    feedback = self.externalSampler.rejectionFeedback
            if iterations >= maxIterations:
                raise RejectionException(f'failed to generate scenario in {iterations} iterations')
            iterations += 1
            try:
                if self.externalSampler is not None:
                    self.externalSampler.sample(feedback)
                sample = Samplable.sampleAll(self.dependencies)
            except RejectionException as e:
                optionallyDebugRejection(e)
                rejection = e
                continue
            rejection = None
            ego = sample[self.egoObject]

            # Normalize types of some built-in properties
            for obj in objects:
                sampledObj = sample[obj]
                assert not needsSampling(sampledObj)
                # position, heading
                assert isinstance(sampledObj.position, Vector)
                assert isinstance(sampledObj.heading, float)
                # behavior
                behavior = sampledObj.behavior
                if behavior is not None and not isinstance(behavior, Behavior):
                    raise InvalidScenarioError(
                        f'behavior {behavior} of Object {obj} is not a behavior')

            # Check built-in requirements for instances
            for i in range(len(self._instances)):
                vi = sample[self._instances[i]]

                # Require that the observing entity, if one has been specified,
                # can see the instance.
                if vi._observingEntity is not None:
                    observing_entity = sample[vi._observingEntity]
                    occluding_objects = tuple(sample[obj] for obj in objects \
                                         if sample[obj] is not observing_entity \
                                         and sample[obj] is not vi and obj.occluding)
                    if not observing_entity.canSee(vi, occludingObjects=occluding_objects):
                        rejection = 'instance visibility (from observing entity)'
                        break
                # Require that the non-observing entity, if one has been specified,
                # can see the instance.
                if vi._nonObservingEntity is not None:
                    non_observing_entity = sample[vi._nonObservingEntity]
                    occluding_objects = tuple(sample[obj] for obj in objects \
                                         if sample[obj] is not non_observing_entity \
                                         and sample[obj] is not vi and obj.occluding)
                    if non_observing_entity.canSee(vi, occludingObjects=occluding_objects):
                        rejection = 'instance visibility (from non-observing entity)'

            # Check built-in requirements for objects
            for i in range(len(objects)):
                vi = sample[objects[i]]
                # Require object to be contained in the workspace/valid region
                container = self.containerOfObject(vi)
                if not container.containsObject(vi):
                    rejection = 'object containment'
                    break
                # Require object to be visible from the ego object
                if vi.requireVisible and vi is not ego:
                    occluding_objects = tuple(sample[obj] for obj in objects if sample[obj] is not ego \
                                         and sample[obj] is not vi and obj.occluding)

                    if not ego.canSee(vi, occludingObjects=occluding_objects):
                        rejection = 'object visibility (from ego)'
                        break

                        break
                # Require object to not intersect another object
                if not vi.allowCollisions:
                    for j in range(i):
                        vj = sample[objects[j]]

                        if not vj.allowCollisions and vi.intersects(vj):
                            rejection = 'object intersection'
                            break
                if rejection is not None:
                    break
            if rejection is not None:
                optionallyDebugRejection()
                continue
            # Check user-specified requirements
            for req in activeReqs:
                if req.satisfiedBy(sample) == rv_ltl.B4.FALSE:
                    rejection = str(req)
                    optionallyDebugRejection()
                    break

        # obtained a valid sample; assemble a scene from it
        scene = self._makeSceneFromSample(sample)
        return scene, iterations

    def _makeSceneFromSample(self, sample):
        sampledObjects = tuple(sample[obj] for obj in self.objects)
        ego = sample[self.egoObject]
        sampledParams = {}
        for param, value in self.params.items():
            sampledValue = sample[value]
            assert not needsLazyEvaluation(sampledValue)
            sampledParams[param] = sampledValue
        sampledNamespaces = {}
        for modName, namespace in self.behaviorNamespaces.items():
            sampledNamespace = { name: sample[value] for name, value in namespace.items() }
            sampledNamespaces[modName] = (namespace, sampledNamespace, namespace.copy())
        temporalReqs = (BoundRequirement(req, sample, req.proposition) for req in self.requirements)
        monitors = []
        for req in self.monitors:
            breq = BoundRequirement(req, sample, None)
            monitor = breq.evaluate()
            if not isinstance(monitor, Monitor):
                raise RuntimeParseError('"require monitor X" with X not a monitor'
                                        f' on line {breq.line}')
            monitors.append(monitor)
        terminationConds = (BoundRequirement(req, sample, req.proposition)
                            for req in self.terminationConditions)
        termSimulationConds = (BoundRequirement(req, sample, req.proposition)
                               for req in self.terminateSimulationConditions)
        recordedExprs = (BoundRequirement(req, sample, req.proposition) for req in self.recordedExprs)
        recordedInitialExprs = (BoundRequirement(req, sample, req.proposition)
                                for req in self.recordedInitialExprs)
        recordedFinalExprs = (BoundRequirement(req, sample, req.proposition)
                              for req in self.recordedFinalExprs)
        scene = Scene(self.workspace, sampledObjects, ego, sampledParams,
                      temporalReqs, terminationConds, termSimulationConds,
                      recordedExprs, recordedInitialExprs,recordedFinalExprs,
                      monitors, sampledNamespaces, self.dynamicScenario,
                      sample, self.compileOptions)
        return scene

    def resetExternalSampler(self):
        """Reset the scenario's external sampler, if any.

        If the Python random seed is reset before calling this function, this
        should cause the sequence of generated scenes to be deterministic.
        """
        self.externalSampler = ExternalSampler.forParameters(self.externalParams, self.params)

    def conditionOn(self, scene=None, objects=(), params={}):
        """Condition the scenario on particular values for some objects or parameters.

        This method changes the distribution of the scenario and should be used with
        care: it does not attempt to check that the new distribution is equivalent to the
        old one or that it has nonzero probability of satisfying the scenario's
        requirements.

        For example, to sample object #5 in the scenario once and then leave it fixed in
        all subsequent samples::

            sceneA, _ = scenario.generate()
            scenario.conditionOn(scene=sceneA, objects=(5,))
            sceneB, _ = scenario.generate()     # will have the same object 5 as sceneA

        Args:
            scene (Scene): Scene from which to take values for the given **objects**,
                if any.
            objects: Sequence of indices specifying which objects in this scenario should
                be conditioned on the corresponding objects in **scene** (i.e. those with
                the same index in the list of objects).
            params (dict): Dictionary of global parameters to condition and their new
                values (which may be constants or distributions).
        """
        assert objects or params
        assert bool(scene) == bool(objects)
        if scene:
            assert len(self.objects) == len(scene.objects)
        for i in objects:
            assert i < len(self.objects)
            self.objects[i].conditionTo(scene.objects[i])
        for param, newVal in params.items():
            curVal = self.params[param]
            if isinstance(curVal, Samplable):
                if not isinstance(newVal, Samplable):
                    newVal = ConstantSamplable(newVal)
                curVal.conditionTo(newVal)
            else:
                self.params[param] = newVal

    def getSimulator(self):
        if self.simulator is None:
            raise RuntimeError('scenario does not specify a simulator')
        import scenic.syntax.veneer as veneer
        return veneer.instantiateSimulator(self.simulator, self.params)

    def sceneToBytes(self, scene, allowPickle=False):
        """Encode a `Scene` sampled from this scenario to a `bytes` object.

        The serialized scene may be reconstituted with `sceneFromBytes`. The format used
        is suitable for long-term storage of scenes, although it is not guaranteed to be
        compatible across major versions of Scenic. For further discussion and usage
        examples, see :ref:`serialization`.

        Raises:
            SerializationError: if the scene could not be properly encoded. This should
                not happen unless your scenario includes a user-defined `Distribution`
                subclass with an unusual value type. If you get this exception, see the
                documentation for the internal class `Serializer` for solutions.
        """
        ser = Serializer(allowPickle=allowPickle)
        ser.writeScene(self, scene)
        return ser.getBytes()

    def sceneFromBytes(self, data, verify=True, allowPickle=False):
        """Decode a `Scene` serialized with `sceneToBytes`.

        Args:
            data (bytes): Encoding of a `Scene` sampled from this scenario.
            verify (bool): If true (the default), raise an exception if the scene
                appears to have been generated from a different scenario (meaning
                it will almost certainly not decode correctly).
            allowPickle (bool): Enable using `pickle` to deserialize custom object
                types. False by default because it allows malicious data to trigger
                arbitrary code execution (see the `pickle` documentation). Use this
                option only if you trust the source of the data and it is not practical
                to implement serialization for the datatypes you need.

        Raises:
            SerializationError: if the scene could not be properly decoded.
        """
        ser = Serializer(data, allowPickle=allowPickle)
        return ser.readScene(self, verify=verify)

    def simulationToBytes(self, simulation, allowPickle=False):
        """Encode a `Simulation` sampled from this scenario to a `bytes` object.

        The serialized simulation may be replayed with `simulationFromBytes`. As with
        `sceneToBytes`, the format used is suitable for long-term storage but is not
        guaranteed to be compatible across major versions of Scenic.

        Raises:
            SerializationError: if the simulation could not be properly encoded. This should
                not happen unless your scenario includes a user-defined `Distribution`
                subclass with an unusual value type. If you get this exception, see the
                documentation for the internal class `Serializer` for solutions.

        .. note::

            The returned data encodes both the scene comprising the initial condition for the
            simulation and the simulation itself. If you will be running many simulations
            starting from the same scene, you can save space by separately encoding the scene
            and the various simulations: use `sceneToBytes` and `Simulation.getReplay` for
            encoding, and the **replay** argument of `Simulator.simulate` for decoding.
        """
        sceneData = self.sceneToBytes(simulation.scene)
        replay = simulation.getReplay()
        return sceneData + replay

    def simulationFromBytes(self, data, simulator, *,
                            verify=True, allowPickle=False, **kwargs):
        """Replay a `Simulation` serialized with `simulationToBytes`.

        Args:
            data (bytes): Encoding of a `Simulation` sampled from this scenario.
            simulator (Simulator): Simulator in which to run the simulation. Using
                a different simulator configuration than that used for the original
                simulation may cause errors or unexpected behavior. If you need to do
                this, see the **enableDivergenceCheck** option of `Simulator.simulate`.
            verify (bool): As in `sceneFromBytes`.
            allowPickle (bool): As in `sceneFromBytes`.
            kwargs: All additional keyword arguments are passed through to the simulator;
                see `Simulator.simulate` for the available configuration options.

        Returns:
            A `Simulation` object representing the completed simulation.

        Raises:
            SerializationError: if the simulation could not be properly decoded.
            DivergenceError: if the replayed simulation has diverged from the original
                (requires the original to have been run with divergence-checking support;
                see `Simulator.simulate`).
        """
        if not isinstance(data, io.BufferedIOBase):
            data = io.BytesIO(data)
        scene = self.sceneFromBytes(data, verify=verify, allowPickle=allowPickle)
        return simulator.simulate(scene, replay=data, **kwargs)
