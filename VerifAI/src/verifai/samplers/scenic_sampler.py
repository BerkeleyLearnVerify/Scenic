
"""Interface to Scenic."""

import importlib.metadata

import scenic
from scenic.core.distributions import needsSampling, Options
from scenic.core.vectors import Vector
from scenic.core.type_support import canCoerceType, coerce, underlyingType

# TODO unify handling of these custom types!
from scenic.simulators.utils.colors import Color
from scenic.simulators.gta.interface import CarModel as GTACarModel
from scenic.simulators.webots.road.car_models import (
    CarModel as WebotsCarModel, carModels as webotsCarModels)

from verifai.features import (Constant, Categorical, Real, Box, Array, Struct,
                              Feature, FeatureSpace)
from verifai.samplers.feature_sampler import FeatureSampler
from verifai.utils.frozendict import frozendict

scenicMajorVersion = int(importlib.metadata.version('scenic').split('.')[0])
if scenicMajorVersion >= 3:
    from scenic.core.vectors import Orientation
else:
    Orientation = None

scalarDomain = Real()
vectorDim = 2 if scenicMajorVersion < 3 else 3
vectorDomain = Array(scalarDomain, (vectorDim,))
orientationDomain = Array(scalarDomain, (4,))  # as quaternions
gtaModelDomain = Categorical(*GTACarModel.models.values())
webotsModelDomain = Categorical(*webotsCarModels)
colorDomain = Box((0, 1), (0, 1), (0, 1))

def convertToVerifaiType(value, strict=True):
    """Attempt to convert a Scenic value to a type known to VerifAI"""
    ty = underlyingType(value)
    if ty is float or ty is int:
        return float(value)
    elif ty is list or ty is tuple:
        return tuple(convertToVerifaiType(e, strict=strict) for e in value)
    elif issubclass(ty, dict) and not needsSampling(value):
        return frozendict(value)
    elif ty is GTACarModel:
        return value
    elif ty is WebotsCarModel:
        return value
    elif ty is Color:
        return value
    elif canCoerceType(ty, Vector):
        return tuple(coerce(value, Vector))
    elif Orientation and isinstance(value, Orientation):
        return tuple(value.getRotation().as_quat())
    elif strict:    # Unknown type, so give up if we're being strict
        raise RuntimeError(
            f'attempted to convert Scenic value {value} of unknown type {ty}')
    else:
        return value

def domainForValue(value):
    """Return a Domain for this type of Scenic value, when possible"""
    ty = underlyingType(value)
    if ty is float or ty is int:
        domain = scalarDomain
    elif ty is GTACarModel:
        domain = gtaModelDomain
    elif ty is WebotsCarModel:
        domain = webotsModelDomain
    elif ty is Color:
        domain = colorDomain
    elif canCoerceType(ty, Vector):
        domain = vectorDomain
    elif ty is Orientation:
        domain = orientationDomain
    elif ty is str:
        # We can only handle strings when they come from a finite set of
        # possibilities; we heuristically detect that here.
        if isinstance(value, Options):
            domain = Categorical(*value.options)
        else:
            domain = None   # we can't ensure the domain is finite
    else:
        domain = None   # no corresponding Domain known
    if not needsSampling(value):
        # We can handle constants of unknown types, but when possible we
        # convert the value to a VerifAI type.
        value = convertToVerifaiType(value, strict=False)
        return Constant(value)
    return domain

def pointForValue(dom, scenicValue):
    """Convert a sampled Scenic value to a point in the corresponding Domain"""
    if isinstance(dom, Constant):
        value = convertToVerifaiType(scenicValue, strict=False)
        assert value == dom.value
        return value
    elif isinstance(dom, Categorical):
        value = convertToVerifaiType(scenicValue, strict=False)
        assert value in dom.values
        return value
    elif dom == scalarDomain:
        if not isinstance(scenicValue, (float, int)):
            raise RuntimeError(
                f'could not coerce Scenic value {scenicValue} to scalar')
        return coerce(scenicValue, float)
    elif dom == vectorDomain:
        return tuple(coerce(scenicValue, Vector))
    elif dom == orientationDomain:
        return tuple(scenicValue.getRotation().as_quat())
    elif dom == colorDomain:
        assert isinstance(scenicValue, (tuple, list))
        assert len(scenicValue) == 3
        return scenicValue
    else:
        raise RuntimeError(
            f'Scenic value {scenicValue} has unexpected domain {dom}')

#: Scenic global parameters not included in generated samples
ignoredParameters = {
    'externalSampler', 'externalSamplerRejectionFeedback',
    'verifaiSamplerType', 'verifaiSamplerParams',
    'behavior',
}
#: Scenic object properties not included in generated samples
defaultIgnoredProperties = {
    'shape',
    'viewAngle', 'viewAngles', 'visibleDistance', 'cameraOffset',
    'viewRayDensity', 'viewRayCount', 'viewRayDistanceScaling',
    'baseOffset', 'contactTolerance', 'onDirection', 'sideComponentThresholds',
    'allowCollisions', 'requireVisible', 'regionContainedIn', 'occluding',
    'showVisibleRegion',
    'mutator', 'mutationEnabled', 'mutationScale',
    'positionStdDev', 'headingStdDev', 'orientationStdDev',
    'behavior', 'lastActions',
}
# certain built-in properties requiring type normalization before Scenic 3
normalizedProperties = {
    'position': Vector,
    'heading': float
} if scenicMajorVersion < 3 else {}
# hard-coded Domains for certain properties
specialDomainProperties = {
    'webotsType': Categorical(*(model.name for model in webotsCarModels)),
    'color': colorDomain,   # to allow Colors, tuples, or lists
}

def domainForObject(obj, ignoredProperties):
    """Construct a Domain for the given Scenic Object"""
    domains = {}
    for prop in obj.properties:
        if prop in ignoredProperties or prop.startswith('_'):
            continue
        value = getattr(obj, prop)
        if prop in normalizedProperties:
            value = coerce(value, normalizedProperties[prop])
        # TODO improve this system... (need to get better type info in Scenic)
        if prop in specialDomainProperties and needsSampling(value):
            dom = specialDomainProperties[prop]
        else:
            dom = domainForValue(value)
        if dom is None:
            ty = underlyingType(value)
            print(f'WARNING: skipping property "{prop}" of unknown type {ty}')
        else:
            domains[prop] = dom
    # add type as additional property
    value = type(obj).__name__
    dom = domainForValue(value)
    assert dom is not None
    assert 'type' not in domains
    domains['type'] = dom
    return Struct(domains)

def pointForObject(dom, obj):
    """Convert a sampled Scenic Object to a point in its Domain"""
    values = []
    for prop, subdom in dom.domainNamed.items():
        if prop == 'type':      # special case for 'type' pseudo-property
            scenicValue = type(obj).__name__
        elif (prop == 'speed'
              and isinstance(subdom, Constant)
              and scenicMajorVersion < 3):
            # work around bug in Scenic 2.1
            scenicValue = subdom.value
        else:
            scenicValue = getattr(obj, prop)
        values.append(pointForValue(subdom, scenicValue))
    return dom.makePoint(*values)

def spaceForScenario(scenario, ignoredProperties):
    """Construct a FeatureSpace for the given Scenic Scenario."""
    # create domains for objects
    assert scenario.egoObject is scenario.objects[0]
    doms = (domainForObject(obj, ignoredProperties)
            for obj in scenario.objects)
    objects = Struct({ f'object{i}': dom for i, dom in enumerate(doms) })

    # create domains for global parameters
    paramDoms = {}
    quotedParams = {}
    for param, value in scenario.params.items():
        if param in ignoredParameters:
            continue
        dom = domainForValue(value)
        if dom is None:
            ty = underlyingType(value)
            print(f'WARNING: skipping param "{param}" of unknown type {ty}')
        else:
            if not param.isidentifier():    # munge quoted parameter names
                newparam = 'quoted_param' + str(len(quotedParams))
                quotedParams[newparam] = param
                param = newparam
            paramDoms[param] = dom
    params = Struct(paramDoms)

    space = FeatureSpace({
        'objects': Feature(objects),
        'params': Feature(params)
    })
    return space, quotedParams

class ScenicSampler(FeatureSampler):
    """Samples from the induced distribution of a Scenic scenario.

    Created using the `fromScenario` and `fromScenicCode` class methods.

    See :ref:`scene generation` in the Scenic documentation for details of how
    Scenic's sampler works. Note that VerifAI's other samplers can be used from
    within a Scenic scenario by defining :term:`external parameters`.
    """

    def __init__(self, scenario, maxIterations=None, ignoredProperties=None):
        self.scenario = scenario
        self.maxIterations = 2000 if maxIterations is None else maxIterations
        self.lastScene = None
        if ignoredProperties is None:
            ignoredProperties = defaultIgnoredProperties
        space, self.quotedParams = spaceForScenario(scenario, ignoredProperties)
        super().__init__(space)

    @classmethod
    def fromScenario(cls, path, maxIterations=None,
                     ignoredProperties=None, **kwargs):
        """Create a sampler corresponding to a Scenic program.

        The only required argument is ``path``, and ``maxIterations`` may be useful if
        your scenario requires a very large number of rejection sampling iterations.
        See `scenic.scenarioFromFile` for details on optional keyword arguments used to
        customize compilation of the Scenic file.

        Args:
            path (str): path to a Scenic file.
            maxIterations (int): maximum number of rejection sampling iterations
              (default 2000).
            ignoredProperties: properties of Scenic objects to not include in
              generated samples (see ``defaultIgnoredProperties`` for the default).
            kwargs: additional keyword arguments passed to `scenic.scenarioFromFile`;
              e.g. ``params`` to override global parameters or ``model`` to set the
              :term:`world model`.
        """
        scenario = scenic.scenarioFromFile(path, **kwargs)
        return cls(scenario, maxIterations=maxIterations,
                   ignoredProperties=ignoredProperties)

    @classmethod
    def fromScenicCode(cls, code, maxIterations=None,
                       ignoredProperties=None, **kwargs):
        """As above, but given a Scenic program as a string."""
        scenario = scenic.scenarioFromString(code, **kwargs)
        return cls(scenario, maxIterations=maxIterations,
                   ignoredProperties=ignoredProperties)

    def nextSample(self, feedback=None):
        ret = self.scenario.generate(
            maxIterations=self.maxIterations, feedback=feedback, verbosity=0
        )
        self.lastScene, _ = ret
        return self.pointForScene(self.lastScene)

    def pointForScene(self, scene):
        """Convert a sampled Scenic :obj:`Scene` to a point in our feature space."""
        lengths, dom = self.space.domains
        assert lengths is None
        assert scene.egoObject is scene.objects[0]
        objDomain = dom.domainNamed['objects']
        assert len(objDomain.domains) == len(scene.objects)
        objects = (pointForObject(objDomain.domainNamed[f'object{i}'], obj)
                   for i, obj in enumerate(scene.objects))
        objPoint = objDomain.makePoint(*objects)

        paramDomain = dom.domainNamed['params']
        params = {}
        for param, subdom in paramDomain.domainNamed.items():
            originalName = self.quotedParams.get(param, param)
            params[param] = pointForValue(subdom, scene.params[originalName])
        paramPoint = paramDomain.makePoint(**params)

        return self.space.makePoint(objects=objPoint, params=paramPoint)

    def paramDictForSample(self, sample):
        """Recover the dict of global parameters from a `ScenicSampler` sample."""
        params = sample.params._asdict()
        corrected = {}
        for newName, quotedParam in self.quotedParams.items():
            corrected[quotedParam] = params.pop(newName)
        corrected.update(params)
        return corrected
