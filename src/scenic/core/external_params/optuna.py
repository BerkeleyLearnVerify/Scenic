from abc import abstractmethod
import math

import optuna

from scenic.core.distributions import (
    DiscreteRange,
    Normal,
    Options,
    Range,
    toDistribution,
)
from scenic.core.external_params.external_params import *
from scenic.core.lazy_eval import valueInContext
from scenic.core.regions import PointInRegionDistribution
import scenic.syntax.veneer


class OptunaSampler(ExternalSampler):
    def __init__(self, params, globalParams):
        super().__init__(params, globalParams)

        optuna.logging.set_verbosity(
            globalParams.get("optunaLogLevel", optuna.logging.WARNING)
        )

        self.sampler = globalParams.get("optunaSampler", None)
        self.study_name = globalParams.get("optunaStudyName", None)
        self.storage = globalParams.get("optunaStorage", None)
        self.study = optuna.create_study(
            sampler=self.sampler,
            study_name=self.study_name,
            pruner=optuna.pruners.NopPruner(),
            storage=self.storage,
            direction="minimize",
        )
        self.params = tuple(params)
        for index, param in enumerate(self.params):
            if not isinstance(param, OptunaParameter):
                raise RuntimeError(
                    f"OptunaSampler given parameter of wrong type: {param}"
                )
            param.sampler = self
            param.index = index

    @classmethod
    def getExternalParamType(cls):
        return OptunaParameter

    @classmethod
    def getExternalParameterConverter(cls):
        return OptunaParameterConverter(cls, cls.getExternalParamType())

    @property
    def trial(self):
        return self.cachedSample

    def nextSample(self, feedback):
        if feedback is not None:
            if self.study_name is None:
                warnings.warn(
                    "Feedback passed without setting Optuna study name. Feedback will not have any effect."
                )
            assert self.trial is not None
            self.study.tell(self.trial, feedback)

        return self.study.ask()

    def valueFor(self, param):
        if param.isTimeSeries:
            raise ValueError(
                "OptunaSampler does currently support timeSeries parameters."
            )

        return param.suggestValue(self)


class OptunaParameterConverter(ExternalParameterConverter):
    def convertInner(self, dist):
        distCond = dist._conditioned
        if isinstance(distCond, Range):
            self.convert(distCond.low)
            self.convert(distCond.high)
            newDist = OptunaRange(distCond.low, distCond.high)
            self.registerExternalParams(newDist)
            dist.conditionTo(newDist)
        elif isinstance(distCond, DiscreteRange):
            self.convert(distCond.low)
            self.convert(distCond.high)
            newDist = OptunaDiscreteRange(distCond.low, distCond.high)
            self.registerExternalParams(newDist)
            dist.conditionTo(newDist)
        elif isinstance(distCond, Options):
            for o in distCond.options:
                self.convert(o)
            newDist = OptunaOptions(distCond.options)
            self.registerExternalParams(newDist)
            dist.conditionTo(newDist)
        elif isinstance(distCond, Normal):
            self.convert(distCond.stddev)
            self.convert(distCond.mean)
            newDist = Normal.cdfinv(distCond.mean, distCond.stddev, OptunaRange(-1, 1))
            self.registerExternalParams(newDist)
            dist.conditionTo(newDist)
        elif (
            isinstance(distCond, PointInRegionDistribution)
            and not distCond._deterministic
        ):
            self.convert(distCond.region)

            # def makeSampleVals(dims):
            #     if isinstance(dims, tuple):
            #         return toDistribution(tuple(makeSampleVals(dim) for dim in dims))
            #     elif isinstance(dims, int):
            #         return toDistribution(tuple(OptunaRange(0,1) for _ in range(dims)))
            #     else:
            #         assert False, dims

            # sampleVals = makeSampleVals(dist.region._sampleVals)
            sampleVals = toDistribution(tuple(OptunaRange(0, 1) for _ in range(3)))
            self.registerExternalParams(sampleVals)
            newDist = PointInRegionDistribution(
                region=distCond.region, tag=distCond.tag, sampleVals=sampleVals
            )
            dist.conditionTo(newDist)
        elif distCond._deterministic:
            for dep in distCond._dependencies:
                self.convert(dep)
        else:
            raise NotImplementedError


class OptunaParameter(ExternalParameter):
    def __init__(self):
        super().__init__()
        self.index = None

    @abstractmethod
    def suggestValue(self, trial):
        pass

    @property
    def optunaName(
        self,
    ):
        assert self.index is not None
        return f"{type(self)}_{self.index}"


class OptunaRange(OptunaParameter):
    """A :obj:`~scenic.core.distributions.Range` (real interval) sampled by VerifAI."""

    _defaultValueType = float

    def __init__(self, low, high):
        super().__init__()
        self.low = low
        self.high = high

    def suggestValue(self, sampler):
        return sampler.trial.suggest_float(self.optunaName, self.low, self.high)


class OptunaDiscreteRange(OptunaParameter):
    """A :obj:`~scenic.core.distributions.DiscreteRange` (integer interval) sampled by Optuna."""

    _defaultValueType = int

    def __init__(self, low, high):
        super().__init__()
        self.low = low
        self.high = high

    def suggestValue(self, sampler):
        return sampler.trial.suggest_int(self.optunaName, self.low, self.high)


class _OptunaCategoricalHelper(OptunaParameter):
    _defaultValueType = int

    def __init__(self, numOptions):
        super().__init__()
        self.numOptions = numOptions

    def suggestValue(self, sampler):
        return sampler.trial.suggest_categorical(
            self.optunaName, list(range(self.numOptions + 1))
        )


class OptunaOptions(Options):
    """An :obj:`~scenic.core.distributions.Options` (discrete set) sampled by Optuna."""

    @staticmethod
    def makeSelector(n, weights):
        if weights:
            warnings.warn("Ignoring weights passed to OptunaOptions.")
        return _OptunaCategoricalHelper(n)
