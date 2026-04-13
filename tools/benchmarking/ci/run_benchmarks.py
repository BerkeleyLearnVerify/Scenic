import math
from pathlib import Path
import sys
import time
import warnings

import pyperf

import scenic
from scenic.core.distributions import RejectionException

scenic.setDebuggingOptions(fullBacktrace=True, debugExceptions=False)


def normalizeOptionValue(value):
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    if value == "True":
        return True
    if value == "False":
        return False
    return value


def parseOptions(path):
    prefix = "# option: "
    prefixLen = len(prefix)
    options = {}
    with path.open() as f:
        for i, line in enumerate(f):
            if not line.startswith(prefix):
                break
            rest = line[prefixLen:]
            name, sep, value = rest.rpartition("=")
            if sep != "=":
                raise RuntimeError(f"benchmark option line {i} is malformed")
            if name in options:
                raise RuntimeError(
                    f"benchmark option '{name}' is specified multiple times"
                )
            options[name] = normalizeOptionValue(value)
    return options


def sampleForIterations(loops, path, iterations):
    options = parseOptions(path)
    times = []

    for _ in range(loops):
        # Recompile each time to reset caches, checker state, etc.
        scenario = scenic.scenarioFromFile(path, **options)

        n = iterations
        t0 = time.perf_counter()

        try:
            while n > 0:
                scene, its = scenario.generate(maxIterations=n)
                n -= its
        except RejectionException:
            pass

        times.append(time.perf_counter() - t0)

    return math.fsum(times)


def generateScenes(loops, path, count):
    options = parseOptions(path)
    times = []

    for _ in range(loops):
        # Recompile each time to reset caches, checker state, etc.
        scenario = scenic.scenarioFromFile(path, **options)

        t0 = time.perf_counter()

        for _ in range(count):
            scenario.generate()

        times.append(time.perf_counter() - t0)

    return math.fsum(times)


benchmarkTypes = {
    "compile": ("bench_func", scenic.scenarioFromFile),
    "sample10": ("bench_time_func", sampleForIterations, 10),
    "sample100": ("bench_time_func", sampleForIterations, 100),
    "scene10": ("bench_time_func", generateScenes, 10),
    "scene100": ("bench_time_func", generateScenes, 100),
}
defaultTypes = ("compile", "scene10")
for ty in defaultTypes:
    assert ty in benchmarkTypes


if __name__ == "__main__":

    def add_cmdline_args(cmd, args):
        cmd.extend(args.benchmarks)
        if args.types:
            cmd.append("--types")
            cmd.extend(args.types)

    runner = pyperf.Runner(add_cmdline_args=add_cmdline_args)
    types = ["all"]
    types.extend(benchmarkTypes)
    runner.argparser.add_argument(
        "--types", nargs="+", action="extend", metavar="TYPE", choices=types
    )
    runner.argparser.add_argument(
        "benchmarks", nargs="*", type=Path, help="benchmarks to run (default all)"
    )
    runner.parse_args()
    benchmarks = runner.args.benchmarks
    if not benchmarks:
        benchmarks = sorted(Path(__file__).parent.glob("*.scenic"))
    types = runner.args.types
    if not types:
        types = defaultTypes
    elif "all" in types:
        types = tuple(benchmarkTypes)

    if not sys.warnoptions:
        warnings.simplefilter("ignore")

    for path in benchmarks:
        name = path.stem
        for ty in types:
            benchTy, func, *args = benchmarkTypes[ty]
            method = getattr(runner, benchTy)
            method(f"{name}_{ty}", func, path, *args)
