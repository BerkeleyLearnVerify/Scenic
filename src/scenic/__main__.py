
### Top-level functionality of the scenic package as a script:
### load a scenario and generate scenes in an infinite loop.

import sys
import time
import argparse
import random

if sys.version_info >= (3, 8):
    from importlib import metadata
else:
    import importlib_metadata as metadata

import scenic.syntax.translator as translator
import scenic.core.errors as errors
from scenic.core.simulators import SimulationCreationError

parser = argparse.ArgumentParser(prog='scenic', add_help=False,
                                 usage='scenic [-h | --help] [options] FILE [options]',
                                 description='Sample from a Scenic scenario, optionally '
                                             'running dynamic simulations.')

mainOptions = parser.add_argument_group('main options')
mainOptions.add_argument('-S', '--simulate', action='store_true',
                         help='run dynamic simulations from scenes '
                              'instead of simply showing diagrams of scenes')
mainOptions.add_argument('-s', '--seed', help='random seed', type=int)
mainOptions.add_argument('-v', '--verbosity', help='verbosity level (default 1)',
                         type=int, choices=(0, 1, 2, 3), default=1)
mainOptions.add_argument('-p', '--param', help='override a global parameter',
                         nargs=2, default=[], action='append', metavar=('PARAM', 'VALUE'))
mainOptions.add_argument('-m', '--model', help='specify a Scenic world model', default=None)
mainOptions.add_argument('--scenario', default=None,
                         help='name of scenario to run (if file contains multiple)')

# Simulation options
simOpts = parser.add_argument_group('dynamic simulation options')
simOpts.add_argument('--time', help='time bound for simulations (default none)',
                     type=int, default=None)
simOpts.add_argument('--count', help='number of successful simulations to run (default infinity)',
                     type=int, default=0)
simOpts.add_argument('--max-sims-per-scene', type=int, default=1, metavar='N',
                     help='max # of rejected simulations before sampling a new scene (default 1)')

# Interactive rendering options
intOptions = parser.add_argument_group('static scene diagramming options')
intOptions.add_argument('-d', '--delay', type=float,
                        help='loop automatically with this delay (in seconds) '
                             'instead of waiting for the user to close the diagram')
intOptions.add_argument('-z', '--zoom', help='zoom expansion factor (default 1)',
                        type=float, default=1)

# Debugging options
debugOpts = parser.add_argument_group('debugging options')
debugOpts.add_argument('--show-params', help='show values of global parameters',
                       action='store_true')
debugOpts.add_argument('--show-records', help='show values of recorded expressions',
                       action='store_true')
debugOpts.add_argument('-b', '--full-backtrace', help='show full internal backtraces',
                       action='store_true')
debugOpts.add_argument('--pdb', action='store_true',
                       help='enter interactive debugger on errors (implies "-b")')
debugOpts.add_argument('--pdb-on-reject', action='store_true',
                       help='enter interactive debugger on rejections (implies "-b")')
ver = metadata.version('scenic')
debugOpts.add_argument('--version', action='version', version=f'Scenic {ver}',
                       help='print Scenic version information and exit')
debugOpts.add_argument('--dump-initial-python', help='dump initial translated Python',
                       action='store_true')
debugOpts.add_argument('--dump-ast', help='dump final AST', action='store_true')
debugOpts.add_argument('--dump-python', help='dump Python equivalent of final AST',
                       action='store_true')
debugOpts.add_argument('--no-pruning', help='disable pruning', action='store_true')
debugOpts.add_argument('--gather-stats', type=int, metavar='N',
                       help='collect timing statistics over this many scenes')

parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
                    help=argparse.SUPPRESS)

# Positional arguments
parser.add_argument('scenicFile', help='a Scenic file to run', metavar='FILE')

# Parse arguments and set up configuration
args = parser.parse_args()
delay = args.delay
errors.showInternalBacktrace = args.full_backtrace
if args.pdb:
    errors.postMortemDebugging = True
    errors.showInternalBacktrace = True
if args.pdb_on_reject:
    errors.postMortemRejections = True
    errors.showInternalBacktrace = True
params = {}
for name, value in args.param:
    # Convert params to ints or floats if possible
    try:
        value = int(value)
    except ValueError:
        try:
            value = float(value)
        except ValueError:
            pass
    params[name] = value
translator.dumpTranslatedPython = args.dump_initial_python
translator.dumpFinalAST = args.dump_ast
translator.dumpASTPython = args.dump_python
translator.verbosity = args.verbosity
translator.usePruning = not args.no_pruning
if args.seed is not None and args.verbosity >= 1:
    print(f'Using random seed = {args.seed}')
    random.seed(args.seed)

# Load scenario from file
if args.verbosity >= 1:
    print('Beginning scenario construction...')
startTime = time.time()
scenario = errors.callBeginningScenicTrace(
    lambda: translator.scenarioFromFile(args.scenicFile,
                                        params=params,
                                        model=args.model,
                                        scenario=args.scenario)
)
totalTime = time.time() - startTime
if args.verbosity >= 1:
    print(f'Scenario constructed in {totalTime:.2f} seconds.')

if args.simulate:
    simulator = errors.callBeginningScenicTrace(scenario.getSimulator)

def generateScene():
    startTime = time.time()
    scene, iterations = errors.callBeginningScenicTrace(
        lambda: scenario.generate(verbosity=args.verbosity)
    )
    if args.verbosity >= 1:
        totalTime = time.time() - startTime
        print(f'  Generated scene in {iterations} iterations, {totalTime:.4g} seconds.')
        if args.show_params:
            for param, value in scene.params.items():
                print(f'    Parameter "{param}": {value}')
    return scene, iterations

def runSimulation(scene):
    startTime = time.time()
    if args.verbosity >= 1:
        print(f'  Beginning simulation of {scene.dynamicScenario}...')
    try:
        simulation = errors.callBeginningScenicTrace(
            lambda: simulator.simulate(scene, maxSteps=args.time, verbosity=args.verbosity,
                                       maxIterations=args.max_sims_per_scene)
        )
    except SimulationCreationError as e:
        if args.verbosity >= 1:
            print(f'  Failed to create simulation: {e}')
        return False
    if args.verbosity >= 1:
        totalTime = time.time() - startTime
        print(f'  Ran simulation in {totalTime:.4g} seconds.')
    if simulation and args.show_records:
        for name, value in simulation.result.records.items():
            if isinstance(value, list):
                print(f'    Record "{name}": (time series)')
                for step, subval in value:
                    print(f'      {step:4d}: {subval}')
            else:
                print(f'    Record "{name}": {value}')
    return simulation is not None

try:
    if args.gather_stats is None:   # Generate scenes interactively until killed
        import matplotlib.pyplot as plt
        successCount = 0
        while True:
            scene, _ = generateScene()
            if args.simulate:
                success = runSimulation(scene)
                if success:
                    successCount += 1
                    if 0 < args.count <= successCount:
                        break
            else:
                if delay is None:
                    scene.show(zoom=args.zoom)
                else:
                    scene.show(zoom=args.zoom, block=False)
                    plt.pause(delay)
                    plt.clf()
    else:   # Gather statistics over the specified number of scenes
        its = []
        startTime = time.time()
        while len(its) < args.gather_stats:
            scene, iterations = generateScene()
            its.append(iterations)
        totalTime = time.time() - startTime
        count = len(its)
        print(f'Sampled {len(its)} scenes in {totalTime:.2f} seconds.')
        print(f'Average iterations/scene: {sum(its)/count}')
        print(f'Average time/scene: {totalTime/count:.2f} seconds.')

except KeyboardInterrupt:
    pass

finally:
    if args.simulate:
        simulator.destroy()

def dummy():    # for the 'scenic' entry point to call after importing this module
    pass
