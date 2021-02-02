import os
import subprocess
import argparse
import json

parser = argparse.ArgumentParser(prog='scenic', add_help=False,
                                 usage='scenic [-h | --help] [options] FILE [options]',
                                 description='Sample from a Scenic scenario, optionally '
                                             'running dynamic simulations.')

parser.add_argument('--scenarios', help='path to scenario configuration file')
parser.add_argument('--sensors', help='path to sensor configuration file')

args = parser.parse_args()

with open(args.scenarios, 'r') as f:
	scenario_config = json.load(f)

for scenic_script_fpath in scenario_config['scripts']:
	scenario_fname = os.path.basename(scenic_script_fpath)
	scenario_name = scenario_fname.split('.')[0]

	scenario_recording_dir = os.path.join(scenario_config['output_dir'], scenario_name)

	if not os.path.isdir(scenario_recording_dir):
		os.mkdir(scenario_recording_dir)

	for _ in range(scenario_config['simulations_per_scenario']):
		command = 'python -m scenic -S --time {} --count 1 -r --sensors {} --recording_dir {} -m scenic.simulators.carla.model {}'.format(
			scenario_config['time_per_simulation'],
			args.sensors,
			scenario_recording_dir,
			scenic_script_fpath,
		)

		subprocess.call(command, shell=True)