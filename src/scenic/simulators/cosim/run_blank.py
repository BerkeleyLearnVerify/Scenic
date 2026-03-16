import sys
import os
import argparse
import time
from utils.util import read_run_config, prepare_sim_dirs, run_simulations, run_simulations_in_background, run_simulation_in_docker 

# use case: python run_blank.py -r configs/run_cosim_blank.json -v
def get_arguments(argv):
    parser = argparse.ArgumentParser(description='METS-R simulation')
    parser.add_argument('-r','--run_config', default='configs/run_cosim_CARLAT5.json',
                        help='the folder that contains all the input data')
    parser.add_argument('-v', '--verbose', action='store_true', default=False,
                        help='verbose mode')
    parser.add_argument('-n', '--name', default="random_choice")
    args = parser.parse_args(argv)

    config = read_run_config(args.run_config)
    config.verbose = args.verbose

    return config

if __name__ == '__main__':
    config = get_arguments(sys.argv[1:])
    os.chdir("docker")
    os.system("docker-compose up -d")
    os.chdir("..")

    time.sleep(10) # wait 10s for the Kafka servers to be up

    # Prepare simulation directories
    dest_data_dirs = prepare_sim_dirs(config)

    try:
        # Launch the simulations
        container_ids = run_simulation_in_docker(config)
        print("Docker Started Successfully")
        while True:
            time.sleep(1)
    finally:
        for cid in container_ids:
            print(f"Stopping docker cid: {cid}")
            os.system(f"docker stop {cid}")
