import os
import re

import requests

SIMULATORS = ["carla", "webots"]
SIMULATOR_URLS = {
    "carla": "https://github.com/carla-simulator/carla/releases/latest",
    "webots": "https://github.com/cyberbotics/webots/releases/latest",
}
SIMULATORS_REGEXES = {
    "carla": 'carla-simulator/carla/releases/tag/([^"]+)',
    "webots": 'cyberbotics/webots/releases/tag/([^"]+)',
}


def check_path_exists(version, simulator):
    path = f"/software/{simulator}{version}"
    if os.path.exists(path):
        print(f"Latest {simulator} version {version} already present on the machine.\n")
    else:
        s = """
                       _          
 _    _____ ________  (_)__  ___ _
| |/|/ / _ `/ __/ _ \/ / _ \/ _ `/
|__,__/\_,_/_/ /_//_/_/_//_/\_, / 
                           /___/  
        """
        print(s)
        print(
            f"A new {simulator} version ({version}) needs to be installed and tested in CI.\n"
        )


def version_check(regex_match, sim_name):
    try:
        version = regex_match.group(1)
        check_path_exists(version, sim_name)
    except AttributeError:
        print(f"Error: Unable to find the latest {sim_name} version using regex.")


for sim in SIMULATORS:
    print(f"Checking for {sim}...")
    url = SIMULATOR_URLS[sim]
    response = requests.get(url)
    regex_match = re.search(SIMULATORS_REGEXES[sim], response.text)
    version_check(regex_match, sim)
