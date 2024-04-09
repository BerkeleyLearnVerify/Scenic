import os
import requests
import re

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
        print(f"A new {simulator} version ({version}) needs to be installed and tested in CI.\n")

print("Checking for CARLA...")
carla_url = 'https://github.com/carla-simulator/carla/releases/latest'
carla_response = requests.get(carla_url)
carla_match = re.search(r'carla-simulator/carla/releases/tag/([^"]+)', carla_response.text)

try:
    version = carla_match.group(1)
    check_path_exists(version, "CARLA_")
except AttributeError:
    print("Error: Unable to find the latest CARLA version using regex.")

print("Checking for Webots...")
webots_url = 'https://github.com/cyberbotics/webots/releases/latest'
webots_response = requests.get(webots_url)
webots_match = re.search(r'cyberbotics/webots/releases/tag/([^"]+)', webots_response.text)

try:
    version = webots_match.group(1)
    check_path_exists(version, 'webots')
except AttributeError:
    print("Error: Unable to find the latest Webots version using regex.")