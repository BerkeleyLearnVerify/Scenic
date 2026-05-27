import socket
import json
import os
import re
import subprocess
import time
import shutil
from os import path
import platform
from contextlib import closing
from types import SimpleNamespace
import sys
import zipfile
import threading
from threading import Event
from http.server import SimpleHTTPRequestHandler, HTTPServer
from datetime import datetime

"""
Helper functions for METSR-SIM and METSR-HPC
"""

# Function for checking whether the socket connection is on
def check_socket(host, port):
    flag = True
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex((host, port)) == 0:
            flag =  True
        else:
            flag =  False
    time.sleep(1)
    return flag

# Factory for processsing a str list with a given func
def str_list_mapper_gen(func):
    def str_list_mapper(str_list):
        return [func(str) for str in str_list]
    return str_list_mapper

_PROPERTY_RE = re.compile(r"^(\s*)([A-Za-z0-9_]+)\s*=\s*(.*?)(\r?\n?)$")
_MISSING = object()

_PROPERTY_OPTION_ALIASES = {
    "SIMULATION_STEP_SIZE": ("sim_step_size",),
    "ENABLE_JSON_WRITE": ("json_output",),
    "NUM_OF_EV": ("num_etaxi",),
    "NUM_OF_BUS": ("num_ebus",),
    "RH_SHARE_PERCENTAGE": ("rh_share_file",),
    "RH_WAITING_TIME": ("rh_wait_file",),
    "BT_STD_FILE": ("bt_event_std_file",),
    "EV_DEMAND_FILE": ("private_ev_demand_file",),
    "GV_DEMAND_FILE": ("private_gv_demand_file",),
    "EV_CHARGING_PREFERENCE": ("private_ev_charging_preference",),
}


def _camel_to_snake(name):
    """Return a config-friendly lowercase form for mixed-case property names."""
    name = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name)
    return name.lower()


def _config_names_for_property(property_name):
    names = [property_name.lower(), _camel_to_snake(property_name)]
    names.extend(_PROPERTY_OPTION_ALIASES.get(property_name, ()))

    deduped = []
    for name in names:
        if name not in deduped:
            deduped.append(name)
    return deduped


def _get_option(options, name):
    if isinstance(options, dict):
        return options.get(name, _MISSING)
    return getattr(options, name, _MISSING)


def _first_option_value(options, names):
    for name in names:
        value = _get_option(options, name)
        if value is not _MISSING and value is not None:
            return True, value
    return False, None


def _format_property_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _property_line(key, value, newline):
    newline = newline or "\n"
    return f"{key} = {_format_property_value(value)}{newline}"


def _ensure_extension(value, extension):
    value = str(value)
    if value.lower().endswith(extension):
        return value
    return value + extension


def _rewrite_data_path(line, src_data_dir):
    if "data/" not in line:
        return line
    src_data_dir = src_data_dir.replace("\\", "/").rstrip("/")
    return line.replace("data/", src_data_dir + "/")


def _property_override_value(key, options, port, instance):
    """Find the value that should be written for a Data.properties key.

    Most properties are now mapped automatically from the lowercase key name
    used in JSON configs, for example CAR_FOLLOWING_MODEL -> car_following_model.
    Existing HPC config names are kept as aliases so old configs still work.
    """
    if key == "NETWORK_LISTEN_PORT":
        return True, port
    if key == "RANDOM_SEED":
        found, seeds = _first_option_value(options, ("random_seeds",))
        if found:
            return True, seeds[instance]
        return False, None
    if key == "STANDALONE":
        return True, False
    if key == "SYNCHRONIZED":
        return True, True
    if key == "AGG_DEFAULT_PATH":
        return True, "agg_output"
    if key == "JSON_DEFAULT_PATH":
        return True, "trajectory_output"

    if key == "ZONES_SHAPEFILE":
        found, value = _first_option_value(options, ("zones_shapefile",))
        if found:
            return True, value
        found, value = _first_option_value(options, ("zone_file",))
        if found:
            return True, _ensure_extension(value, ".shp")
    elif key == "ZONES_CSV":
        found, value = _first_option_value(options, ("zones_csv",))
        if found:
            return True, value
        found, value = _first_option_value(options, ("zone_file",))
        if found:
            return True, _ensure_extension(value, ".csv")
    elif key == "CHARGER_SHAPEFILE":
        found, value = _first_option_value(options, ("charger_shapefile",))
        if found:
            return True, value
        found, value = _first_option_value(options, ("charging_station_file",))
        if found:
            return True, _ensure_extension(value, ".shp")
    elif key == "CHARGER_CSV":
        found, value = _first_option_value(options, ("charger_csv",))
        if found:
            return True, value
        found, value = _first_option_value(options, ("charging_station_file",))
        if found:
            return True, _ensure_extension(value, ".csv")
    elif key == "RH_DEMAND_SHARABLE":
        found, value = _first_option_value(options, _config_names_for_property(key))
        if found:
            return True, value
        found, _ = _first_option_value(options, ("rh_wait_file", "rh_waiting_time"))
        if found:
            return True, True
        return False, None

    return _first_option_value(options, _config_names_for_property(key))


# Function for modifying simulation properties
def modify_property_file(options, src_data_dir, dest_data_dir, port, instance, template):
    fname = src_data_dir + "/Data.properties." + template
    if not path.exists(fname):
        print("ERROR, cannot find the property template file at ", fname)
        sys.exit(-1)

    f = open(fname, "r")
    lines = f.readlines()
    f.close()
    fname = dest_data_dir + "/Data.properties"
    f_new = open(fname, "w")
    for l in lines:
        match = _PROPERTY_RE.match(l)
        if match:
            _, key, _, newline = match.groups()
            found, value = _property_override_value(key, options, port, instance)
            if found:
                l = _property_line(key, value, newline)
        l = _rewrite_data_path(l, src_data_dir)
        
        f_new.write(l)
    f_new.close()

def force_copytree(src, dst):
    """
    Recursively copy a directory tree, overwriting the destination directory if it exists.
    """
    # Check if the destination directory exists
    if os.path.exists(dst):
        # Remove the destination directory and all its contents
        shutil.rmtree(dst)
    
    # Copy the source directory to the destination
    shutil.copytree(src, dst)

# Copy necessary files for running the simulation
def prepare_sim_dirs(options):
    src_data_dir = "data"
    # check if metsr_port in the NameSpace options
    if hasattr(options, 'metsr_port'):
        # check if metsr_port number is equal to the number of simulations
        if options.num_simulations > len(options.metsr_port):
            print("ERROR , port number is less than the number of simulation instances")
            sys.exit(-1)
        else:
            options.ports = options.metsr_port
    else:
        print("No port number specified, find available ports for simulation instances")
        find_free_ports(options, options.num_simulations)
    if len(options.ports) != options.num_simulations:
        print("ERROR , cannot specify port number for all simulation instances")
        sys.exit(-1)


    dest_data_dirs = []
    options.sim_dirs = []
    for i in range(options.num_simulations):
        # make a directory to run the simulator
        dir_name = get_sim_dir(options, i)
        if not path.exists(dir_name):
            os.makedirs(dir_name)
        options.sim_dirs.append(dir_name)
        shutil.copy(src_data_dir+"/log4j.properties", dir_name + "/log4j.properties")
        # copy the simulation config files
        dest_data_dir = dir_name + "/" + "data"

        if not path.exists(dest_data_dir):
            os.mkdir(dest_data_dir)
            # copy the entire data directory
            force_copytree(src_data_dir, dest_data_dir)

        modify_property_file(options, src_data_dir, dest_data_dir, options.ports[i], i, options.template)
        dest_data_dirs.append(dest_data_dir[:-5]) # -5 to remove the "/data" part

    return dest_data_dirs

# Function for getting the file name list of demand scenarios
# def prepare_scenario_dict(options, path):
#     scenarios = os.listdir(path)
#     i = 0
#     scenarios = sorted(scenarios)
#     options.scenarios=[]
#     options.cases = [[] for j in range(len(scenarios))]
#     for scenario in scenarios:
#         options.scenarios.append(scenario)
#         cases = os.listdir(path+"/"+scenario)
#         cases = sorted(cases)
#         for case in cases:
#             options.cases[i].append(case.split("_")[1])
#         i+=1

# Functions for finding available port
def find_free_ports(options, num_simulations):
    options.ports = []
    while True:
        for i in range(num_simulations):
            with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
                s.bind(('localhost', 0))
                options.ports.append(s.getsockname()[1])
        try:
            for port in options.ports:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.bind(('', port))
                s.close()
            break
        except:
            print("The port is not valid anymore, regenerate it")
            continue
    time.sleep(1)
     
# Read json format configuration 
def _load_raw_config(fname):
    """Recursively load a config JSON, merging parent_config fields first."""
    fname = os.path.abspath(fname)
    with open(fname, "r") as f:
        raw = json.load(f)

    if "parent_config" in raw:
        parent_path = os.path.abspath(
            os.path.join(os.path.dirname(fname), raw["parent_config"])
        )
        parent_raw = _load_raw_config(parent_path)
        child_fields = {k: v for k, v in raw.items() if k != "parent_config"}
        return {**parent_raw, **child_fields}

    return raw


def read_run_config(fname):
    merged = _load_raw_config(fname)
    config = SimpleNamespace(**merged)

    if len(config.random_seeds) != config.num_simulations:
       print("ERROR, please specify random seeds for all simulation instances")
       sys.exit(-1)

    return config

# Construct the java classpath with all the required jar files. 
# If includeBin is False it won't add the METS_R/bin directory to classpath.
# This is needed for simulation command.
def get_classpath(options, includeBin=True, separator=":"):
    
    classpath = ""

    if not path.exists(options.repast_plugin_dir):
        print(f"ERROR , repast plugins not found at {options.repast_plugin_dir}")
        sys.exit(-1)
    
    classpath += options.repast_plugin_dir + "repast.simphony.runtime_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.runtime_2.7.0/lib/*" + separator + \
                 options.sim_dir + "lib/*" + separator    
 
    


    return classpath

def get_classpath2(options, includeBin=True, separator=":"):
    
    classpath = ""

    classpath += options.repast_plugin_dir + "repast.simphony.runtime_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.runtime_2.7.0/lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.batch_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.batch_2.7.0/lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.distributed.batch_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.distributed.batch_2.7.0/lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.core_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.core_2.7.0/lib/*" + separator + \
                 options.sim_dir + "bin" + separator + \
                 options.sim_dir + "lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.bin_and_src_2.7.0/repast.simphony.bin_and_src.jar" + separator + \
                 options.repast_plugin_dir + "repast.simphony.essentials_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.gis_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.gis_2.7.0/lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.sql_2.7.0/bin" + separator + \
                 options.repast_plugin_dir + "repast.simphony.sql_2.7.0/lib/*" + separator + \
                 options.repast_plugin_dir + "repast.simphony.scenario_2.7.0/bin" + separator 

    return classpath

# Function for starting the simulation
def run_simulations(options):
    for i in range(0, options.num_simulations):
        cwd = str(os.getcwd())
        if platform.system() == "Windows":
             # go to sim directory
            os.chdir(options.sim_dirs[i])

            # print(get_classpath(options, False, separator = ";"))
            # run the simulation on a new terminal
            sim_command = '"' + options.java_path + 'java"' + " " + \
                    options.java_options + " " + \
                    "-classpath " + \
                    '"' +get_classpath(options, False, separator = ";") + '" '  + \
                    "repast.simphony.runtime.RepastMain " + \
                    options.sim_dir + "mets_r.rs"
            # print(sim_command)
            if options.verbose: # print the sim output to the console
                subprocess.Popen(sim_command, shell=True)
            else:
                subprocess.Popen(sim_command + " > sim_{}.log 2>&1 &".format(i), shell=True)
        else:
            # go to sim directory
            os.chdir(options.sim_dirs[i])
            # run simulator on new terminal
            sim_command = options.java_path + "java " + \
                    options.java_options + " " + \
                    "-classpath " + \
                    get_classpath(options, False) + " "  + \
                    "repast.simphony.runtime.RepastMain " + \
                    options.sim_dir + "mets_r.rs"
            if options.verbose:
                os.system(sim_command)
            else:
                os.system(sim_command + " > sim_{}.log 2>&1 &".format(i))
        # go back to test directory
        os.chdir(cwd)

def run_simulations_in_background(options):
    for i in range(0, options.num_simulations):
        cwd = str(os.getcwd())
        if platform.system() == "Windows":
             # go to sim directory
            os.chdir(options.sim_dirs[i])
            # run the simulation on a new terminal
            sim_command = '"' +  options.java_path + 'java"'+ " -Xmx16G "  + \
                    "-cp " + \
                    '"' + get_classpath2(options, False, separator = ";") + '" ' + \
                    "repast.simphony.batch.BatchMain " + \
                    "-params " + options.sim_dir + "mets_r.rs/batch_params.xml " +\
                    "-interactive " + options.sim_dir + "mets_r.rs "
            # print(sim_command)
            if options.verbose: # print the sim output to the console
                subprocess.Popen(sim_command)
            else:
                subprocess.Popen(sim_command + " > sim_{}.log 2>&1 &".format(i), shell=True)
        else:
            # go to sim directory
            os.chdir(options.sim_dirs[i])
            # run simulator on new terminal 
            sim_command = '' +  options.java_path + 'java'+ " -Xmx16G "  + \
                    "-cp " + \
                    get_classpath2(options, False) + ' ' + \
                    "repast.simphony.batch.BatchMain " + \
                    "-params " + options.sim_dir + "mets_r.rs/batch_params.xml " +\
                    "-interactive " + options.sim_dir + "mets_r.rs "
            if options.verbose:
                os.system(sim_command)
            else:
                os.system(sim_command + " > sim_{}.log 2>&1 &".format(i))
        # go back to test directory
        os.chdir(cwd)

def run_simulation_in_docker(options):
    for i in range(0, options.num_simulations):
        cwd = str(os.getcwd())
        os.chdir(options.sim_dirs[i])

        sim_command = '' +  options.java_path + 'java'+ " -Xmx16G "  + \
            "-cp " + \
            get_classpath2(options, False) + ' ' + \
            "repast.simphony.batch.BatchMain " + \
            "-params " + options.sim_dir + "mets_r.rs/batch_params.xml " +\
            "-interactive " + options.sim_dir + "mets_r.rs"
        
        docker_command = f'docker run -d --rm --mount src="{os.getcwd()}",target=/home/test,type=bind --net=host ennuilei/mets-r_sim  /bin/bash -c "cd /home/test && ' + sim_command + '"'
        result = subprocess.run(docker_command, shell=True, text=True, capture_output=True)
        if options.verbose:
            print("Container ID:", result.stdout)
            # print("Error msg:", result.stderr)
        # container_id = result.stdout.strip()
        container_id = result.stdout.strip()
        os.chdir(cwd)
        return container_id
        # os.chdir(cwd)

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, **kwargs):
            self.custom_directory = directory
            super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.end_headers()

def start_cors_http_server(directory, stop_event, port=8000):
    """Start a CORS-enabled HTTP server for the specified directory."""
    handler_class = lambda *args, **kwargs: CORSRequestHandler(*args, directory=directory, **kwargs)
    server_address = ('', port)
    httpd = HTTPServer(server_address, handler_class)

    def run_server():
        while not stop_event.is_set():
            httpd.handle_request()
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    return server_thread

def run_visualization_server(data_folder, server_port = 8000):
    # store the current work directory
    # workdir = os.getcwd()
    # Ensure the data folder exists
    if not os.path.exists(data_folder):
        os.makedirs(data_folder)
        print(f"Created data folder: {data_folder}")
    
    # Start the HTTP server in a separate thread
    # os.chdir(data_folder)  # Change to the specified directory
    stop_event = Event() 
    server_thread = start_cors_http_server(data_folder, stop_event, server_port)
    print(f"Serving {data_folder} with CORS enabled on port {server_port}...")

    # recovery the work directory
    # os.chdir(workdir)

    return stop_event, server_thread

def stop_visualization_server(stop_event, server_thread, port=8000):
    stop_event.set()

    # Send dummy request to unblock handle_request()
    try:
        with socket.create_connection(("localhost", port), timeout=1) as sock:
            sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
    except Exception as e:
        print(f"Dummy request to unblock server failed (probably fine): {e}")

    server_thread.join()
    print("Visualization server stopped.")

# Get the directory for storing simulation outputs
def get_sim_dir(options, i):
    sim_dir = "output/"+ options.name + "_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_seed_" + str(options.random_seeds[i])
    return sim_dir

# 
