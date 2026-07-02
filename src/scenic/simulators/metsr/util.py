"""Helper functions for METSR-SIM and METSR-HPC."""

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
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from datetime import datetime


# ---------------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------------

def check_socket(host, port):
    flag = True
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex((host, port)) == 0:
            flag =  True
        else:
            flag =  False
    time.sleep(1)
    return flag


def str_list_mapper_gen(func):
    def str_list_mapper(str_list):
        return [func(str) for str in str_list]
    return str_list_mapper


def _is_sequence(value):
    return isinstance(value, (list, tuple))


def _as_list(value):
    if isinstance(value, (list, tuple)):
        return list(value)
    return [value]


def _broadcast(value, length):
    if length == 1:
        return [value]
    if _is_sequence(value) and len(value) == length:
        return list(value)
    return [value] * length


# ---------------------------------------------------------------------------
# METSRClient payload helpers
# ---------------------------------------------------------------------------

VEHICLE_SENSOR_DSRC = 0
VEHICLE_SENSOR_CV2X = 1
VEHICLE_SENSOR_MOBILE_DEVICE = 2

VEHICLE_SENSOR_TYPES = {
    "dsrc": VEHICLE_SENSOR_DSRC,
    "80211p": VEHICLE_SENSOR_DSRC,
    "cv2x": VEHICLE_SENSOR_CV2X,
    "c-v2x": VEHICLE_SENSOR_CV2X,
    "c_v2x": VEHICLE_SENSOR_CV2X,
    "mobile": VEHICLE_SENSOR_MOBILE_DEVICE,
    "mobiledevice": VEHICLE_SENSOR_MOBILE_DEVICE,
    "mobile_device": VEHICLE_SENSOR_MOBILE_DEVICE,
    "mobile-device": VEHICLE_SENSOR_MOBILE_DEVICE,
}

REQUEST_ID_FIELDS = ("reqID", "requestID", "requestId", "ID", "id")
REQUEST_ZONE_FIELDS = (
    "zoneID",
    "zoneId",
    "zone",
    "originZone",
    "origZone",
    "origin",
    "orig",
)


def _request_id_from_record(record):
    if not isinstance(record, dict):
        return record
    for key in REQUEST_ID_FIELDS:
        value = record.get(key)
        if value is not None:
            return value
    return None


def _request_zone_from_record(record):
    if not isinstance(record, dict):
        return None
    for key in REQUEST_ZONE_FIELDS:
        value = record.get(key)
        if value is None:
            continue
        if isinstance(value, dict):
            nested_value = _request_zone_from_record(value)
            if nested_value is not None:
                return nested_value
            continue
        return value
    return None


def _normalize_sensor_type(sensor_type):
    if isinstance(sensor_type, str):
        key = sensor_type.strip().lower()
        compact_key = key.replace(" ", "").replace("_", "").replace("-", "")
        if key in VEHICLE_SENSOR_TYPES:
            return VEHICLE_SENSOR_TYPES[key]
        if compact_key in VEHICLE_SENSOR_TYPES:
            return VEHICLE_SENSOR_TYPES[compact_key]
        raise ValueError(
            "Unknown sensorType. Use 0/'dsrc', 1/'cv2x', or 2/'mobile_device'."
        )
    return sensor_type


def _looks_like_centerline(value):
    if not _is_sequence(value) or len(value) == 0:
        return False
    first_point = value[0]
    if not _is_sequence(first_point) or len(first_point) < 2:
        return False
    return not _is_sequence(first_point[0])


def _set_road_reference(record, field_prefix, value):
    if value is None:
        return
    if _is_sequence(value) and not isinstance(value, str):
        record[field_prefix + "Roads"] = list(value)
    else:
        record[field_prefix + "Road"] = value


# ---------------------------------------------------------------------------
# Simulation property/config helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Simulation file preparation
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Port and config helpers
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Java classpath helpers
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Simulation launch helpers
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Visualization server helpers
# ---------------------------------------------------------------------------

class CORSRequestHandler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def __init__(self, *args, directory=None, **kwargs):
            self.custom_directory = directory
            super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        if self.path.rstrip('/').endswith('manifest.json'):
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
        super().end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.end_headers()


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 64


def start_cors_http_server(directory, stop_event, port=8000):
    """Start a CORS-enabled HTTP server for the specified directory."""
    handler_class = lambda *args, **kwargs: CORSRequestHandler(*args, directory=directory, **kwargs)
    server_address = ('', port)
    httpd = ReusableThreadingHTTPServer(server_address, handler_class)
    httpd.timeout = 0.5

    def run_server():
        try:
            httpd.serve_forever(poll_interval=0.2)
        finally:
            httpd.server_close()
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.httpd = httpd
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

def stop_visualization_server(stop_event, server_thread, port=8000, join_timeout=2.0):
    stop_event.set()
    httpd = getattr(server_thread, "httpd", None)
    if httpd is not None:
        httpd.shutdown()

    # Send dummy request to unblock handle_request()
    if httpd is None:
        try:
            with socket.create_connection(("localhost", port), timeout=1) as sock:
                sock.sendall(b"HEAD / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        except Exception:
            pass

    server_thread.join(timeout=join_timeout)
    if server_thread.is_alive():
        print(f"Visualization server thread did not stop within {join_timeout:.1f} seconds.")
    else:
        print("Visualization server stopped.")


def _simulation_folder_from_config(config, sim_index=0, output_root="output"):
    sim_dirs = getattr(config, "sim_dirs", None)
    if sim_dirs:
        try:
            return sim_dirs[sim_index]
        except IndexError as exc:
            raise IndexError(
                f"sim_index {sim_index} is outside config.sim_dirs with {len(sim_dirs)} entries"
            ) from exc

    sim_folder = getattr(config, "sim_folder", None)
    if sim_folder:
        if isinstance(sim_folder, (list, tuple)):
            try:
                return sim_folder[sim_index]
            except IndexError as exc:
                raise IndexError(
                    f"sim_index {sim_index} is outside config.sim_folder with {len(sim_folder)} entries"
                ) from exc
        return sim_folder

    sim_dir = getattr(config, "sim_dir", None)
    if sim_dir:
        sim_dir_candidates = sim_dir if isinstance(sim_dir, (list, tuple)) else [sim_dir]
        try:
            sim_dir_candidate = sim_dir_candidates[sim_index]
        except IndexError as exc:
            raise IndexError(
                f"sim_index {sim_index} is outside config.sim_dir with {len(sim_dir_candidates)} entries"
            ) from exc
        if _folder_has_trajectory_output(sim_dir_candidate):
            return sim_dir_candidate

    run_name = getattr(config, "name", None)
    if not run_name:
        raise ValueError(
            "Cannot infer simulation output folder: config needs sim_dirs, sim_folder, or a name."
        )

    seeds = getattr(config, "random_seeds", None) or []
    seed = seeds[sim_index] if sim_index < len(seeds) else None
    output_root = os.path.abspath(output_root)
    try:
        names = os.listdir(output_root)
    except OSError as exc:
        raise FileNotFoundError(f"Simulation output root does not exist: {output_root}") from exc

    prefix = f"{run_name}_"
    seed_suffix = f"_seed_{seed}" if seed is not None else None
    candidates = []
    for name in names:
        if not name.startswith(prefix):
            continue
        if seed_suffix is not None and not name.endswith(seed_suffix):
            continue
        candidate = os.path.join(output_root, name)
        if os.path.isdir(candidate):
            candidates.append(candidate)

    if not candidates:
        seed_text = f" and seed {seed}" if seed is not None else ""
        raise FileNotFoundError(
            f"No finished simulation output folder found for config name {run_name!r}{seed_text} under {output_root}"
        )

    return max(candidates, key=os.path.getmtime)


def _folder_has_trajectory_output(sim_folder):
    if not sim_folder or not os.path.isdir(sim_folder):
        return False
    for root in _configured_trajectory_roots(sim_folder):
        if _latest_trajectory_directory(root, prefer_binary=False) is not None:
            return True
    return False


def latest_trajectory_output_dir_from_config(
        config,
        sim_index=0,
        trajectory_output_dir=None,
        prefer_binary=True,
        wait_seconds=0,
        output_root="output"):
    sim_folder = _simulation_folder_from_config(
        config,
        sim_index=sim_index,
        output_root=output_root,
    )
    if trajectory_output_dir is not None:
        roots = [_resolve_trajectory_root(sim_folder, trajectory_output_dir)]
    else:
        roots = _configured_trajectory_roots(sim_folder)

    deadline = time.time() + max(0, float(wait_seconds or 0))
    while True:
        for root in roots:
            latest_directory = _latest_trajectory_directory(root, prefer_binary=prefer_binary)
            if latest_directory is not None:
                return latest_directory

        if time.time() >= deadline:
            break
        time.sleep(0.5)

    roots_text = ", ".join(str(root) for root in roots if root)
    raise FileNotFoundError("No trajectory output directory found under " + roots_text)


def start_visualization_server_from_config(
        config,
        sim_index=0,
        trajectory_output_dir=None,
        server_port=8000,
        prefer_binary=True,
        wait_seconds=0,
        output_root="output"):
    latest_directory = latest_trajectory_output_dir_from_config(
        config,
        sim_index=sim_index,
        trajectory_output_dir=trajectory_output_dir,
        prefer_binary=prefer_binary,
        wait_seconds=wait_seconds,
        output_root=output_root,
    )
    print(
        f"Starting visualization server for {_trajectory_format_name(latest_directory)} "
        f"trajectory output: {latest_directory}"
    )
    stop_event, server_thread = run_visualization_server(latest_directory, server_port)

    def stop():
        stop_visualization_server(stop_event, server_thread, server_port)

    return SimpleNamespace(
        directory=latest_directory,
        port=server_port,
        stop_event=stop_event,
        server_thread=server_thread,
        stop=stop,
    )


# ---------------------------------------------------------------------------
# Trajectory output helpers
# ---------------------------------------------------------------------------

def _read_property_values(properties_path):
    values = {}
    try:
        with open(properties_path, "r") as properties_file:
            for raw_line in properties_file:
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
    except OSError:
        pass
    return values


def _read_trajectory_manifest(directory):
    manifest_path = os.path.join(directory, "manifest.json")
    try:
        with open(manifest_path, "r") as manifest_file:
            return json.load(manifest_file)
    except (OSError, json.JSONDecodeError):
        return None


def _resolve_trajectory_root(sim_folder, configured_path):
    if not configured_path:
        return None
    configured_path = os.path.normpath(configured_path)
    if os.path.isabs(configured_path):
        return configured_path
    return os.path.join(sim_folder, configured_path)


def _configured_trajectory_roots(sim_folder):
    properties = _read_property_values(os.path.join(sim_folder, "data", "Data.properties"))
    roots = []
    for key in ("TRAJECTORY_BINARY_DEFAULT_PATH", "JSON_DEFAULT_PATH"):
        root = _resolve_trajectory_root(sim_folder, properties.get(key))
        if root and root not in roots:
            roots.append(root)

    default_root = os.path.join(sim_folder, "trajectory_output")
    if default_root not in roots:
        roots.append(default_root)
    return roots


def _trajectory_format_score(directory):
    try:
        names = os.listdir(directory)
    except OSError:
        return 0

    if "manifest.json" in names:
        return 3
    lowered = [name.lower() for name in names]
    if any(name.endswith(".bin") for name in lowered):
        return 2
    if any(name.endswith(".json") for name in lowered):
        return 1
    return 0


def _trajectory_format_name(directory):
    manifest = _read_trajectory_manifest(directory)
    if manifest is not None:
        output_format = manifest.get("format", "binary")
        version = manifest.get("version")
        sparse_frame_groups = manifest.get("sparseFrameGroups") or []
        sparse_suffix = " sparse" if sparse_frame_groups else ""
        if version is not None:
            return f"{output_format} v{version}{sparse_suffix}"
        return f"{output_format}{sparse_suffix}"

    score = _trajectory_format_score(directory)
    if score >= 2:
        return "binary"
    if score == 1:
        return "JSON"
    return "trajectory"


def _trajectory_manifest_summary(directory, manifest):
    chunks = manifest.get("chunks", [])
    active_chunk = manifest.get("activeChunk", {})
    road_dictionary = manifest.get("roadIdDictionary", [])
    zone_dictionary = manifest.get("zoneDictionary", [])
    charging_station_dictionary = manifest.get("chargingStationDictionary", [])
    schemas = manifest.get("schemas", {})
    frame_groups = manifest.get("frameGroups", [])
    sparse_frame_groups = manifest.get("sparseFrameGroups") or []
    sparse_frame_group_mode = manifest.get("sparseFrameGroupMode")

    return {
        "directory": directory,
        "manifest_path": os.path.join(directory, "manifest.json"),
        "format": manifest.get("format"),
        "version": manifest.get("version"),
        "byte_order": manifest.get("byteOrder"),
        "coord_scale": manifest.get("coordScale"),
        "initial_x": manifest.get("initialX"),
        "initial_y": manifest.get("initialY"),
        "tick_interval": manifest.get("tickInterval"),
        "link_snapshot_interval": manifest.get("linkSnapshotInterval"),
        "chunk_tick_limit": manifest.get("chunkTickLimit"),
        "chunk_count": len(chunks),
        "active_chunk": active_chunk,
        "road_count": len(road_dictionary),
        "zone_count": len(zone_dictionary),
        "charging_station_count": len(charging_station_dictionary),
        "frame_groups": frame_groups,
        "sparse_frame_groups": sparse_frame_groups,
        "sparse_frame_group_mode": sparse_frame_group_mode,
        "has_sparse_frame_groups": bool(sparse_frame_groups),
        "has_sparse_zone_frames": "zone" in sparse_frame_groups,
        "has_sparse_charging_station_frames": "chargingStation" in sparse_frame_groups,
        "schema_names": sorted(schemas.keys()),
        "has_zone_attributes": bool(zone_dictionary) or "zone" in schemas,
        "has_charging_station_attributes": (
            bool(charging_station_dictionary) or "chargingStation" in schemas
        ),
        "has_split_energy_fields": (
            "frameHeader" in schemas
            and any("energyPrivateEV" in field for field in schemas["frameHeader"])
        ),
    }


def _latest_trajectory_directory(root, prefer_binary=True):
    if root is None or not os.path.isdir(root):
        return None

    candidates = []
    root_score = _trajectory_format_score(root)
    if root_score > 0:
        candidates.append((root_score, os.path.getmtime(root), root))

    for name in os.listdir(root):
        candidate = os.path.join(root, name)
        if not os.path.isdir(candidate):
            continue
        score = _trajectory_format_score(candidate)
        if score > 0:
            candidates.append((score, os.path.getmtime(candidate), candidate))

    if not candidates:
        subdirs = [
            os.path.join(root, name)
            for name in os.listdir(root)
            if os.path.isdir(os.path.join(root, name))
        ]
        if not subdirs:
            return None
        return max(subdirs, key=os.path.getmtime)

    if prefer_binary:
        binary_candidates = [candidate for candidate in candidates if candidate[0] >= 2]
        if binary_candidates:
            return max(binary_candidates, key=lambda item: item[1])[2]

    return max(candidates, key=lambda item: item[1])[2]

# ---------------------------------------------------------------------------
# Simulation output path helpers
# ---------------------------------------------------------------------------

def get_sim_dir(options, i):
    sim_dir = "output/"+ options.name + "_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_seed_" + str(options.random_seeds[i])
    return sim_dir

# 
