#
# Copyright (c) 2019-2020 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from websocket import create_connection
from enum import Enum
import json
import lgsvl
import math
import logging
import sys
import os

log = logging.getLogger(__name__)


class CoordType(Enum):
    Unity = 1
    Northing = 2
    Latitude = 3


class Connection:
    def __init__(self, simulator, ego_agent, ip=os.environ.get("LGSVL__AUTOPILOT_0_HOST", "localhost"), port="8888"):
        """
        simulator: is an lgsvl.Simulator object
        ego_agent: an lgsvl.EgoVehicle object, this is intended to be used with a vehicle equipped with Apollo 5.0
        ip: address of the machine where the Apollo stack is running
        port: the port number for Dreamview
        """
        self.url = "ws://" + ip + ":" + port + "/websocket"
        self.sim = simulator
        self.ego = ego_agent
        self.ws = create_connection(self.url)
        self.gps_offset = lgsvl.Vector()

    def set_destination(self, x_long_east, z_lat_north, y=0, coord_type=CoordType.Unity):
        """
        This function can accept a variety of Coordinate systems

        If using Unity World Coordinate System:
        x_long_east = x
        z_lat_north = z
        y = y

        If using Latitude/Longitude:
        x_long_east = Longitude
        z_lat_north = Latitude

        If using Easting/Northing:
        x_long_east = Easting
        z_lat_north = Northing
        """
        current_pos = self.ego.state.transform
        current_gps = self.sim.map_to_gps(current_pos)
        heading = math.radians(current_gps.orientation)

        # Start position should be the position of the GPS
        # Unity returns the position of the center of the vehicle so adjustment is required
        northing_adjustment = (
            math.sin(heading) * self.gps_offset.z - math.cos(heading) * self.gps_offset.x
        )
        easting_adjustment = (
            math.cos(heading) * self.gps_offset.z + math.sin(heading) * self.gps_offset.x
        )

        if coord_type == CoordType.Unity:
            transform = lgsvl.Transform(
                lgsvl.Vector(x_long_east, y, z_lat_north), lgsvl.Vector(0, 0, 0)
            )
            gps = self.sim.map_to_gps(transform)
            dest_x = gps.easting
            dest_y = gps.northing

        elif coord_type == CoordType.Northing:
            dest_x = x_long_east
            dest_y = z_lat_north

        elif coord_type == CoordType.Latitude:
            transform = self.sim.map_from_gps(longitude=x_long_east, latitude=z_lat_north)
            gps = self.sim.map_to_gps(transform)
            dest_x = gps.easting
            dest_y = gps.northing

        else:
            log.error(
                "Please input a valid Coordinate Type: Unity position, Easting/Northing, or Longitude/Latitude"
            )
            return

        self.ws.send(
            json.dumps(
                {
                    "type": "SendRoutingRequest",
                    "start": {
                        "x": current_gps.easting + easting_adjustment,
                        "y": current_gps.northing + northing_adjustment,
                        "z": 0,
                        "heading": heading,
                    },
                    "end": {"x": dest_x, "y": dest_y, "z": 0},
                    "waypoint": "[]",
                }
            )
        )

        return

    def enable_module(self, module):
        """
        module is the name of the Apollo 5.0 module as seen in the "Module Controller" tab of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "START_MODULE", "value": module})
        )
        return

    def disable_module(self, module):
        """
        module is the name of the Apollo 5.0 module as seen in the "Module Controller" tab of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "STOP_MODULE", "value": module})
        )
        return

    def set_hd_map(self, hd_map):
        """
        Folders in /apollo/modules/map/data/ are the available HD maps
        Map options in Dreamview are the folder names with the following changes:
            - underscores (_) are replaced with spaces
            - the first letter of each word is capitalized

        hd_map parameter is the modified folder name.
        hd_map should match one of the options in the right-most drop down in the top-right corner of Dreamview.
        """

        word_list = []
        for s in hd_map.split('_'):
            word_list.append(s[0].upper() + s[1:])

        mapped_map = ' '.join(word_list)

        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MAP", "value": mapped_map})
        )

        if not self.get_current_map() == mapped_map:
            folder_name = hd_map.replace(" ", "_")
            error_message = (
                "HD Map {0} was not set. Verify the files exist in "
                "/apollo/modules/map/data/{1} and restart Dreamview -- Aborting..."
            )
            log.error(
                error_message.format(
                    mapped_map, folder_name
                )
            )
            sys.exit(1)
        return

    def set_vehicle(self, vehicle, gps_offset_x=0.0, gps_offset_y=0.0, gps_offset_z=-1.348):
        # Lincoln2017MKZ from LGSVL has a GPS offset of 1.348m behind the center of the vehicle, lgsvl.Vector(0.0, 0.0, -1.348)
        """
        Folders in /apollo/modules/calibration/data/ are the available vehicle calibrations
        Vehicle options in Dreamview are the folder names with the following changes:
            - underscores (_) are replaced with spaces
            - the first letter of each word is capitalized

        vehicle parameter is the modified folder name.
        vehicle should match one of the options in the middle drop down in the top-right corner of Dreamview.
        """

        word_list = []
        for s in vehicle.split('_'):
            word_list.append(s[0].upper() + s[1:])

        mapped_vehicle = ' '.join(word_list)

        self.ws.send(
            json.dumps(
                {"type": "HMIAction", "action": "CHANGE_VEHICLE", "value": mapped_vehicle}
            )
        )

        self.gps_offset = lgsvl.Vector(gps_offset_x, gps_offset_y, gps_offset_z)

        if not self.get_current_vehicle() == mapped_vehicle:
            folder_name = vehicle.replace(" ", "_")
            error_message = (
                "Vehicle calibration {0} was not set. Verify the files exist in "
                "/apollo/modules/calibration/data/{1} and restart Dreamview -- Aborting..."
            )
            log.error(
                error_message.format(
                    mapped_vehicle, folder_name
                )
            )
            sys.exit(1)
        return

    def set_setup_mode(self, mode):
        """
        mode is the name of the Apollo 5.0 mode as seen in the left-most drop down in the top-right corner of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MODE", "value": mode})
        )
        return

    def get_module_status(self):
        """
        Returns a dict where the key is the name of the module and value is a bool based on the module's current status
        """
        self.reconnect()
        data = json.loads(
            self.ws.recv()
        )  # This first recv() call returns the SimControlStatus in the form '{"enabled":false,"type":"SimControlStatus"}'
        while data["type"] != "HMIStatus":
            data = json.loads(self.ws.recv())

        # The second recv() call also contains other information:
        #   the current map, vehicle, and mode:
        #       data["data"]["currentMap"], data["data"]["currentVehicle"], data["data"]["currentMode"]
        #
        #   the available maps, vehicles, and modes:
        #       data["data"]["maps"], data["data"]["vehicles"], data["data"]["modes"]
        #
        #   the status of monitors components:
        #       data["data"]["monitoredComponents"]

        return data["data"]["modules"]

    def get_current_map(self):
        """
        Returns the current HD Map loaded in Dreamview
        """
        self.reconnect()
        data = json.loads(self.ws.recv())
        while data["type"] != "HMIStatus":
            data = json.loads(self.ws.recv())
        return data["data"]["currentMap"]

    def get_current_vehicle(self):
        """
        Returns the current Vehicle configuration loaded in Dreamview
        """
        self.reconnect()
        data = json.loads(self.ws.recv())
        while data["type"] != "HMIStatus":
            data = json.loads(self.ws.recv())
        return data["data"]["currentVehicle"]

    def reconnect(self):
        """
        Closes the websocket connection and re-creates it so that data can be received again
        """
        self.ws.close()
        self.ws = create_connection(self.url)
        return

    def enable_apollo(self, dest_x, dest_z, modules):
        """
        Enables a list of modules and then sets the destination
        """
        for mod in modules:
            log.info("Starting {} module...".format(mod))
            self.enable_module(mod)

        self.set_destination(dest_x, dest_z)

    def disable_apollo(self):
        """
        Disables all Apollo modules
        """
        module_status = self.get_module_status()
        for module in module_status.keys():
            self.disable_module(module)

    def check_module_status(self, modules):
        """
        Checks if all modules in a provided list are enabled
        """
        module_status = self.get_module_status()
        for module, status in module_status.items():
            if not status and module in modules:
                log.warning(
                    "Warning: Apollo module {} is not running!!!".format(module)
                )

    def setup_apollo(self, dest_x, dest_z, modules, default_timeout=60.0):
        """
        Starts a list of Apollo modules and sets the destination. Will wait for Control module to send a message before returning.
        Control sending a message indicates that all modules are working and Apollo is ready to continue.
        """
        initial_state = self.ego.state
        mod_status = self.get_module_status()

        # If any module is not enabled, Control may still send a message even though Apollo is not ready
        if not all(mod_status[mod] for mod in modules):
            self.disable_apollo()

        self.enable_apollo(dest_x, dest_z, modules)
        self.ego.is_control_received = False

        def on_control_received(agent, kind, context):
            if kind == "checkControl":
                agent.is_control_received = True
                log.info("Control message received")

        self.ego.on_custom(on_control_received)

        try:
            timeout = float(os.environ.get("LGSVL__DREAMVIEW__CONTROL_MESSAGE_TIMEOUT_SECS", default_timeout))
        except Exception:
            timeout = default_timeout
            log.warning("Invalid LGSVL__DREAMVIEW__CONTROL_MESSAGE_TIMEOUT_SECS, using default {0}s".format(default_timeout))

        run_time = 2
        elapsed = 0
        while timeout <= 0.0 or float(elapsed) < timeout:
            self.sim.run(run_time)

            if self.ego.is_control_received:
                break

            if elapsed > 0 and elapsed % (run_time * 5) == 0:
                self.check_module_status(modules)
                log.info("{} seconds have passed but Ego hasn't received any control messages.".format(elapsed))
                log.info("Please also check if your route has been set correctly in Dreamview.")

            elapsed += run_time
        else:
            log.error("No control message from Apollo within {} seconds. Aborting...".format(timeout))
            self.disable_apollo()
            raise WaitApolloError()

        self.ego.state = initial_state


class WaitApolloError(Exception):
    """
    Raised when Apollo control message is not received in time
    """

    pass
