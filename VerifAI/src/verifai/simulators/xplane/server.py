import functools, argparse, select
import struct, array
import time
import sys
import math
import socket

import numpy as np
import pandas as pd
import yaml
from dotmap import DotMap

try:
    from xpc import XPlaneConnect
except ImportError as e:
    raise RuntimeError('the X-Plane interface requires XPlaneConnect') from e

import verifai, verifai.server, verifai.falsifier, verifai.samplers
import verifai.simulators.xplane.utils.controller as simple_controller
from verifai.simulators.xplane.utils.geometry import (euclidean_dist, quaternion_for,
    initial_bearing, cross_track_distance, compute_heading_error)

position_dref = ["sim/flightmodel/position/local_x",
                 "sim/flightmodel/position/local_y",
                 "sim/flightmodel/position/local_z"]

orientation_dref = ["sim/flightmodel/position/theta",
                    "sim/flightmodel/position/phi",
                    "sim/flightmodel/position/q"]

velocity_drefs = [      # velocity and acceleration drefs to null between runs
    "sim/flightmodel/position/local_vx",
    "sim/flightmodel/position/local_vy",
    "sim/flightmodel/position/local_vz",
    "sim/flightmodel/position/local_ax",
    "sim/flightmodel/position/local_ay",
    "sim/flightmodel/position/local_az",
    "sim/flightmodel/position/P",
    "sim/flightmodel/position/Q",
    "sim/flightmodel/position/R",
]

fuel_dref = "sim/flightmodel/weight/m_fuel"

class XPlaneServer(verifai.server.Server):
    def __init__(self, sampling_data, monitor, options):
        super().__init__(sampling_data, monitor, options)

        self.runway_heading = options.runway.heading
        self.rh_sin, self.rh_cos = math.sin(self.runway_heading), math.cos(self.runway_heading)
        self.start_lat, self.start_lon = options.runway.start_lat, options.runway.start_lon
        self.end_lat, self.end_lon = options.runway.end_lat, options.runway.end_lon
        self.desired_heading = initial_bearing(self.start_lat, self.start_lon,
                                               self.end_lat, self.end_lon)
        self.origin_x, self.origin_z = options.runway.origin_x, options.runway.origin_z

        path = pd.read_csv(options.runway.elevations)
        self.x, self.y, self.z = path['x'].values, path['y'].values, path['z'].values
        assert len(self.x) == len(self.y), "latitude and longitude values do not align"

        self.controller = options.get('controller')
        self.frametime = 1.0 / options.get('framerate', 10.0)
        self.timeout = options.get('timeout', 8000)
        self.predicates = options.get('predicates', {})
        self.verbosity = options.get('verbosity', 0)

        video = options.get('video')
        if video is not None:
            region = video['region']
            size = video.get('size')
            if size is not None:
                size = (size['width'], size['height'])
            import utils.images as im
            self.grab_image = lambda: im.grab_image(region, resizeTo=size)
        else:
            self.grab_image = None

        try:
            self.xpcserver = XPlaneConnect(timeout=self.timeout)
            input('Please start a new flight and then press ENTER.')
            self.initFuel = list(self.xpcserver.getDREF(fuel_dref))
            theta, phi, q = self.xpcserver.getDREFs(orientation_dref)
            self.initTheta, self.initPhi = math.radians(theta[0]), math.radians(phi[0])
            x, y, z = self.xpcserver.getDREFs(position_dref)
            self.initPosition = (x[0], y[0], z[0])
        except socket.timeout as e:
            raise RuntimeError('unable to connect to X-Plane') from e

    def evaluate_sample(self, sample):
        simulation_data = self.run_simulation(sample)
        value = 0 if self.monitor is None else self.monitor.evaluate(simulation_data)
        return value

    def run_simulation(self, sample):
        # Get runway endpoints
        start_lat, start_lon = self.start_lat, self.start_lon
        end_lat, end_lon = self.end_lat, self.end_lon

        # Extract test parameters from sample
        if isinstance(self.sampler, verifai.ScenicSampler):
            # special case for parameters whose names aren't valid identifiers
            # TODO generalize this mechanism to other sampler types?
            params = self.sampler.paramDictForSample(sample)
        else:
            params = sample.params._asdict()
        simulation_time = params.pop('simulation_length', 30)
        setup_time = params.pop('setup_time', 2)
        # anything left in params we assume to be a dref
        custom_drefs = params
        # get plane position and heading
        plane = sample.objects[0]
        heading = plane.heading
        plane_x_sc, plane_z_sc = plane.position

        # Compute initial plane XZ position and heading in X-Plane local coordinates
        start_heading = self.runway_heading - heading
        plane_x = self.origin_x + (self.rh_cos * plane_x_sc) + (self.rh_sin * plane_z_sc)
        plane_z = self.origin_z - (self.rh_cos * plane_z_sc) + (self.rh_sin * plane_x_sc)

        # Find correct Y position for plane
        x_y = (plane_x, plane_z)
        euclid_dists = np.array([euclidean_dist(x_y, (self.x[i], self.z[i]))
                                for i in range(len(self.x))])
        best_elev = np.argmin(euclid_dists)
        plane_y = self.y[best_elev]

        # Reset plane and apply brake
        self.xpcserver.sendDREFs(velocity_drefs, [0]*len(velocity_drefs))
        self.xpcserver.sendCTRL([0, 0, 0, 0])
        self.xpcserver.sendCOMM("sim/operation/fix_all_systems")
        self.xpcserver.sendDREF(fuel_dref, self.initFuel)
        self.xpcserver.sendDREF("sim/flightmodel/controls/parkbrake", 1)

        # Set new plane position
        self.xpcserver.sendDREFs(position_dref, [plane_x, plane_y, plane_z])
        # Set new plane orientation
        quaternion = quaternion_for(self.initTheta, self.initPhi, start_heading)
        self.xpcserver.sendDREF("sim/flightmodel/position/q", quaternion)
        # Set any other specified drefs
        self.xpcserver.sendDREFs(list(custom_drefs.keys()), list(custom_drefs.values()))

        # Wait for weather, etc. to stabilize, then release brake
        time.sleep(setup_time)
        self.xpcserver.sendCOMM("sim/operation/fix_all_systems")
        time.sleep(0.1)
        self.xpcserver.sendDREF("sim/flightmodel/controls/parkbrake", 0)

        # Execute a run
        if self.verbosity >= 1:
            print('Starting run...')
        start = time.time()
        current = start
        lats, lons, psis, ctes, hes, times, images = [], [], [], [], [], [], []
        while current - start < simulation_time:
            times.append(current - start)
            # Get current plane state
            # Use modified getPOSI to get lat/lon in double precision
            lat, lon, _, _, _, psi, _ = self.xpcserver.getPOSI()
            lats.append(lat); lons.append(lon); psis.append(psi)
            # Compute cross-track and heading errors
            cte = cross_track_distance(start_lat, start_lon, end_lat, end_lon, lat, lon)
            heading_err = compute_heading_error(self.desired_heading, psi)
            ctes.append(cte); hes.append(heading_err)
            # Run controller for one step, if desired
            if self.controller is not None:
                self.controller(self.xpcserver, lat, lon, psi, cte, heading_err)
            # Save screenshot for videos
            if self.grab_image is not None:
                images.append(self.grab_image())
            if self.verbosity >= 2:
                print(f'cte: {cte}; heading_err: {heading_err}')

            # Limit framerate (for fast controllers)
            wait_time = self.frametime - (time.time() - current)
            if wait_time > 0:
                time.sleep(wait_time)

            current = time.time()
        self.images = images

        # Do some simple checks to see if the plane has gotten stuck
        thresh = 0.000001
        end_point_check, mid_point_check = True, True
        if abs(lats[0] - lats[-1]) < thresh and abs(lons[0] - lons[-1]) < thresh:
            end_point_check = False
        num_lats = len(lats)
        if abs(lats[0] - lats[num_lats//2]) < thresh and abs(lons[0] - lons[num_lats//2]) < thresh:
            mid_point_check = False
        if not (mid_point_check or end_point_check):
            raise RuntimeError('Plane appears to have gotten stuck!')

        # Compute time series for each atomic predicate
        simulation_data = {}
        for name, predicate in self.predicates.items():
            series = [
                (time, predicate(lat, lon, psi, cte, he))
                for time, lat, lon, psi, cte, he in zip(times, lats, lons, psis, ctes, hes)
            ]
            simulation_data[name] = series
        return simulation_data


class XPlaneFalsifier(verifai.falsifier.mtl_falsifier):
    def __init__(self, monitor, sampler_type=None, sampler=None, sample_space=None,
                 falsifier_params={}, server_options={}):
        super().__init__(monitor, sampler_type=sampler_type, sampler=sampler,
                         sample_space=sample_space,
                         falsifier_params=falsifier_params, server_options=server_options)
        self.verbosity = falsifier_params.get('verbosity', 0)
        video = falsifier_params.get('video')
        if video is not None:
            import utils.images       # will throw exception if required libraries not available
            self.video_threshold = video['threshold']
            self.video_framerate = video['framerate']
        else:
            self.video_threshold = None

    def init_server(self, server_options):
        samplingConfig = DotMap(sampler=self.sampler,
                                sampler_type=self.sampler_type,
                                sample_space=self.sample_space)
        self.server = XPlaneServer(samplingConfig, self.monitor, server_options)

    def populate_error_table(self, sample, rho, error=True):
        super().populate_error_table(sample, rho, error)
        if self.video_threshold is not None and rho <= self.video_threshold:
            if error:
                index = len(self.error_table.table) - 1
                name = f'error-{index}'
            else:
                index = len(self.safe_table.table) - 1
                name = f'safe-{index}'
            import utils.images
            utils.images.write_video(self.server.images, filename=name+'.avi',
                                     fps=self.video_framerate)


def run_test(configuration, runway, verbosity=0):
    # Load Scenic scenario
    print('Loading scenario...')
    sampler = verifai.ScenicSampler.fromScenario(configuration['scenario'])

    # Define predicates and specifications
    def nearcenterline(lat, lon, psi, cte, he):
        """MTL predicate 'cte < 1.5', i.e. within 1.5 m of centerline."""
        return 1.5 - abs(cte)
    predicates = { 'nearcenterline': nearcenterline }
    specification = [configuration.get('specification', 'G nearcenterline')]

    # Set up controller
    framerate = configuration['framerate']
    controller = simple_controller.control if configuration['controller'] else None

    # Get options for video recording
    video = configuration.get('video')
    if video is None or not video['record']:
        video = None
    else:
        video['framerate'] = framerate

    # Create falsifier (and underlying server)
    serverOptions = DotMap(port=8080, controller=controller, framerate=framerate,
                           predicates=predicates, verbosity=verbosity,
                           runway=runway, video=video)
    falsifierOptions = DotMap(
        n_iters=configuration['runs'],
        verbosity=verbosity, video=video,
        save_error_table=True, save_safe_table=True,
        error_table_path=configuration['error_table'],
        safe_table_path=configuration['safe_table'],
    )
    falsifier = XPlaneFalsifier(specification, sampler=sampler,
                                falsifier_params=falsifierOptions,
                                server_options=serverOptions)

    # Perform falsification
    try:
        falsifier.run_falsifier()
    except socket.timeout as e:
        raise RuntimeError('lost connection to X-Plane') from e
    finally:
        # Print error and safe tables
        if verbosity >= 1:
            print('Error Table (also saved to {}):'.format(falsifier.error_table_path))
            print(falsifier.error_table.table)
            print('Safe Table (also saved to {}):'.format(falsifier.safe_table_path))
            print(falsifier.safe_table.table)
        # Write out final cross-entropy distributions
        es = sampler.scenario.externalSampler
        if es is not None:
            es = es.sampler.domainSampler
            if isinstance(es, verifai.samplers.cross_entropy.CrossEntropySampler):
                if verbosity >= 1:
                    print('Cross-entropy distributions:')
                with open(configuration['cross_entropy'], 'w') as outfile:
                    for s in es.split_sampler.samplers:
                        if verbosity >= 1:
                            print(s.dist)
                        outfile.write(str(s.dist)+'\n')


def load_yaml(filename):
    with open(filename, 'r') as stream:
        options = yaml.safe_load(stream)
    return options


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', help='experiment configuration file', default='config.yaml')
    parser.add_argument('-r', '--runway', help='runway configuration file', default='runway.yaml')
    parser.add_argument('-v', '--verbosity', type=int, default=0)
    args = parser.parse_args()

    # Parse runway configuration
    runway = load_yaml(args.runway)
    rads = runway['radians']
    runway_heading = runway['heading']
    if not rads:
        runway_heading = math.radians(runway_heading)
    runway_data = DotMap(
        heading=runway_heading, elevations=runway['elevations'],
        origin_x=runway['origin_X'], origin_z=runway['origin_Z'],
        start_lat=runway['start_lat'], start_lon=runway['start_lon'],
        end_lat=runway['end_lat'], end_lon=runway['end_lon']
    )

    # Parse experiment configuration
    configuration = load_yaml(args.config)

    run_test(configuration, runway_data, verbosity=args.verbosity)
