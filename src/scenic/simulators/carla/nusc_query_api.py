from nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.scripts.export_2d_annotations_as_json import post_process_coords
from nuscenes.utils.geometry_utils import view_points
from nuscenes.map_expansion import arcline_path_utils

from pyquaternion.quaternion import Quaternion
import numpy as np

import shapely
from shapely.geometry import Point, Polygon, LineString, MultiLineString
from shapely.ops import unary_union

from datetime import datetime, timezone, timedelta

from collections import defaultdict

class NuscQueryAPI:
	LOCATIONS = ['boston-seaport', 'singapore-onenorth', 'singapore-queenstown', 'singapore-hollandvillage']
	VALID_LOCATIONS = {'boston-seaport'}

	VEHICLE_CATEGORIES = {'vehicle.car', 'vehicle.truck', 'vehicle.emergency.police', 'vehicle.bicycle', 'vehicle.motorcycle'}
	PEDESTRIAN_CATEGORIES = {'human.pedestrian.adult', 'human.pedestrian.child', 'human.pedestrian.construction_worker', 'human.pedestrian.police_officer'}
	OBJECT_CATEGORIES = {'movable_object.trafficcone'}
	VALID_CATEGORIES = VEHICLE_CATEGORIES.union(PEDESTRIAN_CATEGORIES, OBJECT_CATEGORIES)

	INVALID_CATEGORIES = {'movable_object.pushable_pullable','movable_object.debris', \
	'movable_object.barrier','human.pedestrian.wheelchair','human.pedestrian.stroller', \
	'human.pedestrian.personal_mobility', 'animal', \
	'vehicle.bus.rigid', 'vehicle.bus.bendy','vehicle.trailer','vehicle.emergency.ambulance', \
	'vehicle.construction'}

	SINGAPORE_TIMEZONE = timezone(timedelta(hours=8))
	# -4 hours from UTC because data was captured at Eastern Savings Time
	BOSTON_TIMEZONE = timezone(timedelta(hours=-4))
	TIME_ZONE = {'singapore-onenorth': SINGAPORE_TIMEZONE,
				 'boston-seaport': BOSTON_TIMEZONE,
				 'singapore-queenstown': SINGAPORE_TIMEZONE,
				 'singapore-hollandvillage': SINGAPORE_TIMEZONE}

	def __init__(self, version='v1.0-trainval', dataroot='/home/bridge_simulator_to_realworld/nuscenes'):
		self.nusc = NuScenes(version=version, dataroot=dataroot, verbose=True)

		# Create reverse mapping for image filenames to sample data tokens
		self.img_filename_to_sample_data_token = {}

		# Create reverse mapping for image filenames to location
		self.img_filename_to_location = {}

		self.img_filenames = set()

		for scene in self.nusc.scene:
			num_samples = scene['nbr_samples']
			curr_sample_token = scene['first_sample_token']

			log = self.nusc.get('log', scene['log_token'])
			location = log['location']

			if location not in self.VALID_LOCATIONS:
				continue

			for _ in range(num_samples):
				sample = self.nusc.get('sample', curr_sample_token)
				cam_front_data = self.nusc.get('sample_data', sample['data']['CAM_FRONT'])

				# Extract file name without directories; e.g., samples/CAM_FRONT/filename.jpg becomes filename.jpg
				img_filename = cam_front_data['filename'].split('/')[-1]
				self.img_filenames.add(img_filename)
				self.img_filename_to_sample_data_token[img_filename] = cam_front_data['token']

				self.img_filename_to_location[img_filename] = location

				curr_sample_token = sample['next']

		# Load map for each location
		self.nusc_map = {}
		for loc in self.LOCATIONS:
			self.nusc_map[loc] = NuScenesMap(dataroot=dataroot, map_name=loc)

		# Retrieve road, curb, and sidewalk geometry for each location
		self.road = {}
		self.sidewalk = {}
		self.curb = {}

		# Construct Scenic network for each location
		self.scenic_network = {}

		for location in NuscQueryAPI.LOCATIONS:
			map_api = self.nusc_map[location]

			road_polys = [map_api.extract_polygon(rec['polygon_token']) for rec in map_api.road_segment]
			sidewalk_polys = [map_api.extract_polygon(rec['polygon_token']) for rec in map_api.walkway]

			self.road[location] = unary_union(road_polys)
			self.sidewalk[location] = unary_union(sidewalk_polys)

			curb_lines = []
			for road_poly in self.road[location]:
				# Retrieve the curb, which is the boundary of the road polygon
				oriented_poly = shapely.geometry.polygon.orient(road_poly) # Orient the vertices CCW
				curb_vertices = list(oriented_poly.exterior.coords)
				curb_lines.append(LineString(curb_vertices))

			self.curb[location] = MultiLineString(curb_lines)

	def get_location(self, img_filename):
		return self.img_filename_to_location[img_filename]

	def _ann_in_view(box, cs_rec, pose_rec, camera_intrinsic):
		bbox = box.copy()

		# Move bbox to the ego-pose frame.
		bbox.translate(-np.array(pose_rec['translation']))
		bbox.rotate(Quaternion(pose_rec['rotation']).inverse)

		# Move them to the calibrated sensor frame.
		bbox.translate(-np.array(cs_rec['translation']))
		bbox.rotate(Quaternion(cs_rec['rotation']).inverse)

		# Filter out the corners that are not in front of the calibrated sensor.
		corners_3d = bbox.corners()
		in_front = np.argwhere(corners_3d[2, :] > 0).flatten()
		corners_3d = corners_3d[:, in_front]

		# Project 3d box to 2d.
		corner_coords = view_points(corners_3d, camera_intrinsic, True).T[:, :2].tolist()

		# Keep only corners that fall within the image.
		final_coords = post_process_coords(corner_coords)

		# Skip if the convex hull of the re-projected corners does not intersect the image canvas.
		if final_coords is None:
			return False

		return True

	def get_img_data(self, img_filename):
		sample_data_token = self.img_filename_to_sample_data_token[img_filename]
		sample_data = self.nusc.get('sample_data', sample_data_token)
		sample = self.nusc.get('sample', sample_data['sample_token'])
		scene = self.nusc.get('scene', sample['scene_token'])

		# Load road geometry
		location = self.get_location(img_filename)
		road = self.get_whole_map(location)['road']

		# Get the calibrated sensor and ego pose record to get the transformation matrices.
		cs_rec = self.nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])
		pose_rec = self.nusc.get('ego_pose', sample_data['ego_pose_token'])
		camera_intrinsic = np.array(cs_rec['camera_intrinsic'])

		ego_heading = Quaternion(pose_rec['rotation']).yaw_pitch_roll[0] * 180 / np.pi
		ego_xy = tuple(pose_rec['translation'][:2])

		# Get annotation records
		ann_recs = [self.nusc.get('sample_annotation', ann_token) for ann_token in sample['anns']]

		vehicles = []
		pedestrians = []
		objs = []

		for ann_rec in ann_recs:
			# Discard if not desired vehicle type
			instance = self.nusc.get('instance', ann_rec['instance_token'])
			category = self.nusc.get('category', instance['category_token'])

			box = self.nusc.get_box(ann_rec['token'])

			if not NuscQueryAPI._ann_in_view(box, cs_rec, pose_rec, camera_intrinsic):
				continue

			# Get heading angle (yaw) from quaternion in degrees
			heading = box.orientation.yaw_pitch_roll[0]
			heading = ((heading * 180 / np.pi) + 360) % 360

			# Birds-eye coordinates
			xy = box.center[:2]

			# Reject images with INVALID traffic participant ON road
			if (category['name'] in self.INVALID_CATEGORIES) and (road.intersects(Point(xy[0], xy[1]))):
				return 0

			# Skip annotation if it's not a desired vehicle type
			if category['name'] not in self.VALID_CATEGORIES:
				continue

			# Birds-eye 2D bounding box
			corners = box.bottom_corners()
			corners = corners[:2, :]
			vertices = []
			for corner_idx in range(corners.shape[1]):
				vertices.append((corners[0, corner_idx], corners[1, corner_idx]))
			box_2d = Polygon(vertices)

			obj_dict = {'heading': heading, 'position': tuple(xy), 'box': box_2d}

			if category['name'] in self.VEHICLE_CATEGORIES:
				vehicles.append(obj_dict)

			if category['name'] in self.PEDESTRIAN_CATEGORIES:
				pedestrians.append(obj_dict)

			if category['name'] in self.OBJECT_CATEGORIES:
				objs.append(obj_dict)

		# Sort by distance to ego
		def ego_dist_key_func(v):
			return (v['position'][0] - ego_xy[0]) ** 2 + (v['position'][1] - ego_xy[1]) ** 2

		vehicles.sort(key=ego_dist_key_func)
		pedestrians.sort(key=ego_dist_key_func)
		objs.sort(key=ego_dist_key_func)

		ego_dict = {'heading': ego_heading, 'position': ego_xy}
		retval = {'EgoCar': ego_dict, 'Vehicles': vehicles, 'Pedestrians': pedestrians, 'Objects': objs}

		# Get rectangular patch within map containing ego and vehicles
		# Start by computing the min and max in both dimensions
		x_min = min([ego_xy[0]] + [v['position'][0] for v in vehicles])
		y_min = min([ego_xy[1]] + [v['position'][1] for v in vehicles])
		x_max = max([ego_xy[0]] + [v['position'][0] for v in vehicles])
		y_max = max([ego_xy[1]] + [v['position'][1] for v in vehicles])

		# Add buffer of 25 meters per side
		x_min -= 25
		y_min -= 25
		x_max += 25
		y_max += 25

		# Load API for relevant map
		location = self.img_filename_to_location[img_filename]
		map_api = self.nusc_map[location]

		# Retrieve all lane and sidewalk polygons in patch
		patch_coords = (x_min, y_min, x_max, y_max)
		map_record_tokens = map_api.get_records_in_patch(patch_coords, ['road_segment', 'walkway'])
		road_records = [map_api.get('road_segment', token) for token in map_record_tokens['road_segment']]
		sidewalk_records = [map_api.get('walkway', token) for token in map_record_tokens['walkway']]

		lane_polys = [map_api.extract_polygon(rec['polygon_token']) for rec in road_records]
		sidewalk_polys = [map_api.extract_polygon(rec['polygon_token']) for rec in sidewalk_records]

		retval['road'] = unary_union(lane_polys)
		retval['sidewalk'] = unary_union(sidewalk_polys)

		# Get local time, dividing timestamp by 10^6 since it's represented in microseconds
		local_time = datetime.fromtimestamp(sample_data['timestamp'] / 1e6, tz=NuscQueryAPI.TIME_ZONE[location])
		retval['time'] = '{:02d}:{:02d}'.format(local_time.hour, local_time.minute)
		retval['description'] = scene['description']

		# Also return helper function that retrieves traffic flow at any point
		def get_traffic_flow(point):
			closest_lane_token = map_api.get_closest_lane(point[0], point[1])
			arcline = map_api.get_arcline_path(closest_lane_token)

			# Set heading to dummy zero value (not needed for function below)
			ego_pose = [point[0], point[1], 0]

			traffic_pose, _ = arcline_path_utils.project_pose_to_lane(ego_pose, arcline)

			return traffic_pose[2]

		retval['traffic_flow'] = get_traffic_flow

		return retval

	def get_whole_map(self, location):
		retval = {}
		retval['road'] = self.road[location]
		retval['sidewalk'] = self.sidewalk[location]
		retval['curb'] = self.curb[location]

		return retval

	def get_img_filenames(self):
		'''
		Returns a Set of all the image filenames.
		'''

		return self.img_filenames