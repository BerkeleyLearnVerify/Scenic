import os
import cv2
import json
import numpy as np

import scenic.simulators.carla.utils.recording_utils as rec_utils

from PIL import Image, ImageDraw

class SensorConfig:
	def __init__(self, sensor_config_file):
		with open(sensor_config_file, 'r') as f:
			sensor_data = json.load(f)

		self.sensors = {s['name']: s for s in sensor_data}

	def get(self, sensor_name):
		return self.sensors[sensor_name]

	def get_sensors(self):
		return self.sensors.values()

class SimulationData:
	def __init__(self, simulation_dir, sensor_config):
		# Parameter for lazy fetching of data
		self.have_fetched_data = False

		self.bboxes_fpath = os.path.join(simulation_dir, 'annotations', 'bboxes.json')
		self.bbox_recording = None

		self.sensor_data = {}

		for sensor in sensor_config.get_sensors():
			sensor_dir = os.path.join(simulation_dir, sensor['name'])

			if sensor['type'] == 'rgb':
				# Lazily retrieve simulation data from disk later, just store paths for now
				sensor_data_paths = {
					'rgb': os.path.join(sensor_dir, 'rgb.mp4'),
					'depth': os.path.join(sensor_dir, 'depth.mp4'),
					'semantic': os.path.join(sensor_dir, 'semantic.mp4'),
				}

				data = {
					'type': sensor['type'],
					'data_paths': sensor_data_paths,
				}

				self.sensor_data[sensor['name']] = data
			elif sensor['type'] == 'lidar':
				sensor_data_paths = {
					'lidar': os.path.join(sensor_dir, 'lidar.json')
				}

				data = {
					'type': sensor['type'],
					'data_paths': sensor_data_paths,
				}

				self.sensor_data[sensor['name']] = data

	def fetch_data_from_disk(self):
		self.bbox_recording = rec_utils.BBoxRecording.import_from_file(self.bboxes_fpath)

		for sensor in self.sensor_data.values():
			if sensor['type'] == 'rgb':
				recordings = {
					'rgb': rec_utils.VideoRecording.import_from_file(sensor['data_paths']['rgb']),
					'depth': rec_utils.VideoRecording.import_from_file(sensor['data_paths']['depth']),
					'semantic': rec_utils.VideoRecording.import_from_file(sensor['data_paths']['semantic']),
				}

				sensor['recordings'] = recordings
			elif sensor['type'] == 'lidar':
				recordings = {
					'lidar': rec_utils.FrameRecording.import_from_file(sensor['data_paths']['lidar']),
				}

				sensor['recordings'] = recordings

		self.have_fetched_data = True

	def __len__(self):
		if not self.have_fetched_data:
			self.fetch_data_from_disk()

		return len(self.bbox_recording.frames)

	def __getitem__(self, frame_idx):
		if not self.have_fetched_data:
			self.fetch_data_from_disk()

		retval = {}

		retval['bboxes'] = self.bbox_recording.get_frame(frame_idx)

		for sensor_name, sensor in self.sensor_data.items():
			if sensor['type'] == 'rgb':
				sensor_frame = {
					'rgb': sensor['recordings']['rgb'].get_frame(frame_idx),
					'depth': sensor['recordings']['depth'].get_frame(frame_idx),
					'semantic': sensor['recordings']['semantic'].get_frame(frame_idx),
				}

				retval[sensor_name] = sensor_frame
			elif sensor['type'] == 'lidar':
				sensor_frame = {
					'lidar': sensor['recordings']['lidar'].get_frame(frame_idx)
				}

				retval[sensor_name] = sensor_frame

		return retval

class DataAPI:
	def __init__(self, data_dir, sensor_config):
		self.data_dir = data_dir
		self.sensor_config = sensor_config

		# Index all simulations in data directory
		self.simulation_data = {}

		scenario_dir_names = os.listdir(self.data_dir)

		for scen_dir_name in scenario_dir_names:
			scenario_dir = os.path.join(self.data_dir, scen_dir_name)
			simulation_dir_names = os.listdir(scenario_dir)

			for sim_dir_name in simulation_dir_names:
				simulation_dir = os.path.join(scenario_dir, sim_dir_name)

				sim_data = SimulationData(simulation_dir, self.sensor_config)

				simulation_name = os.path.join(scen_dir_name, sim_dir_name)
				self.simulation_data[simulation_name] = sim_data

	def get_simulations(self):
		return self.simulation_data

def get_bbox_3d_projected(bboxes, sensor):
	retval = []

	# Set up calibration matrix to be used for bounding box projection
	calibration = np.identity(3)
	VIEW_WIDTH = sensor['settings']['VIEW_WIDTH']
	VIEW_HEIGHT = sensor['settings']['VIEW_HEIGHT']
	VIEW_FOV = sensor['settings']['VIEW_FOV']
	calibration[0, 2] = VIEW_WIDTH / 2.0
	calibration[1, 2] = VIEW_HEIGHT / 2.0
	calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))

	for obj_type in bboxes.keys():
		for bb in bboxes[obj_type]:
			bbox_cords = np.array(bb)

			cords_x_y_z = np.transpose(bbox_cords)
			cords_x_y_z_1 = np.vstack((cords_x_y_z, np.ones(cords_x_y_z.shape[1])))

			# Transform bounding box coords from ego to sensor
			sensor_to_ego = np.eye(4)
			sensor_to_ego[0, 3] = sensor['transform']['location'][0]
			sensor_to_ego[1, 3] = sensor['transform']['location'][1]
			sensor_to_ego[2, 3] = sensor['transform']['location'][2]
			ego_to_sensor = np.linalg.inv(sensor_to_ego)

			cords_x_y_z = np.dot(ego_to_sensor, cords_x_y_z_1)[:3]

			# Project 3D bounding box coordinates onto image plane
			cords_y_minus_z_x = np.vstack([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
			camera_bbox = np.transpose(np.dot(calibration, cords_y_minus_z_x))
			camera_bbox = np.vstack([camera_bbox[:, 0] / camera_bbox[:, 2], camera_bbox[:, 1] / camera_bbox[:, 2], camera_bbox[:, 2]])
			camera_bbox = np.transpose(camera_bbox)

			# Only draw bbox if it's in front of sensor
			if all(camera_bbox[:, 2] > 0):
				retval.append(camera_bbox.tolist())

	return retval

def draw_bbox_3d(bboxes, sensor, img, output_filepath):
	bboxes_projected = get_bbox_3d_projected(bboxes, sensor)

	frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	im_pil = Image.fromarray(frame)

	draw = ImageDraw.Draw(im_pil)

	# Point radius
	r = 6

	FILL = 'red'
	LINE_WIDTH = 4

	for bbox in bboxes_projected:
		for point in bbox:
			draw.ellipse([point[0] - r, point[1] - r, point[0] + r, point[1] + r], fill=FILL)

		bottom_rect = [(point[0], point[1]) for point in bbox[:4]]
		bottom_rect.append((bbox[0][0], bbox[0][1]))
		draw.line(bottom_rect, fill=FILL, width=LINE_WIDTH)

		top_rect = [(point[0], point[1]) for point in bbox[4:]]
		top_rect.append((bbox[4][0], bbox[4][1]))
		draw.line(top_rect, fill=FILL, width=LINE_WIDTH)

		for point_idx in range(4):
			bottom_pt = bbox[point_idx]
			top_pt = bbox[point_idx + 4]
			draw.line([bottom_pt[0], bottom_pt[1], top_pt[0], top_pt[1]], fill=FILL, width=LINE_WIDTH)

	im_pil.save(output_filepath)

def get_bbox_2d(bboxes, sensor):
	bboxes_projected = get_bbox_3d_projected(bboxes, sensor)

	retval = []

	for bbox in bboxes_projected:
		x_cords = [cord[0] for cord in bbox]
		y_cords = [cord[1] for cord in bbox]

		min_x = min(x_cords)
		min_y = min(y_cords)

		max_x = max(x_cords)
		max_y = max(y_cords)

		retval.append([[min_x, min_y], [min_x, max_y], [max_x, max_y], [max_x, min_y]])

	return retval

def draw_bbox_2d(bboxes, sensor, img, output_filepath):
	frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	im_pil = Image.fromarray(frame)
	draw = ImageDraw.Draw(im_pil)

	bboxes_2d = get_bbox_2d(bboxes, sensor)

	# Point radius
	r = 6

	FILL = 'red'
	LINE_WIDTH = 4

	for bbox in bboxes_2d:
		for point in bbox:
			draw.ellipse([point[0] - r, point[1] - r, point[0] + r, point[1] + r], fill=FILL)

		rect = [(point[0], point[1]) for point in bbox]
		rect.append((bbox[0][0], bbox[0][1]))
		draw.line(rect, fill=FILL, width=LINE_WIDTH)

	im_pil.save(output_filepath)

def save_frame(img, output_filepath):
	frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	im_pil = Image.fromarray(frame)
	im_pil.save(output_filepath)

def save_point_cloud(lidar_points, output_filepath):
	SEMANTIC_CLASS_TO_COLOR = {
		0: [0, 0, 0, 255],
		1: [70, 70, 70, 255],
		2: [190, 153, 153, 255],
		3: [250, 170, 160, 255],
		4: [220, 20, 60, 255],
		5: [153, 153, 153, 255],
		6: [157, 234, 50, 255],
		7: [128, 64, 128, 255],
		8: [244, 35, 232, 255],
		9: [107, 142, 35, 255],
		10: [0, 0, 142, 255],
		11: [102, 102, 156, 255],
		12: [220, 220, 0, 255]
	}

	with open(output_filepath, 'w') as f:		
		for p in lidar_points:
			if p[3] not in SEMANTIC_CLASS_TO_COLOR:
				continue
			semantic_color = SEMANTIC_CLASS_TO_COLOR[p[3]]
			
			# Convert to right-handed coordinate system
			f.write('{} {} {} {} {} {}\n'.format(-p[0], p[1], p[2], semantic_color[0], semantic_color[1], semantic_color[2]))