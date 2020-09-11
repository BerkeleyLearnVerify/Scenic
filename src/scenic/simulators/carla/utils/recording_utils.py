#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

import cv2
import json
import numpy as np

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    pass

VIEW_WIDTH = 1280.0
VIEW_HEIGHT = 720.0
VIEW_FOV = 90.0

BB_COLOR = (248, 64, 24)

class BBoxUtil(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_bounding_boxes(vehicles, camera):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """

        bounding_boxes = [BBoxUtil.get_bounding_box(vehicle, camera) for vehicle in vehicles]
        # filter objects behind camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        return bounding_boxes

    @staticmethod
    def get_3d_bounding_boxes(vehicles, camera):
        bounding_boxes = []

        for vehicle in vehicles:
            bb_cords = BBoxUtil._create_bb_points(vehicle)
            cords_x_y_z = BBoxUtil._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
            cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
            bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
            bounding_boxes.append(bbox)

        return bounding_boxes

    @staticmethod
    def get_2d_bounding_boxes(bounding_boxes_3d):
        min_x_coords = [np.min(bbox[:, 0]) for bbox in bounding_boxes_3d]
        max_x_coords = [np.max(bbox[:, 0]) for bbox in bounding_boxes_3d]
        min_y_coords = [np.min(bbox[:, 1]) for bbox in bounding_boxes_3d]
        max_y_coords = [np.max(bbox[:, 1]) for bbox in bounding_boxes_3d]

        bounding_boxes_2d = []

        for min_x, max_x, min_y, max_y in zip(min_x_coords, max_x_coords, min_y_coords, max_y_coords):
            # Construct 4 corners
            bbox_2d = []
            bbox_2d.append([min_x, min_y])
            bbox_2d.append([max_x, min_y])
            bbox_2d.append([max_x, max_y])
            bbox_2d.append([min_x, max_y])

            bounding_boxes_2d.append(bbox_2d)

        return bounding_boxes_2d

    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        for bbox in bounding_boxes:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
            # draw lines
            # base
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
            # top
            pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
            pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
            # base-top
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
        display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = BBoxUtil._create_bb_points(vehicle)
        cords_x_y_z = BBoxUtil._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = BBoxUtil._vehicle_to_world(cords, vehicle)
        sensor_cord = BBoxUtil._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = BBoxUtil.get_matrix(bb_transform)
        vehicle_world_matrix = BBoxUtil.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = BBoxUtil.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def sensor_transform_to_world(cords, sensor_transform):
        """
        Transforms sensor coordinates to world.
        """

        sensor_world_matrix = BBoxUtil.get_matrix(sensor_transform)
        world_cords = np.dot(sensor_world_matrix, cords)
        return world_cords

    @staticmethod
    def world_to_sensor_transform(cords, sensor_transform):
        sensor_world_matrix = BBoxUtil.get_matrix(sensor_transform)
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_sensor_calibrated(cords, sensor_calibration):
        cords_x_y_z = cords[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        calib_cords = np.transpose(np.dot(sensor_calibration, cords_y_minus_z_x))
        calib_cords = np.concatenate([calib_cords[:, 0] / calib_cords[:, 2], calib_cords[:, 1] / calib_cords[:, 2], calib_cords[:, 2]], axis=1)

        return calib_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix

# Stores bounding boxes for a single simulation run
class BBoxRecording:
    def __init__(self, frames=None):
        self.frames = []
        if frames is not None:
            self.frames = frames

    def add_frame(self, bounding_boxes):
        self.frames.append(bounding_boxes)

    def get_frame(self, frame_idx):
        return self.frames[frame_idx]

    @staticmethod
    def import_from_file(filepath):
        with open(filepath, 'r') as f:
            json_data = json.load(f)

        return BBoxRecording(json_data)

    def save(self, filepath):
        with open(filepath, 'w') as f:
            json.dump(self.frames, f)

class VideoRecording:
    def __init__(self, frames=None):
        # Each frame is a numpy array
        self.frames = []
        if frames is not None:
            self.frames = frames

    def add_frame(self, frame_data):
        frame_array = np.frombuffer(frame_data.raw_data, dtype=np.dtype("uint8"))
        frame_array = np.reshape(frame_array, (frame_data.height, frame_data.width, 4))
        frame_array = frame_array[:, :, :3]
        self.frames.append(frame_array)

    def get_frame(self, frame_idx):
        return self.frames[frame_idx]

    @staticmethod
    def import_from_file(filepath):
        stream = cv2.VideoCapture(filepath)
        num_frames = int(stream.get(cv2.CAP_PROP_FRAME_COUNT))

        frames = []

        for frame_idx in range(num_frames):
            stream.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            _, frame = stream.read()
            frames.append(np.array(frame[:, :, :3], dtype=np.dtype("uint8")))

        return VideoRecording(frames)

    def save(self, filepath):
        if len(self.frames) == 0:
            print('Tried to save video, but no frames have been recorded')
            return

        frame_height, frame_width, _ = self.frames[0].shape

        out = cv2.VideoWriter(
            filepath,
            cv2.VideoWriter_fourcc(*'mp4v'),
            30.0,
            (frame_width, frame_height),
            True
        )

        for frame in self.frames:
            out.write(frame)

        out.release()

class LidarRecording:
    def __init__(self, frames=None):
        # Each frame is a list of classified lidar points
        self.frames = []
        if frames is not None:
            self.frames = frames

    def add_frame(self, lidar_points):
        self.frames.append(lidar_points)

    def get_frame(self, frame_idx):
        return self.frames[frame_idx]

    @staticmethod
    def import_from_file(filepath):
        with open(filepath, 'r') as f:
            json_data = json.load(f)

        return LidarRecording(json_data)

    def save(self, filepath):
        with open(filepath, 'w') as f:
            json.dump(self.frames, f)