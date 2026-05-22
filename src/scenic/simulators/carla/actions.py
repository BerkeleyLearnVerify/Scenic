"""Actions for dynamic agents in CARLA scenarios."""

import math as _math

import carla as _carla
import numpy as np
from scipy import linalg

from scenic.domains.driving.actions import *
import scenic.simulators.carla.utils.utils as _utils

################################################
# Actions available to all carla.Actor objects #
################################################

SetLocationAction = SetPositionAction  # TODO refactor


class SetAngularVelocityAction(Action):
    def __init__(self, angularVel):
        self.angularVel = angularVel

    def applyTo(self, obj, sim):
        xAngularVel = self.angularVel * _math.cos(obj.heading)
        yAngularVel = self.angularVel * _math.sin(obj.heading)
        newAngularVel = _utils.scalarToCarlaVector3D(xAngularVel, yAngularVel)
        obj.carlaActor.set_angular_velocity(newAngularVel)


class SetTransformAction(Action):
    def __init__(self, pos, heading):
        self.pos = pos
        self.heading = heading

    def applyTo(self, obj, sim):
        transform = _utils.scenicToCarlaTransform(
            obj,
            world=sim.world,
            snapToGround=obj.snapToGround,
            pos=self.pos,
        )
        transform.rotation.yaw = -_math.degrees(self.heading) - 90
        obj.carlaActor.set_transform(transform)


#############################################
# Actions specific to carla.Vehicle objects #
#############################################


class _CarlaVehicle:
    # Mixin identifying CARLA vehicles.
    # Used to avoid importing the Vehicle class from the CARLA model, which is
    # a Scenic module not importable from Python.
    pass


class VehicleAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _CarlaVehicle)


class SetManualGearShiftAction(VehicleAction):
    def __init__(self, manualGearShift):
        if not isinstance(manualGearShift, bool):
            raise TypeError("Manual gear shift must be a boolean.")
        self.manualGearShift = manualGearShift

    def applyTo(self, obj, sim):
        vehicle = obj.carlaActor
        ctrl = vehicle.get_control()
        ctrl.manual_gear_shift = self.manualGearShift
        vehicle.apply_control(ctrl)


class SetGearAction(VehicleAction):
    def __init__(self, gear):
        if not isinstance(gear, int):
            raise TypeError("Gear must be an int.")
        self.gear = gear

    def applyTo(self, obj, sim):
        vehicle = obj.carlaActor
        ctrl = vehicle.get_control()
        ctrl.gear = self.gear
        vehicle.apply_control(ctrl)


class SetManualFirstGearShiftAction(VehicleAction):  # TODO eliminate
    def applyTo(self, obj, sim):
        ctrl = _carla.VehicleControl(manual_gear_shift=True, gear=1)
        obj.carlaActor.apply_control(ctrl)


class SetTrafficLightAction(VehicleAction):
    """Set the traffic light to desired color. It will only take
    effect if the car is within a given distance of the traffic light.

    Arguments:
        color: the string red/yellow/green/off/unknown
        distance: the maximum distance to search for traffic lights from the current position
    """

    def __init__(self, color, distance=100, group=False):
        self.color = _utils.scenicToCarlaTrafficLightStatus(color)
        if color is None:
            raise ValueError("Color must be red/yellow/green/off/unknown.")
        self.distance = distance

    def applyTo(self, obj, sim):
        traffic_light = obj._getClosestTrafficLight(self.distance)
        if traffic_light is not None:
            traffic_light.set_state(self.color)


class SetAutopilotAction(VehicleAction):
    """Enable or disable CARLA autopilot with optional Traffic Manager settings.

    Arguments:
        enabled: Enable or disable autopilot (bool)
        kwargs: Additional autopilot options such as:

            * ``speed``: Target speed of the car in m/s (default: None). Mutually exclusive with ``vehicle_percentage_speed_difference``.
            * ``vehicle_percentage_speed_difference``: Percentage difference between intended speed and the current speed limit. Can be negative to exceed the speed limit.
            * ``path``: Route for the vehicle to follow (default: None)
            * ``ignore_signs_percentage``: Percentage of ignored traffic signs (default: 0)
            * ``ignore_lights_percentage``: Percentage of ignored traffic lights (default: 0)
            * ``ignore_walkers_percentage``: Percentage of ignored pedestrians (default: 0)
            * ``auto_lane_change``: Whether to allow automatic lane changes (default: False)
    """

    def __init__(self, enabled, **kwargs):
        if not isinstance(enabled, bool):
            raise TypeError("Enabled must be a boolean.")

        self.enabled = enabled

        # Default values for optional parameters
        self.speed = kwargs.get("speed", None)
        self.vehicle_percentage_speed_difference = kwargs.get(
            "vehicle_percentage_speed_difference", None
        )
        self.path = kwargs.get("path", None)
        self.ignore_signs_percentage = kwargs.get("ignore_signs_percentage", 0)
        self.ignore_lights_percentage = kwargs.get("ignore_lights_percentage", 0)
        self.ignore_walkers_percentage = kwargs.get("ignore_walkers_percentage", 0)
        self.auto_lane_change = kwargs.get("auto_lane_change", False)  # Default: False

        if (self.speed is not None) and (
            self.vehicle_percentage_speed_difference is not None
        ):
            raise ValueError(
                "Provide either 'speed' or 'vehicle_percentage_speed_difference', not both."
            )

    def applyTo(self, obj, sim):
        vehicle = obj.carlaActor
        vehicle.set_autopilot(self.enabled, sim.tm.get_port())

        # Apply auto lane change setting
        sim.tm.auto_lane_change(vehicle, self.auto_lane_change)

        if self.path:
            sim.tm.set_route(vehicle, self.path)
        if self.speed is not None:
            sim.tm.set_desired_speed(vehicle, 3.6 * self.speed)
        if self.vehicle_percentage_speed_difference is not None:
            sim.tm.vehicle_percentage_speed_difference(
                vehicle, self.vehicle_percentage_speed_difference
            )

        # Apply traffic management settings
        sim.tm.update_vehicle_lights(vehicle, True)
        sim.tm.ignore_signs_percentage(vehicle, self.ignore_signs_percentage)
        sim.tm.ignore_lights_percentage(vehicle, self.ignore_lights_percentage)
        sim.tm.ignore_walkers_percentage(vehicle, self.ignore_walkers_percentage)


class SetVehicleLightStateAction(VehicleAction):
    """Set the vehicle lights' states.

    Arguments:
        vehicleLightState: Which lights are on.
    """

    def __init__(self, vehicleLightState):
        self.vehicleLightState = vehicleLightState

    def applyTo(self, obj, sim):
        obj.carlaActor.set_light_state(self.vehicleLightState)


#################################################
# Actions available to all carla.Walker objects #
#################################################


class _CarlaPedestrian:
    # Mixin identifying CARLA pedestrians. (see _CarlaVehicle)
    pass


class PedestrianAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _CarlaPedestrian)


class SetJumpAction(PedestrianAction):
    def __init__(self, jump):
        if not isinstance(jump, bool):
            raise TypeError("Jump must be a boolean.")
        self.jump = jump

    def applyTo(self, obj, sim):
        walker = obj.carlaActor
        ctrl = walker.get_control()
        ctrl.jump = self.jump
        walker.apply_control(ctrl)


class SetWalkAction(PedestrianAction):
    def __init__(self, enabled, maxSpeed=1.4):
        if not isinstance(enabled, bool):
            raise TypeError("Enabled must be a boolean.")
        self.enabled = enabled
        self.maxSpeed = maxSpeed

    def applyTo(self, obj, sim):
        controller = obj.carlaController
        if self.enabled:
            controller.start()
            controller.go_to_location(sim.world.get_random_location_from_navigation())
            controller.set_max_speed(self.maxSpeed)
        else:
            controller.stop()


class TrackWaypointsAction(VehicleAction):
    """Track (x, y) waypoints in Scenic coordinates at a target speed."""

    PREDICTIVE_LENGTH = 3.0
    MIN_SPEED = 1.0
    WHEEL_BASE = 3.0
    EPS = 1e-9

    Q = np.array([[1.0, 0.0], [0.0, 3.0]])
    R = np.array([[10.0]])

    def __init__(self, waypoints, cruising_speed=10.0):
        self.waypoints = waypoints
        self.cruising_speed = float(cruising_speed)
        self._route_key = id(waypoints)

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + _math.pi) % (2 * _math.pi) - _math.pi

    @classmethod
    def _lqr_gain(cls, speed):
        A = np.array([[0.0, speed], [0.0, 0.0]])
        B = np.array([[0.0], [speed / cls.WHEEL_BASE]])
        V = linalg.solve_continuous_are(A, B, cls.Q, cls.R)
        return np.linalg.solve(cls.R, B.T @ V)

    def _ensure_state(self, obj):
        if getattr(obj, "_tw_route_key", None) == self._route_key:
            return

        waypoints = np.asarray(self.waypoints, dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[1] < 2 or len(waypoints) < 3:
            raise ValueError(
                "TrackWaypointsAction expects waypoints as an array of shape (N, 2) with N >= 3."
            )

        obj._tw_waypoints = waypoints[:, :2]
        obj._tw_index = 1
        obj._tw_route_key = self._route_key

    def applyTo(self, obj, sim):
        self._ensure_state(obj)
        waypoints = obj._tw_waypoints
        index = obj._tw_index

        actor = obj.carlaActor
        transform = actor.get_transform()

        pos = _utils.carlaToScenicPosition(transform.location)
        vel = _utils.carlaToScenicPosition(actor.get_velocity())

        x, y = pos.x, pos.y
        speed = max(self.MIN_SPEED, _math.hypot(vel.x, vel.y))

        forward = transform.get_forward_vector()
        fx, fy = forward.x, -forward.y
        heading = self._wrap_to_pi(_math.atan2(fx, fy))

        x_f = x + self.PREDICTIVE_LENGTH * fx
        y_f = y + self.PREDICTIVE_LENGTH * fy
        lookahead = np.array([x_f, y_f])

        nearest = int(np.argmin(np.linalg.norm(waypoints - lookahead, axis=1)))
        if index < nearest < (len(waypoints) - 1):
            index = nearest
        index = max(1, min(index, len(waypoints) - 2))
        obj._tw_index = index

        p1 = waypoints[index - 1]
        p2 = waypoints[index]
        p3 = waypoints[index + 1]

        if (np.linalg.norm(p1 - lookahead) - np.linalg.norm(p1 - p2)) > (
            np.linalg.norm(p3 - lookahead) - np.linalg.norm(p3 - p2)
        ):
            p1, p2 = p2, p3

        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        desired_heading = _math.atan2(dx, dy)
        heading_error = self._wrap_to_pi(heading - desired_heading)

        denom = _math.hypot(dx, dy) + self.EPS
        cross_track_error = (dx * y_f - dy * x_f + y2 * x1 - y1 * x2) / denom

        K = self._lqr_gain(speed)
        err = np.array([[-cross_track_error], [heading_error]])
        steer = float(np.clip((-(K @ err)).item(), -1.0, 1.0))

        u = float(np.clip(self.cruising_speed - speed, -1.0, 1.0))
        throttle = max(0.0, u)
        brake = max(0.0, -u)

        ctrl = obj.control
        ctrl.steer = steer
        ctrl.throttle = throttle
        ctrl.brake = brake
