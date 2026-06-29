model scenic.simulators.isaac.model

import math

param forkliftAssetPath = "Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd"

LOWERED_LIFT_POSITION = -0.03
CARRY_LIFT_POSITION = 1.85
RELEASE_LIFT_POSITION = 1.45

LOAD_PICKUP_DISTANCE = 2.2
SHELF_PLACE_DISTANCE = 2.35

APPROACH_SPEED = -1.5
SHELF_APPROACH_SPEED = -1.5
BACK_AWAY_SPEED = 0.45
TURN_SPEED = -0.95

MAX_STEERING = 0.55
TURN_TOLERANCE = math.radians(4)

SETTLE_STEPS = 30
LIFT_STEPS = 90
LOWER_STEPS = 90
PLACE_STEPS = 70
BACK_AWAY_STEPS = 360


def wrapAngle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def headingErrorTo(targetHeading, obj):
    return wrapAngle(targetHeading - obj.yaw)


class PalletLoad(IsaacSimObject):
    width: 1.75
    length: 1.0
    height: 0.26
    density: 90
    isaac_asset_path: "Isaac/Props/Pallet/pallet.usd"


class AckermannForkliftB(IsaacSimRobot):
    width: 1.3
    length: 2.8
    height: 2.2
    isaac_asset_path: globalParameters.forkliftAssetPath
    initial_rotation: (-90 deg, 0, 0)
    wheel_controller: "ackermann"

    # ForkliftB geometry/controller metadata.
    wheel_base: 1.65
    track_width: 0.82
    wheel_radius: 0.255
    front_wheel_radius: 0.255
    back_wheel_radius: 0.255

    max_wheel_velocity: 8.0
    max_wheel_rotation_angle: 0.69813
    max_acceleration: 0.8
    max_steering_angle_velocity: 1.0

    # ForkliftB is a single rear-drive / rear-steer vehicle.
    wheel_dof_names: ["back_wheel_drive"]
    steering_dof_names: ["back_wheel_swivel"]
    lift_dof_names: ["lift_joint"]

    steering_sign: 1.0

    def move(self, sim, command):
        """
        command format:
            [steering_angle, speed, lift_position]
        """
        steering, speed, lift_position = command

        if not hasattr(self, "_forklift_cached_dofs"):
            self._drive_dof_indices = sim.backend.articulation_dof_indices(
                sim, self, self.wheel_dof_names
            )
            self._steer_dof_indices = sim.backend.articulation_dof_indices(
                sim, self, self.steering_dof_names
            )
            self._lift_dof_indices = sim.backend.articulation_dof_indices(
                sim, self, self.lift_dof_names
            )
            self._forklift_cached_dofs = True

        if self.controller is None:
            return

        ackermann_command = [
            self.steering_sign * steering,
            0.0,      # steering angle velocity
            speed,    # desired linear speed
            0.0,      # acceleration
            0.0,      # dt
        ]

        steering_positions, wheel_velocities = self.controller.forward(ackermann_command)

        # ForkliftB has one steering swivel and one drive joint.
        steering_position = float(steering_positions[0])
        wheel_velocity = float(wheel_velocities[0])

        action = sim.backend.articulation_action(
            joint_positions=[steering_position, float(lift_position)],
            joint_position_indices=(
                self._steer_dof_indices + self._lift_dof_indices
            ),
            joint_velocities=[wheel_velocity],
            joint_velocity_indices=self._drive_dof_indices,
        )
        sim.backend.apply_articulation_action(
            sim,
            self,
            action,
        )

behavior HoldForks(liftPosition, steps=30):
    for _ in range(steps):
        take applyController([0.0, 0.0, liftPosition])


behavior DriveStraight(speed, liftPosition, steps):
    for _ in range(steps):
        take applyController([0.0, speed, liftPosition])


behavior DriveToObject(target, stopDistance, speed, liftPosition):
    while (distance from ego to target) > stopDistance:
        take applyController([0.0, speed, liftPosition])

    take applyController([0.0, 0.0, liftPosition])


behavior TurnToHeading(targetHeading, liftPosition):
    while abs(headingErrorTo(targetHeading, ego)) > TURN_TOLERANCE:
        error = headingErrorTo(targetHeading, ego)
        steering = math.copysign(MAX_STEERING, error)
        take applyController([steering, TURN_SPEED, liftPosition])

    take applyController([0.0, 0.0, liftPosition])


behavior LowerForks():
    do HoldForks(LOWERED_LIFT_POSITION, SETTLE_STEPS)


behavior LiftForks():
    do HoldForks(CARRY_LIFT_POSITION, LIFT_STEPS)


behavior ReleaseLoad():
    do HoldForks(RELEASE_LIFT_POSITION, LOWER_STEPS)


behavior BackAwayFromShelf():
    do DriveStraight(BACK_AWAY_SPEED, RELEASE_LIFT_POSITION, BACK_AWAY_STEPS)


behavior AckermannForkliftPickup(load, shelf):
    # 1. Lower the forks.
    do LowerForks()

    # 2. Drive straight into the pallet.
    do DriveToObject(load, LOAD_PICKUP_DISTANCE, APPROACH_SPEED, LOWERED_LIFT_POSITION)

    # 3. Lift the pallet.
    do LiftForks()

    # 4. Turn toward the shelf aisle.
    do TurnToHeading(0 deg, CARRY_LIFT_POSITION)

    # 5. Drive toward the shelf.
    do DriveToObject(shelf, SHELF_PLACE_DISTANCE, SHELF_APPROACH_SPEED, CARRY_LIFT_POSITION)

    # 6. Ease forward a little to place the pallet.
    do DriveStraight(SHELF_APPROACH_SPEED * 0.5, CARRY_LIFT_POSITION, PLACE_STEPS)

    # 7. Lower/release the pallet.
    do ReleaseLoad()

    # 8. Back away.
    do BackAwayFromShelf()

    # 9. Stop.
    while True:
        take applyController([0.0, 0.0, RELEASE_LIFT_POSITION])
