from scenic.core.simulators import Action


class _WheeledRobot:
    pass


class _HolonomicRobot:
    pass


class _ManipulatorRobot:
    pass


class _QuadrupedRobot:
    pass


class _Robot:
    pass


class ManipulatorRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _ManipulatorRobot)


class RobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _Robot)


class WheeledRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _WheeledRobot)


class HolonomicRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _HolonomicRobot)


class QuadrupedRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _QuadrupedRobot)


class applyController(RobotAction):

    def __init__(self, command):
        self.command = command

    def applyTo(self, obj, sim):
        obj.move(sim, self.command)


class applyWheeledController(WheeledRobotAction):

    def __init__(self, throttle=0, steering=0):
        self.throttle = throttle
        self.steering = steering

    def applyTo(self, obj, sim):
        obj.move(sim, self.throttle, self.steering)


class applyHolonomicController(HolonomicRobotAction):

    def __init__(self, forward_speed=0, lateral_speed=0, yaw_speed=0):
        self.forward_speed = forward_speed
        self.lateral_speed = lateral_speed
        self.yaw_speed = yaw_speed

    def applyTo(self, obj, sim):
        obj.move(sim, self.forward_speed, self.lateral_speed, self.yaw_speed)


class applyPickPlaceController(ManipulatorRobotAction):

    def __init__(
        self,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        self.target_object = target_object
        self.goal_position = goal_position
        self.end_effector_offset = end_effector_offset
        self.end_effector_orientation = end_effector_orientation

    def applyTo(self, obj, sim):
        obj.move(
            sim,
            self.target_object,
            self.goal_position,
            self.end_effector_offset,
            self.end_effector_orientation,
        )


# ---------- generic manipulator (end-effector) actions ----------
#
# Each action advances the manipulator by a single step; behaviors loop over
# them. They delegate to robot class methods, which dispatch to the active
# backend for backend-owned robots like FrankaPanda.


class SetEEPoseAction(ManipulatorRobotAction):
    """Take one IK step moving the end effector toward a world pose.

    ``orientation`` is an Isaac wxyz quaternion, or None to keep the
    backend-defined default (a downward-facing grasp for the Franka).
    """

    def __init__(self, position, orientation=None):
        self.position = position
        self.orientation = orientation

    def applyTo(self, obj, sim):
        obj.move_to_pose(sim, self.position, self.orientation)


class SetArmJointPoseAction(ManipulatorRobotAction):
    """Command explicit arm joint position targets."""

    def __init__(self, joint_positions):
        self.joint_positions = joint_positions

    def applyTo(self, obj, sim):
        obj.set_joint_positions(sim, self.joint_positions)


class OpenGripperAction(ManipulatorRobotAction):

    def applyTo(self, obj, sim):
        obj.set_gripper(sim, True)


class CloseGripperAction(ManipulatorRobotAction):

    def applyTo(self, obj, sim):
        obj.set_gripper(sim, False)


class HoldPositionAction(ManipulatorRobotAction):
    """Hold the current arm targets so the arm settles in place."""

    def applyTo(self, obj, sim):
        obj.hold_position(sim)
