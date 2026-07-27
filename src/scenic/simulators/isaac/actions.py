from scenic.core.simulators import Action


class ManipulatorTimeout(Exception):
    """Raised when a manipulator end-effector move does not converge within maxSteps."""


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


class ApplyControllerAction(RobotAction):

    def __init__(self, command):
        self.command = command

    def applyTo(self, obj, sim):
        obj.move(sim, self.command)


class ApplyPickPlaceControllerAction(ManipulatorRobotAction):

    def __init__(
        self,
        targetObject,
        goalPosition,
        endEffectorOffset=None,
        endEffectorOrientation=None,
    ):
        self.targetObject = targetObject
        self.goalPosition = goalPosition
        self.endEffectorOffset = endEffectorOffset
        self.endEffectorOrientation = endEffectorOrientation

    def applyTo(self, obj, sim):
        obj.move(
            sim,
            self.targetObject,
            self.goalPosition,
            self.endEffectorOffset,
            self.endEffectorOrientation,
        )


# ---------- generic manipulator (end-effector) actions ----------
#
# Each action advances the manipulator by a single step; behaviors loop over
# them. They delegate to robot class methods, which dispatch to the active
# backend for backend-owned robots like FrankaPanda.


class MoveToEEPoseAction(ManipulatorRobotAction):
    """Take one IK step moving the end effector toward a world pose.

    ``orientation`` is an Isaac wxyz quaternion, or None to keep the
    backend-defined default (a downward-facing grasp for the Franka).
    """

    def __init__(self, position, orientation=None):
        self.position = position
        self.orientation = orientation

    def applyTo(self, obj, sim):
        obj.moveToPose(sim, self.position, self.orientation)


class SetArmJointPoseAction(ManipulatorRobotAction):
    """Command explicit arm joint position targets."""

    def __init__(self, jointPositions):
        self.jointPositions = jointPositions

    def applyTo(self, obj, sim):
        obj.setJointPositions(sim, self.jointPositions)


class OpenGripperAction(ManipulatorRobotAction):

    def applyTo(self, obj, sim):
        obj.setGripper(sim, True)


class CloseGripperAction(ManipulatorRobotAction):

    def applyTo(self, obj, sim):
        obj.setGripper(sim, False)


class HoldPositionAction(ManipulatorRobotAction):
    """Hold the current arm targets so the arm settles in place."""

    def applyTo(self, obj, sim):
        obj.holdPosition(sim)
