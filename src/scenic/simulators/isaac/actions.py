from scenic.core.simulators import Action

class _WheeledRobot: pass
class _HolonomicRobot: pass
class _ManipulatorRobot: pass
class _QuadrupedRobot: pass
class _Robot: pass

class ManipulatorRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _ManipulatorRobot)

class RobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _Robot)
    
class applyController(RobotAction):

    def __init__(self, command):
        self.command = command

    def applyTo(self, obj, sim):
        obj.move(sim, self.command)

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
