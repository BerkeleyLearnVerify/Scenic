from scenic.core.simulators import Action
from scenic.simulators.carla.model import Vehicle
from scenic.domains.driving.roads import LaneSection

class BackgroundDriver:
    """
    Non ego background driver
        Background vehicles are allowed to use autopilot
        behaviors, where low level controllers are handled 
        by the relevent simulator
            (1) Carla if applicable
            (2) METSR by default 
    """
    def setAutoPilot(self, active_autopilot):
        self.autopilot_action = active_autopilot
    
    def setAccelearation(self, acc=0):
        self.target_acceleration = acc
        
class FollowTrajectory(Action):
    """Generate valid trajectory from scenic lanes"""
    def __init__(self, trajectory: list[LaneSection] | None):
        self.trajectory = trajectory
    
    def canBeTakenBy(self, agent):
        return isinstance(agent, Vehicle)
    
    def applyTo(self,obj,sim):
        obj.generateTrajectory(self.trajectory)

class SetAutoPilotAction(Action):
    """ Set autopilot flag """
    def __init__(self, active_autopilot):
        self.active_autopilot = active_autopilot

    def canBeTakenBy(self,agent):
        return isinstance(agent, BackgroundDriver)

    def applyTo(self, obj, sim):
        obj.setAutoPilot(self.active_autopilot)

class SetAccelerationAction:
    """Set obj acclearation for METSR controlled vehs only"""
    def canBeTakenBy(self,agent):
        if hasattr(agent, "carla_actor_flag"):
            if not agent.carla_actor_flag:
                return True
        else: 
            return False
        
    def applyTo(self, obj, sim):
        obj.setAcceleration()
