from scenic.core.simulators import Action


class BackgroundDriver:
    """
    Non ego background driver
        Background vehicles are allowed to use autopilot
        behaviors, where low level controllers are handled 
        by the relevent simulator
            (1) Carla if applicable
            (2) METSR by default 
    """
    def __init__(self):
        carla_actor_flag = False
        finished_route_check = False
    
    def setAutoPilot(self):
        self.autopilot_action = True
    
    def setAccelearation(self, acc=0):
        self.target_acceleration = acc


class SetAutoPilotAction(Action):
    """ Set autopilot flag to be True"""
    def canBeTakenBy(self,agent):
        return isinstance(agent, BackgroundDriver)

    def applyTo(self, obj, sim):
        obj.setAutoPilot()


class SetAccelerationAction:
    def canBeTakenBy(self,agent):
        if hasattr(agent, "carla_actor_flag"):
            if agent.carla_actor_flag:
                return True
        else: 
            return False
        
    def applyTo(self, obj, sim):
        obj.setAcceleration()
