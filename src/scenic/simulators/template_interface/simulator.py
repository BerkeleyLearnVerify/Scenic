"""Simulator interface"""
import logging
import math
import os
import traceback
import warnings

import scenic.core.errors as errors
from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.core.vectors import Vector
from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint


class TemplateSimulator(Simulator):
    """
    Implementation of `Simulator` in scenic.core.simulator.
    At each simulation, Simulator creates a Simulation class
    The Simulation class then runs the actual simulation.

    The only methods that need to be implemented in this class are:

        __init__
        createSimulation
        destroy

    See their respective docstrings for details and how to fill them out.


    The rough execution flow in the background for a n-iteration Scenic run is:

        while number_of_simulations <= n:
            simulation = self.createSimulation()
            run_simulation(simulation)
            number_of_simulations += 1

        self.destory()

    see scenic.core.simulator for more details.
    """

    def __init__(self):
        """
        You can put anything you want in __init__ here, as long as super().__init__() is called.
        Keyword arguments can be added to the signature to set up some desired field, such as the
        size of the simulation timestep
        
        Example 1:
            def __init__(self, timestep=0):
                self.timestep = timestep
                super().__init__()

        Example 2:
            def __init__(self):
                YourSimulatorAPI.turn_on_simulator()
                super().__init__()
        """

        super().__init__()


    def createSimulation(self, scene):
        """
        Creates the Simulation class that will run 1 simulation.
        Depending on your needs and your implementation of the Simulation class,
        you can pass in different arguments.

        Note that you always need to pass in the 'scene' 
        variable for instatiating the Simulation class

        Example:
            return TemplateSimulation(scene, self.timestep, self.simulator_client, **kwargs)
        """
        return TemplateSimulation(scene, **kwargs)

    def destroy(self):
        """
        There is no required code here other than super().destory()
        You can add code does post-processing of the simulation output
        or code that turns of the simulator here

        Example 1:
            YourSimulatorAPI.turn_off_simulator()
            super().destory()

        Example 2:
            YourSimulatorAPI.save_simulation_recordings()
            super().destory()
        """

        super().destroy()


class TemplateSimulation(Simulation):

    """
    Implemation of the Simulation class in scenic.core.simulator

    This class is responsible for:
        1. Running the simulation
        2. Creating the simulation world according to the Scenic program
        3. Passing on Scenic's commands to control the agents to the simulator
        4. Obtaining the world state information from the simulator and passing
            it back to Scenic

    Methods to implement:
        1. __init__
        2. setup
        3. createObjectInSimulator
        4. executeActions
        5. step
        6. getProperties
        7. destory

    See their respective docstrings for details and how to fill them out.
    NOTE: createObjectInSimulator() is already called for you for each object in setup()
    if you call super().setup() in it.

    The rough execution flow when running a simulation is as follows:

        self.setup() # calls createObjectInSimulator inside

        while simulation is running:
            self.executeActions()
            self.step()
            self.getProperties()

        self.destroy()

    See the Simulation class in scenic.core.simulator for more details
    """

    def __init__(self, scene, **kwargs):
        """
        There is no specific requirement for code here
        other than calling super().__init__()

        You can instantiate any variables that would be useful.

        Example:
            def __init__(self, scene, timestep, client_ip):
                self.sim_connection = YourSimulatorAPI.connect_simulator(client_ip)
                YourSimulatorAPI.set_timestep_size(timestep)

                super().__init__()
        """
        super().__init__(scene, **kwargs)

    def setup(self):
        """
        The only requirement here is that you call super().setup(),
        which will call createObjectInSimulator() on each Scenic object for you roughly like this:

            for obj in scene.objects:
                self.createObjectInSimulator(obj)

        You can use setup() as another place to create some useful variables
        or call some useful functions

        Example:
            self.robot_arm_controller = RobotSimulatorAPI.get_arm_controller()
            self.robot_leg_controller = RobotSimulatorAPI.get_leg_controller()
            super().setup()
            return
        """

        super().setup()  # Calls createObjectInSimulator for each object
        return

    def createObjectInSimulator(self, obj):
        """
        Arg
        Object obj: a Scenic obj/agent

        Spawns a single object/agent in the simulator
        with the desired parameters (position, orientation, color, etc.)

        Example:
            if obj is a Car:
                YourSimulatorAPI.spawn_car(
                                position = obj.position,
                                orientation = obj.yaw,
                                color = obj.color,
                                model = obj.modelx
                )

            elif obj is a Human:
                YourSimulatorAPI.spawn_human(
                                position = obj.position 
                                orientation = obj.yaw
                                left_hand_pos = obj.left_hand_pos
                )
        """
        pass

    def executeActions(self, allActions):
        """
        Args:
        Dict[agent_id] -> List[scenic.core.Action] allActions: the dictionary
        with the agent_id as keys and the list of Scenic Actions the agent should carry out
        as the value. 

        Iterates through all the Scenic Actions to be carried out for all the agents.
        For each Scenic Action, this function calls the applyTo() method of the Action's class
        as defined in actions.py.

        Note that when calling applyTo(), we do not actually render the physics of the action.
        That is, the action command is sent, but the world remains frozen.
        The action only gets carried out when we call the step(), where the world unfreezes and
        the simulation physics is advanced by one timestep.

        Example 1:
            for agent, actions in allActions.items():
                for action in actions:
                    action.applyTo(agent, self)
            return

            This is the implementation if you call super().executeActions(allActions)

        Example 2:
            You can implement your Action.applyTo() to return something.
            The following is an example where the applyTo() returns a function that,
            when called in step(), execute the Action

            for agent, actions in allActions.items():
                for action in actions:
                    a = action.applyTo(agent, self)

            self.step_action_buffer.append(a)
            return

            Where we will iterate through the step_action_buffer in step()
            and call each function like this:

            def step(self):
                for a in self.step_action_buffer:
                    a()
        """
        for agent, actions in allActions.items():
            for action in actions:
                action.applyTo(agent, self)
        return

    def step(self):
        """
        This function unfreezes the simulation world and 
        advances the physics by 1 timestep.
        When this function is called, all the Scenic Actions will be executed and
        rendered in simulation.


        Example:
            YourSimulatorAPI.step_physics(self.timestep)
        """
        pass

    def getProperties(self, obj, properties):
        """
        Args:
        Object obj: a Scenic object/agent
        List[String]: a List of property names that needs to be updated

        Returns:
        Dict values: A dictionary that with the property names as keys
        that returns the current value of the property

        Obtains the current values for each Scenic agent/object's properties
        such as current position, orientation, speed, etc. 

        Example 1:
            position = YourSimulatorAPI.get_position(obj.id)
            yaw = YourSimulatorAPI.get_orientation(obj.id)

            values = dict(
                position=position,
                yaw=yaw,
                pitch=0,
                roll=0,
                velocity=velocity,
                speed=speed,
                angularSpeed=angularSpeed,
                angularVelocity=angularVelocity,
            )
        """

    def destroy(self):
        super().destroy()
        return

