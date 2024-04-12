from enum import IntEnum
import random
import numpy as np

import carla
from agents.navigation.agent import *

from pid_follow_controller import *

class LaneChange(IntEnum):
    LEFT = -1
    RIGHT = 1

'''Child of PIDAgent that also overtakes any blocking vehicles.'''
class OvertakeAgent(PIDAgent):

    def __init__(self, vehicle, opt_dict):
        super().__init__(vehicle, opt_dict)

        self.location_pid_dict = {
            'K_P': 1.0,
            'K_D': 0.05,
            'K_I': 1,
            'dt': 0.05
        }
        # Distance to maintain from other vehicles.
        self.clear_dist = 10.0

        if opt_dict:
            if 'target_speed' in opt_dict:
                self.location_pid_dict['dt'] = 1.0 / self.target_speed
            if 'clear_dist' in opt_dict:
                self.clear_dist = opt_dict['clear_dist']
            if 'location_pid_dict' in opt_dict:
                self.location_pid_dict = opt_dict['location_pid_dict']

        self.controller = PIDFollowController(
            vehicle,
            clear_dist=self.clear_dist,
            args_lateral=self.lateral_pid_dict,
            args_longitudinal=self.longitudinal_pid_dict,
            args_location=self.location_pid_dict)
        # Magnitude of lane_state is how far vehicle is from its
        # desired lane. Negative/positive means to the left/right
        # of desired lane, respectively.
        self.lane_state = 0
        self.is_changing_lane = False


    def get_lane_change_w(self, cur_w, lane_change):
        # Return waypoint corresponding to LANE_CHANGE (either LEFT or RIGHT)
        # from waypoint CUR_W. If lane change illegal or unsafe, returns None.
        next_w = None
        lane_safe = lambda w: self.lane_clear(w.transform.location)[0] and \
            self.lane_clear(w.transform.location,
                            min_dist=10.0*self.clear_dist,
                            forward=w.transform.get_forward_vector())[0]
        if lane_change is LaneChange.LEFT and cur_w.lane_change & carla.LaneChange.Left:
            if lane_safe(cur_w.get_left_lane()):
                next_w = cur_w.get_left_lane()
        elif cur_w.lane_change & carla.LaneChange.Right:
            if lane_safe(cur_w.get_right_lane()):
                next_w = cur_w.get_right_lane()
        return next_w


    def change_lane(self, lane_change=None):
        ''' By default, picks left or right at random. If no possible lane change,
        does nothing.'''
        if lane_change:
            potential_change = [lane_change]
        else:
            potential_change = [LaneChange.LEFT, LaneChange.RIGHT]
        potential_w = []
        cur_w = self.waypoints[0]
        for change in potential_change:
            next_w = self.get_lane_change_w(cur_w, change)
            if next_w:
                potential_w.append((change, next_w))
        if potential_w:
            lane_change, next_w = random.choice(potential_w)
            self.is_changing_lane = True
            self.lane_state += lane_change
            self.waypoints = [cur_w, next_w]


    def lane_clear(self, location, min_dist=None, forward=None):
        '''
        Check that the lane LOCATION is in is clear of vehicles.
        Lane is clear if no vehicles within MAX_DIST away.
        If not clear, return tuple of False and blocking vehicle.
        If FORWARD, only check vehicles along direction of FORWARD.
        '''
        def norm(vec):
            return np.sqrt(vec.x ** 2 + vec.y ** 2 + vec.z ** 2)

        def norm_dot(a, b):
            a /= norm(a)
            b /= norm(b)
            dot = a.x * b.x + a.y * b.y + a.z * b.z
            return dot
        
        if not min_dist:
            min_dist = self.clear_dist
            
        lane_id = self._map.get_waypoint(location).lane_id
        vehicles = self._world.get_actors().filter('*vehicle*')
        for v in vehicles:
            # Check if v is self.
            if v.id == self._vehicle.id:
                continue

            # Check if v is on same lane as self.
            v_loc = v.get_location()
            v_w = self._map.get_waypoint(v_loc)
            if lane_id != v_w.lane_id:
                continue

            if forward and norm_dot(forward, v_loc - location) < 0.0:
                continue

            if v_loc.distance(location) < min_dist:
                return (False, v)
        return (True, None)
        

    def run_step(self):
        self_loc = self._vehicle.get_location()
        self_forward = self._vehicle.get_transform().get_forward_vector()
        is_clear, blocker = self.lane_clear(self_loc,
                                            min_dist=2.0*self.clear_dist,
                                            forward=self_forward)
        super().run_step()
        cur_w = self.waypoints[0]
        if self.lane_state != 0:
            speed = self.target_speed * 1.5
        else:
            speed = self.target_speed

        if not is_clear:
            if not self.is_changing_lane:
                self.change_lane()
                    
            return self.controller.run_step(speed,
                                            self.waypoints[0],
                                            blocker.get_location())
        else:
            self.is_changing_lane = False
            if self.lane_state != 0:
                lane_change = LaneChange(-np.sign(self.lane_state))
                self.change_lane(lane_change=lane_change)
                
            return self.controller.run_step(speed,
                                            self.waypoints[0])

