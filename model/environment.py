import random
import math
import numpy as np
import time

from utils.vrep_func import *
from utils.UR5_kinematics import *

pi = math.pi

class Environment():
    def __init__(self, state_dim, action_dim):
        self.action_dim   = action_dim
        self.state_dim    = state_dim
        self._action      = np.zeros([1, action_dim])
        ##############################################################
        self.upper_bound  = np.array([100, 2000, 1000])
        self.lower_bound  = np.array([ 50, 1000,  500])
        ##############################################################
        self.clientID     = Vrep_connect()
        self.random  = True
        self.goal         = np.zeros([1, 6])
        self.eff_errors   = []
        self.track_errors = []

    def reset(self):
        Vrep_disconnect(self.clientID)
        self.clientID = Vrep_connect()
        self.random = True
        state = self.get_state()
        return state

    def step(self, action):
        self.normalize_action(action)

        _reward, done = self.env_sim(self._action)

        next_state = self.get_state()

        _reward = np.array(_reward)
        reward = self.cal_reward(_reward)

        done = True
        return next_state, reward, done

    def get_state(self):
        Vrep_start(self.clientID)
        _, _state, _, _ = Vrep_callLuafunction(self.clientID, 'get_State_Py')
        Vrep_pause(self.clientID)
        return _state
    
    def env_sim(self, _action):
        Vrep_start(self.clientID)
        # (np.array) to (list [[]]) to (list [])
        _action_ = _action.tolist()
        _action = []
        _action.extend(_action_[0])

        _, _, _, _ = Vrep_callLuafunction(self.clientID, 'set_Env_Py', self.save, _action, [], bytearray())
        time.sleep(6)

        Vrep_start(self.clientID)
        _ret, _reward, _, _ = Vrep_callLuafunction(self.clientID, 'return_Error_Py')
        Vrep_pause(self.clientID)
        done = True
        Vrep_stop(self.clientID)
        return _reward, done
    ##############################################################
    def cal_reward(self, _reward):
        force_eror = _reward[0]
        eff_error  = _reward[1]

        force_reward = 0
        eff_reward = 0
        print(f"Average Force Error is : {force_eror}")
        if force_eror > 10: # (N)
            force_reward = force_reward - 10
        elif force_eror > 5:
            force_reward = force_reward - 5
        elif force_eror > 1.5:
            force_reward = force_reward + 5
        else:
            force_reward = force_reward + 10

        print(f"Average EFF Error is : {eff_error}")

        if eff_error > 20: # (mm)
            eff_reward = eff_reward - 10
        elif eff_error > 10:
            eff_reward = eff_reward - 5
        elif eff_error > 5:
            eff_reward = eff_reward + 5
        else:
            eff_reward = eff_reward + 10
        
        if eff_error > 1000:
             force_reward = -10

        reward = force_reward*0.8 + eff_reward*0.2
        
        return reward
    #################################################################

    # def random_goal(self):
    #     # # Define the lower and upper bounds for each dimension
    #     # lower_bounds = [-pi, -pi, -pi, -pi, -pi, -pi]
    #     # upper_bounds = [ pi,  pi,  pi,  pi,  pi,  pi]

    #     # # Generate a random 6-dimensional float list
    #     # Pend = [random.uniform(lower, upper) for lower, upper in zip(lower_bounds, upper_bounds)]

    #     # def check(Pend):
    #     #     XYZ = FK(np.matrix(Pend).T, 0)
    #     #     if XYZ[2] < 0:
    #     #         Pend = self.random_goal()
    #     #     return Pend
        
    #     # Pend = check(Pend) 

    #     randomInt = random.randint(1, 11)

    #     if randomInt == 1:
    #         Pend = [1.47164123, -1.0, 1.76184294, -1.1, -1.57079628, -1.8164123]
    #     elif randomInt == 2:
    #         Pend = [1.47164123, 1.0, -1.56184294, 1, -1.57079628, 1.464123]
    #     elif randomInt == 3:
    #         Pend = [1.47164123, -1.0, 1.66184294, -1.034, -1.57079628, -1.47164123]
    #     elif randomInt == 4:
    #         Pend = [1.47164123, 1.0, -1.46184294, 1.3250, -1.57079628, 1.6164123]
    #     elif randomInt == 5:
    #         Pend = [-1.47164123, -1.0, 1.66184294, -1.350, -1.57079628, -1.6164123]
    #     elif randomInt == 6:
    #         Pend = [-1.47164123, 1.0, -1.86184294, -1.20, -1.57079628, 1.47164123]
    #     elif randomInt == 7:
    #         Pend = [-1.47164123, -1.0, 1.36184294, -1.40, -1.57079628, -1.5164123]
    #     elif randomInt == 8:
    #         Pend = [-1.47164123, 1.0, -1.66184294, 1.50, -1.57079628, -1.47164123]
    #     elif randomInt == 9:
    #         Pend = [-1.47164123, -1.0, 1.66184294, -1.90, -1.57079628, 1.7164123]
    #     else:
    #         Pend = [-1.47164123, 1.0, -1.66184294, 1.60, -1.57079628, -1.47164123]

    #     return Pend
    
    def normalize_action(self, action):
        # map the action from [-1, 1] to [self.lower_bound, self.upper_bound]
        self._action[0, :3] = (self.upper_bound - self.lower_bound) * (action[:3] + 1) / 2 + self.lower_bound

        self._action = np.clip(self._action, self.lower_bound, self.upper_bound)

    # def set_goal(self, goal):
    #     self.random = False
    #     self.goal = goal
    
if __name__ == '__main__':
    env = Environment(2, 2)
    goal = env.random_goal()
    print(goal)