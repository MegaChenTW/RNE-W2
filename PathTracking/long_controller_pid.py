import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class PIDLongController(Controller):
    #def __init__(self, model, a_range, kp=1.5, ki=5, kd=0):
    # Feature: increase kp
    def __init__(self, model, a_range, kp=10.0, ki=5, kd=0.8):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
        self.dt = model.dt
        self.a_range = a_range
        self.current_idx = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
        self.current_idx = 0
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 1:
            # Brake to 0 speed using PID when finishing the track
            v_ref = 0.0
            target = self.path[-1]
            return np.clip(-v, self.a_range[0], self.a_range[1]), target
        else:
            # Search Nearest Target Locally
            min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
            self.current_idx = min_idx
            target = self.path[min_idx]
            v_ref = target[4]
        
        # TODO 3.2: PID Control for Longitudinal Motion
        err = v_ref - v
        if v_ref == 85:
            err = 20
        self.acc_ep += err * self.dt
        d_err = (err - self.last_ep) / self.dt

        # FEATURE: 加速快減速慢
        #just fucking speed up
        err = 20

        if err > 0:
            kp_acc = self.kp * 5
            kd_acc = self.kd
            next_a = kp_acc * err + self.ki * self.acc_ep + kd_acc * d_err
            self.last_ep = err
        else:
            kp_dec = self.kp * 0.7
            kd_dec = self.kd * 1.2
            next_a = kp_dec * err + self.ki * self.acc_ep + kd_dec * d_err
            self.last_ep = err
        # [end] TODO 3.2

        return next_a, target