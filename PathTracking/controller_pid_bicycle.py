import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPIDBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.1.3: Tune PID Gains
                 # 2.5 0.6 0.6 : 0.5130
                 kp=2.5,
                 ki=0.6,  
                 kd=0.6):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
        self.dt = model.dt
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
            return None
        
        # Extract State
        x, y, yaw = info["x"], info["y"], info["yaw"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        # Search Nearest Target Locally
        # Mega: search from front wheel
        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        # TODO 4.1.3: PID Control for Bicycle Kinematic Model
        # 1. 前瞻目標點 (Lookahead)：往前方看向幾個點，解決離散跳動造成的 Kd 震盪
        target_idx = min(min_idx + 2, len(self.path) - 1)
        target = self.path[target_idx]
        
        theta_target = np.arctan2(target[1] - y, target[0] - x)
        theta_err = theta_target - np.deg2rad(yaw)
        
        target_dist = np.hypot(target[0] - x, target[1] - y)
        err = target_dist * np.sin(theta_err)
        
        self.acc_ep += err * self.dt
        # 限制出彎時 Ki 累積的極限值
        #self.acc_ep = np.clip(self.acc_ep, -15.0, 15.0)
        
        next_delta = self.kp * err + self.ki * self.acc_ep + self.kd * (err - self.last_ep) / self.dt
        self.last_ep = err

        # [end] TODO 4.1.3
        return next_delta 
