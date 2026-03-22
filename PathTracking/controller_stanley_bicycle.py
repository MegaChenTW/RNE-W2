import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Stanley Gain
                 kp=5):
        self.path = None
        self.kp = kp
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        # Search Front Wheel Target Locally
        front_x = x + self.l*np.cos(np.deg2rad(yaw))
        front_y = y + self.l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta)) if np.cos(np.deg2rad(delta)) != 0 else v
        
        min_idx, min_dist = utils.search_nearest_local(self.path, (front_x,front_y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx]

        # TODO 4.3.1: Stanley Control for Bicycle Kinematic Model
        # 1. 計算 Heading Error (theta_e)
        theta_path = target[2]
        theta_e_deg = utils.angle_norm(theta_path - yaw)
        theta_e_rad = np.deg2rad(theta_e_deg)

        # 2. 計算 Cross-track Error (e)
        theta_path_rad = np.deg2rad(theta_path)
        e_signed = (front_x - target[0]) * (-np.sin(theta_path_rad)) + (front_y - target[1]) * np.cos(theta_path_rad)
        
        # 3. 計算修正的 Steering Angle (delta)
        vf_safe = max(vf, 1e-5) # 避免除以零
        delta_e_rad = np.arctan2(-self.kp * e_signed, vf_safe)
        
        next_delta = np.rad2deg(theta_e_rad + delta_e_rad)
        # [end] TODO 4.3.1
    
        return next_delta
