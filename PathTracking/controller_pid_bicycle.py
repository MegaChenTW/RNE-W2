import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPIDBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.1.3: Tune PID Gains
                 # --- TUNING GUIDE ---
                 # 1. Set ki to 0.
                 # 2. Increase kp until the car oscillates predictably.
                 # 3. Increase kd to dampen the oscillations and reduce overshoot.
                 # 4. Once kp/kd are good, add a tiny ki if needed to fix persistent small errors.
                 kp=2.0,  # Start with a moderate Kp.
                 ki=0,  # ALWAYS START TUNING WITH KI = 0.
                 kd=0.1): # A good starting Kd is often ~half of Kp.
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
        target = self.path[min_idx]
        dy = target[1] - y
        dx = target[0] - x
        target_yaw = target[2]
        err = dy * np.cos(target_yaw) - dx * np.sin(target_yaw)
        self.acc_ep += err * self.dt
        next_delta = self.kp * err + self.ki * self.acc_ep + self.kd * (err - self.last_ep) / self.dt
        self.last_ep = err

        # [end] TODO 4.1.3
        return next_delta 
