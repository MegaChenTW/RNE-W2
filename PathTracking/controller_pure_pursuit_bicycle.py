import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Pure Pursuit Gain
                 kp=0.1, Lfc=0.5):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.dt = model.dt
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        Ld = self.kp*v + self.Lfc
        
        # TODO 4.3.1: Pure Pursuit Control for Bicycle Kinematic Model
        # 預設目標點為路徑終點，避免 Ld 太大時找不到點而瞄準腳下
        target_idx = len(self.path) - 1 
        for i in range(self.current_idx, len(self.path)):
            dist = np.sqrt((self.path[i][0] - x) **2 + (self.path[i][1] - y) **2)
            if(dist >= Ld):
                target_idx = i
                break
        
        tx, ty = self.path[target_idx][0], self.path[target_idx][1]
        
        # 1. 確保 yaw 轉為弧度參與運算
        alpha = np.atan2(ty - y, tx - x) - np.deg2rad(yaw)
        # 將 alpha 收斂到 [-pi, pi] 之間，避免 360 度邊界突變
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # 2. Pure Pursuit 公式算出來為弧度，需轉為 Degree 交給模擬器
        next_delta = np.rad2deg(np.arctan2(2.0 * self.l * np.sin(alpha), Ld))
        # [end] TODO 4.3.1
        return next_delta
