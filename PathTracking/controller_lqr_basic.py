import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBasic(Controller):
    def __init__(self, model, Q=np.eye(2), R=np.eye(1)):
        self.path = None
        self.Q = Q
        
        # 橫向誤差：越大修正越積極
        self.Q[0,0] = 600
        # 航向角誤差：越大車頭越快對齊軌跡
        self.Q[1,1] = 10
        # 控制力道 越大轉向越平緩/遲鈍
        self.R = R * 100
        
        self.pe = 0
        self.pth_e = 0
        self.dt = model.dt
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.current_idx = 0
    
    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    # State: [x, y, yaw, delta, v, l, dt]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]
        
        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0

        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx

        target = self.path[min_idx]
        
        # Optional TODO: LQR Control for Basic Kinematic Model
        # 1. 計算狀態誤差 (State Errors)
        theta_path = np.deg2rad(target[2])
        
        dx = x - target[0]
        dy = y - target[1]
        
        # 橫向誤差 (Cross-track error, e)
        e = dx * (-np.sin(theta_path)) + dy * np.cos(theta_path)
        
        # 航向角誤差 (Heading error, theta_e)
        theta_e_deg = utils.angle_norm(yaw - target[2])
        theta_e = np.deg2rad(theta_e_deg)
        
        state_err = np.array([[e], [theta_e]])
        
        # 2. 建立離散狀態空間矩陣 A, B
        A = np.array([
            [1.0, v * self.dt],
            [0.0, 1.0]
        ])
        B = np.array([
            [0.0],
            [self.dt]
        ])
        
        # 3. 遞迴解 DARE 找出 P
        P = self._solve_DARE(A, B, self.Q, self.R)
        
        # 4. 計算最佳回饋增益 K
        temp = np.linalg.inv(self.R + B.T @ P @ B)
        K = temp @ B.T @ P @ A
        
        # 5. 計算控制輸入 u
        u = -K @ state_err
        
        # 6. 加上前饋控制 (Feedforward) 並轉為度數輸出
        kappa = target[3]
        w_ref = v * kappa # 前饋角速度 (rad/s)
        
        w_rad = w_ref + u[0, 0]
        next_w = np.rad2deg(w_rad)
        
        return next_w
