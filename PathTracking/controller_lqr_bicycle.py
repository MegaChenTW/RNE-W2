import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, model, Q=None, R=None, control_state='steering_angle'):
        self.path = None
        if control_state == 'steering_angle':
            self.Q = np.eye(2)
            self.R = np.eye(1)
            # TODO 4.4.1: Tune LQR Gains
            self.Q[0,0] = 100  # Cross-track error (e) 權重
            self.Q[1,1] = 10   # Heading error (theta_e) 權重
            self.R[0,0] = 100  # 轉向力道 (delta) 權重
        elif control_state == 'steering_angular_velocity':
            self.Q = np.eye(3)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 100  # Cross-track error (e) 權重
            self.Q[1,1] = 100   # Heading error (theta_e) 權重
            self.Q[2,2] = 100    # Steering angle error (delta_e) 權重
            self.R[0,0] = 5   # Steering angular velocity 權重
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.dt = model.dt
        self.l = model.l
        self.control_state = control_state
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
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

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]
        yaw = utils.angle_norm(yaw)

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        if self.control_state == 'steering_angle':
            # TODO 4.4.1: LQR Control for Bicycle Kinematic Model with steering angle as control input
            # 1. 計算狀態誤差 (State Errors)
            theta_path = np.deg2rad(target[2])
            dx = x - target[0]
            dy = y - target[1]
            
            # 橫向誤差 e
            e = dx * (-np.sin(theta_path)) + dy * np.cos(theta_path)
            
            # 航向角誤差 theta_e
            theta_e_deg = utils.angle_norm(yaw - target[2])
            theta_e = np.deg2rad(theta_e_deg)
            
            state_err = np.array([[e], [theta_e]])
            
            # 2. 建立離散狀態空間模型 (A, B)
            v_safe = max(v, 1e-5) # 避免速度為 0 時 B 矩陣為全零導致不收斂
            A = np.array([[1.0, v_safe * self.dt],
                          [0.0, 1.0]])
            B = np.array([[0.0],
                          [v_safe * self.dt / self.l]])
            
            # 3. 解離散代數黎卡提方程式 (DARE) 求 P
            P = self._solve_DARE(A, B, self.Q, self.R)
            
            # 4. 計算最佳回饋增益 K 與 控制輸入 u
            temp = np.linalg.inv(self.R + B.T @ P @ B)
            K = temp @ B.T @ P @ A
            u = -K @ state_err
            
            # 5. 前饋控制 (Feedforward) + LQR 回饋控制 (Feedback)
            kappa = target[3]
            delta_ff = np.arctan2(self.l * kappa, 1.0) # 根據曲率的前饋轉向角
            
            next_delta_rad = delta_ff + u[0, 0]
            next_delta = np.rad2deg(next_delta_rad)
            # [end] TODO 4.4.1
        elif self.control_state == 'steering_angular_velocity':
            # TODO 4.4.4: LQR Control for Bicycle Kinematic Model with steering angular velocity as control input
            # 1. 計算狀態誤差 (State Errors)
            theta_path = np.deg2rad(target[2])
            dx = x - target[0]
            dy = y - target[1]
            
            # 橫向誤差 e
            e = dx * (-np.sin(theta_path)) + dy * np.cos(theta_path)
            
            # 航向角誤差 theta_e
            theta_e_deg = utils.angle_norm(yaw - target[2])
            theta_e = np.deg2rad(theta_e_deg)

            # 轉向角誤差 delta_e
            kappa = target[3]
            delta_ref = np.arctan2(self.l * kappa, 1.0)
            delta_rad = np.deg2rad(delta)
            delta_e = delta_rad - delta_ref
            
            state_err = np.array([[e], [theta_e], [delta_e]])
            
            # 2. 建立離散狀態空間模型 (A, B)
            v_safe = max(v, 1e-5) # 避免速度為 0
            
            A = np.array([
                [1.0, v_safe * self.dt, 0.0],
                [0.0, 1.0, v_safe * self.dt / self.l],
                [0.0, 0.0, 1.0]
            ])
            B = np.array([
                [0.0],
                [0.0],
                [self.dt]
            ])
            
            # 3. 解離散代數黎卡提方程式 (DARE) 求 P
            P = self._solve_DARE(A, B, self.Q, self.R)
            
            # 4. 計算最佳回饋增益 K 與 控制輸入 u
            temp = np.linalg.inv(self.R + B.T @ P @ B)
            K = temp @ B.T @ P @ A
            u = -K @ state_err
            
            # 5. 將轉向角速度 (u) 積分變回轉向角度 (delta)
            next_delta_rad = delta_rad + u[0, 0] * self.dt
            next_delta = np.rad2deg(next_delta_rad)
            # [end] TODO 4.4.4
        
        return next_delta
