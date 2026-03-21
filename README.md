# Notices

## PID tuning


    Bicycle PID: 
        kp, ki, kd = 1, 0.6, 0.6
        err: 目標點相對於車頭朝向（車體座標系）的橫向誤差（Lateral Error）
        ```python
        theta_target = np.arctan2(target[1] - y, target[0] - x)
        theta_err = theta_target - np.deg2rad(yaw)

        target_dist = np.hypot(target[0] - x, target[1] - y)
        err = target_dist * np.sin(theta_err)
        ```

## Hint

How the program choose simulator, kinematic model, and controller.

### Knowing this will make you understand the structure faster

```python
# in navigation.py:
            if args.simulator == "basic":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        }
                next_v, target = long_controller.feedback(info)
                next_w = controller.feedback(info)
                command = ControlState("basic", next_v, next_w)
            elif args.simulator == "diff_drive":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        }
                next_v, target = long_controller.feedback(info)
                next_w = controller.feedback(info)
                # TODO 2.2.2: Map [v, w] to [lw, rw]
                next_lw = (next_v - next_w * simulator.l) / simulator.wu  
                next_rw = (next_v + next_w * simulator.l) / simulator.wu 
                # [end] TODO 2.2.2
                command = ControlState("diff_drive", next_lw, next_rw)
            elif args.simulator == "bicycle":
                info = {"x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                        "v": simulator.state.v,
                        "delta": simulator.cstate.delta,
                        }
                next_a, target = long_controller.feedback(info)
                info["v"] = info["v"] + next_a * simulator.model.dt
                next_delta = controller.feedback(info)
                command = ControlState("bicycle", next_a, next_delta)

# and later we have:    
        if args.simulator == "basic":
            from Simulation.simulator_basic import SimulatorBasic
            simulator = SimulatorBasic()
            from PathTracking.long_controller_vanilla import VanillaLongController
            l_controller = VanillaLongController()
            if args.controller == "pid":
                from PathTracking.controller_pid_basic import ControllerPIDBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_basic import ControllerPurePursuitBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_basic import ControllerLQRBasic as Controller
                controller = Controller(model=simulator.model)
            else:
                raise NameError("Unknown controller!!")
        elif args.simulator == "diff_drive":
            from Simulation.simulator_differential_drive import SimulatorDifferentialDrive
            simulator = SimulatorDifferentialDrive()
            from PathTracking.long_controller_vanilla import VanillaLongController
            l_controller = VanillaLongController()
            if args.controller == "pid":
                from PathTracking.controller_pid_basic import ControllerPIDBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_basic import ControllerPurePursuitBasic as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_basic import ControllerLQRBasic as Controller
                controller = Controller(model=simulator.model)
            else:
                raise NameError("Unknown controller!!")
        elif args.simulator == "bicycle":
            from Simulation.simulator_bicycle import SimulatorBicycle 
            simulator = SimulatorBicycle()
            from PathTracking.long_controller_pid import PIDLongController
            l_controller = PIDLongController(model=simulator.model, a_range=simulator.a_range)
            if args.controller == "pid":
                from PathTracking.controller_pid_bicycle import ControllerPIDBicycle as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "pure_pursuit":
                from PathTracking.controller_pure_pursuit_bicycle import ControllerPurePursuitBicycle as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "stanley":
                from PathTracking.controller_stanley_bicycle import ControllerStanleyBicycle as Controller
                controller = Controller(model=simulator.model)
            elif args.controller == "lqr":
                from PathTracking.controller_lqr_bicycle import ControllerLQRBicycle as Controller
                controller = Controller(model=simulator.model, control_state=args.lqr_control_state)
            else:
                raise NameError("Unknown controller!!")
        else:
            raise NameError("Unknown simulator!!")
        return simulator, controller, l_controller, None    
```