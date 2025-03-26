from abc import ABC, abstractmethod
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.types.state import JointState
from curobo.rollout.rollout_base import Goal

class MotionPlanning():
    def __init__(self, robot_config: str, world_config: dict, step_dt: float = 0.03):
        self.world_config = world_config
        self.mpc_config = MpcSolverConfig.load_from_robot_config(
            robot_config, world_config, store_rollouts=True, step_dt=step_dt
        )
        self.mpc = MpcSolver(self.mpc_config)
        self.retract_cfg = self.mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        self.start_state = JointState.from_position(self.retract_cfg, joint_names=self.mpc.joint_names)
        self.goal_pose = self.compute_goal_pose()
        self.goal_buffer = self.setup_goal()
    
    def compute_goal_pose(self):
        kinematics_state = self.mpc.rollout_fn.compute_kinematics(
            JointState.from_position(self.retract_cfg + 0.5, joint_names=self.mpc.joint_names)
        )
        return kinematics_state.ee_pose.clone()
    
    def setup_goal(self):
        goal = Goal(
            current_state=self.start_state,
            goal_state=JointState.from_position(self.retract_cfg, joint_names=self.mpc.joint_names),
            goal_pose=self.goal_pose,
        )
        return self.mpc.setup_solve_single(goal, 1)
    
    def update_goal(self):
        self.mpc.update_goal(self.goal_buffer)
    
    def run_motion_planning(self, max_iterations: int = 100):
        for i in range(max_iterations):
            current_state = self.start_state  # Replace with real robot state reading
            result = self.mpc.step(current_state)
            command = result.action
            print("Command :", command)
            print("Pose Error:", result.metrics.pose_error.item())
            self.start_state = command  # Replace with actual state feedback


    def motion_gen_cfg(self):
        pass
    

    def update_world_obstacles(self):
        pass 
    

    def forward_sim(self):
        pass
    

    def forward_real(self):
        
        pass
    
           
    def plan_sim(self):
        pass
    
           
    def plan_real(self):
        pass
    
    
    def attach_object(self):    
        pass
    
    
    def detach_object(self):
        pass

# Example usage
if __name__ == "__main__":
    world_config = {
        "cuboid": {
            "table": {"dims": [2, 2, 0.2], "pose": [0.4, 0.0, -0.1, 1, 0, 0, 0]},
            "cube_1": {"dims": [0.1, 0.1, 0.2], "pose": [0.4, 0.0, 0.5, 1, 0, 0, 0]},
        },
        "mesh": {
            "scene": {
                "pose": [1.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
                "file_path": "scene/nvblox/srl_ur10_bins.obj",
            }
        },
    }
    planner = MotionPlanning("ur5e.yml", world_config)
    planner.run_motion_planning()