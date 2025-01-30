# Third Party
import torch

# cuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

world_config = {
    "mesh": {
        "base_scene": {
            "pose": [10.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
            "file_path": "scene/nvblox/srl_ur10_bins.obj",
        },
    },
    "cuboid": {
        "table": {
            "dims": [5.0, 5.0, 0.2],  # x, y, z
            "pose": [0.0, 0.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
        },
    },
}

motion_gen_config = MotionGenConfig.load_from_robot_config(
    "ur5e.yml",
    world_config,
    interpolation_dt=0.01,
)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()

goal_pose = Pose.from_list([-0.4, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0])  # x, y, z, qw, qx, qy, qz
start_state = JointState.from_position(
    torch.zeros(1, 6).cuda(),
    joint_names=[
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ],
)

result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=1))
traj = result.get_interpolated_plan()  # result.interpolation_dt has the dt between timesteps
print("Trajectory Generated: ", result.success)
print("Trajectory Length: ", len(traj.position))
print("Trajectory Duration: ", len(traj.position) * result.interpolation_dt)
print("Trajectory: ", traj)