from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.types.state import JointState
from curobo.rollout.rollout_base import Goal

# create a world representation
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

mpc_config = MpcSolverConfig.load_from_robot_config(
        "ur5e.yml",
        world_config,
        store_rollouts=True,
        step_dt=0.03,
    )
mpc = MpcSolver(mpc_config)


retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)


kinematics_state = mpc.rollout_fn.compute_kinematics(
    JointState.from_position(retract_cfg + 0.5, joint_names=mpc.joint_names)
)
goal_pose = kinematics_state.ee_pose.clone()


# read start state from robot here:
start_state = JointState.from_position(retract_cfg, joint_names=mpc.joint_names)

goal = Goal(
    current_state=start_state,
    goal_state=JointState.from_position(retract_cfg, joint_names=mpc.joint_names),
    goal_pose=goal_pose,
)

goal_buffer = mpc.setup_solve_single(goal, 1)


mpc.update_goal(goal_buffer)

# using an iterator to prevent infinite loop
i = 0

while True and i < 100: # control loop

    # To change goal pose:
    # goal_buffer.goal_pose.copy_(new_pose) # for Cartesian Pose
    # mpc.enable_pose_cost(enable=True)
    # mpc.enable_cspace_cost(enable=False)
    #
    # To change goal state:
    # goal_buffer.goal_state.copy_(new_goal_state) # for joint configuration
    # mpc.enable_pose_cost(enable=False)
    # mpc.enable_cspace_cost(enable=True)


    # mpc.update_goal(goal_buffer)


    # read current state from robot
    # current_state = your_robot_api.current_state() # convert to JointState

    current_state = start_state

    # run MPC:
    result = mpc.step(current_state)

    # send action to robot:
    command = result.action
    # your_robot_api.send_command(command)
    print("Command : ", command)
    print(result.metrics.pose_error.item())

    # setting new current state from last command for this example:
    # comment this out and instead read state from your robot.
    start_state = command

    i += 1