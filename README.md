# This repository show how to use a robot with isaac sim and the curobo librairie cumotion

The application is sparated in two part :

1. The simulation with isaac sim omniverse
2. The trajectory generation with Curobo and the library Cumotion

[Official IsaacSim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)


[Official IsaacSim core documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=basetask#)


[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

## Simulation with IsaacSim
[Official IsaacSim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)

> [!WARNING] 
> **If you want to use a robot imported from the urdf importer of IsaacSim you need to make this modification to your .USD file.**


1. Open the **.usd** file of your robot with isaac sim

    ![Alt text](/img/usd_before%20modification.png "usd file before modification")

2. Place the frame of your and effector here **xarmlink_tcp** under the root joint

    ![Alt text](/img/frame_tree.png "usd file after modification")

> [!Note]
> I adressed the problem to the Nvidia team you can check this discussion if there are any updates

[Discussion on the Nvidia forum](https://forums.developer.nvidia.com/t/adding-a-new-manipulator-example-doesnt-work/319273/6)

### rmpflow
```yaml
joint_limit_buffers: [.01, .01, .01, .01, .01, .01]
rmp_params:
    cspace_target_rmp:
        metric_scalar: 50.
        position_gain: 100.
        damping_gain: 50.
        robust_position_term_thresh: .5
        inertia: 1.
    cspace_trajectory_rmp:
        p_gain: 100.
        d_gain: 10.
        ff_gain: .25
        weight: 50.
    cspace_affine_rmp:
        final_handover_time_std_dev: .25
        weight: 2000.
    joint_limit_rmp:
        metric_scalar: 1000.
        metric_length_scale: .01
        metric_exploder_eps: 1e-3
        metric_velocity_gate_length_scale: .01
        accel_damper_gain: 200.
        accel_potential_gain: 1.
        accel_potential_exploder_length_scale: .1
        accel_potential_exploder_eps: 1e-2
    joint_velocity_cap_rmp:
        max_velocity: 1.
        velocity_damping_region: .3
        damping_gain: 1000.0
        metric_weight: 100.
    target_rmp:
        accel_p_gain: 30.
        accel_d_gain: 85.
        accel_norm_eps: .075
        metric_alpha_length_scale: .05
        min_metric_alpha: .01
        max_metric_scalar: 10000
        min_metric_scalar: 2500
        proximity_metric_boost_scalar: 20.
        proximity_metric_boost_length_scale: .02
        xi_estimator_gate_std_dev: 20000.
        accept_user_weights: false
    axis_target_rmp:
        accel_p_gain: 210.
        accel_d_gain: 60.
        metric_scalar: 10
        proximity_metric_boost_scalar: 3000.
        proximity_metric_boost_length_scale: .08
        xi_estimator_gate_std_dev: 20000.
        accept_user_weights: false
    collision_rmp:
        damping_gain: 50.
        damping_std_dev: .04
        damping_robustness_eps: 1e-2
        damping_velocity_gate_length_scale: .01
        repulsion_gain: 800.
        repulsion_std_dev: .01
        metric_modulation_radius: .5
        metric_scalar: 10000.
        metric_exploder_std_dev: .02
        metric_exploder_eps: .001
    damping_rmp:
        accel_d_gain: 30.
        metric_scalar: 50.
        inertia: 100.
canonical_resolve:
    max_acceleration_norm: 50.
    projection_tolerance: .01
    verbose: false
body_cylinders:
    - name: base
      pt1: [0,0,.333]
      pt2: [0,0,0.]
      radius: .05
body_collision_controllers:
    - name: onrobot_rg6_base_link
      radius: .05
```

### Task
```python
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "denso_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change the asset path here
        asset_path = "/home/user_name/cobotta_pro_900/cobotta_pro_900/cobotta_pro_900.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")
        gripper = ParallelGripper(
            end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.2, 0.2]) )
        manipulator = SingleManipulator(prim_path="/World/cobotta",
                                        name="cobotta_robot",
                                        end_effector_prim_name="onrobot_rg6_base_link",
                                        gripper=gripper)
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
```

### Controller

### Main programme



## Motion generation with Curobo

[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

To obtain collision-free trajectories, use MotionGen. An example is below,
```python
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
```

## Example with ufactory xarm6 robot


For this example I created two classes :

1. ```class CuroboController(BaseController)```
    
    Responsible to initialize the scene with the robot its gripper and the scene and object needed for the task. It initialize also the configuratoin for the motion planning.
2. ```class CuroboPickPlaceTasks(BaseTask)```

    Responsible to initialize all the object and method to perform the task.


## class CuroboController(BaseController)

arguments

def setup_scene()

- World()
- add_reference_to_stage()
- load_yaml()
- ParallelGripper
- World.scene.add()
- SingleManipulator()
- WorldConfig.from_dict()
- WorldConfig()
- World.scene.add_default_ground_plane()
- USD_Helper.load_stage()
- USD_Helper.add_world_to_stage()
- Task_initialize()

def config_motion_gen()

- TensorDeviceType()
- MotionGenConfig.load_from_robot_config()
- MotionGen()
- MotionGenPlanConfig()

def update_world_obstacles()

def plan()

- Pose()
- JointState()
- result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())

def forward()
- ArticulationAction()
- curobo.articulation_controller.apply_action(art_action) 

## 