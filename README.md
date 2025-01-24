# This repository show how to use a robot with isaac sim and the curobo librairie cumotion

The application is sparated in two part :

1. The simulation with isaac sim omniverse
2. The trajectory generation with Curobo and the library Cumotion

[Official IsaacSim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)


[Official IsaacSim core documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=basetask#)


[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

## Simulation with IsaacSim
[Official IsaacSim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)

[!WARNING] If you want to use a robot imported from the urdf importer of IsaacSim you need to make this modification to your .USD file.



I adressed the problem to the Nvidia team you can check this discussion if there are any updates

[Discussion on the Nvidia foru](https://forums.developer.nvidia.com/t/adding-a-new-manipulator-example-doesnt-work/319273/6)


### Task

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