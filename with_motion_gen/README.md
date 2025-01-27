## Motion generation with Curobo

[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

- ```world_config```: This dictionary defines the environment configuration for the simulation or planning  
    - ```mesh``` : Specifies a mesh file for the base scene and its pose in the world
        - ```pose``` : The position (x, y, z) and orientation (qw, qx, qy, qz) of the scene's mesh.
        - ```file_path```: The file path for the 3D scene model
    - ```cuboid```: Describes a cuboid object (e.g., a table) in the environment
        - ```dims``` : The cuboid's dimensions (x, y, z).
        - ```pose``` : The pose of the cuboid in the environment.
- ```motion_gen_config```: Loads motion generation configuration from a robot-specific YAML.
    - ```"ur5e.yml"``` : Path to the robot's configuration YAML file. 
    - ```world_config``` : The environment configuration dictionary 
    - ```interpolation_dt=0.01``` : Specifies the time interval for interpolating the motion plan.  

- **Return type**: 
    - ```MotionGenConfig``` : The loaded motion generation configuration object. 

- ```motion_gen(motion_gen_config)```: Initializes the motion generator using a ```MotionGenConfig``` object.
- ```motion_gen.warmup()```: Prepares the motion generator for motion planning by initializing internal data structures.
- ```goal_pose = Pose.from_list()```: Creates a Pose object from a list of 7 elements representing position and orientation in quaternion format.
- ```start_state = JointState.from_position```: Creates a JointState object with specified joint positions and joint names.
- ```result = motion_gen.plan_single```: Plans a motion from the given start state to the goal pose
- ```traj = result.get_interpolated_plan()```:  Retrieves the interpolated trajectory from the planning result.

To obtain collision-free trajectories, use ```MotionGen```. An example is below,
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

> [!Notes]  
> **Documentations :**  
> [MotionGenConfig](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenConfig)  
> [MotionGenConfig.load_from_robot_config()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenConfig.load_from_robot_config)  
> [MotionGen](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen)
> [MotionGen.plan_single()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen.plan_single)  
> [MotionGenResult](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult)  
> [MotionGenResult.get_interpolated_plan()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult.get_interpolated_plan)  
> [Pose.from_list()](https://curobo.org/_api/curobo.types.math.html#curobo.types.math.Pose.from_list)  
> [JointState.from_position()](https://curobo.org/_api/curobo.types.state.html#curobo.types.state.JointState.from_position)

## Example with ufactory xarm6 robot


For this example I created two classes :

1. ```class CuroboController(BaseController)```
    
    Responsible to initialize the scene with the robot its gripper and the scene and object needed for the task. It initialize also the configuratoin for the motion planning.
2. ```class CuroboPickPlaceTasks(BaseTask)```

    Responsible to initialize all the object and method to perform the task.


## class CuroboController(BaseController)

arguments

- ```def setup_scene()```

    - ```World()``` : 
    - ```add_reference_to_stage()```
    - ```load_yaml()```
    - ```ParallelGripper```
    - ```World.scene.add()```
    - ```SingleManipulator()```
    - ```WorldConfig.from_dict()```
    - ```WorldConfig()```
    - ```World.scene.add_default_ground_plane()```
    - ```USD_Helper.load_stage()```
    - ```USD_Helper.add_world_to_stage()```
    - ```Task_initialize()```

- ```def config_motion_gen()```

    - ```TensorDeviceType()```
    - ```MotionGenConfig.load_from_robot_config()```
    - ```MotionGen()```
    - ```MotionGenPlanConfig()```

- ```def update_world_obstacles()```

- ```def plan()```

    - ```Pose()```
    - ```JointState()```
    - ```result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())```

- ```def forward()```
    - ```ArticulationAction()```
    - ```curobo.articulation_controller.apply_action(art_action)```

## class CuroboPickPlaceTasks(BaseTask)




## Main program




### Main loop