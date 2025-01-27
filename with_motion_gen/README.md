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
    - file (ur5e.yml), along 
    - with the world_config. 
    - ```interpolation_dt=0.01```
Specifies the time interval for interpolating the motion plan.
- ```motion_gen```:
- ```motion_gen.warmup()```:
- ```goal_pose```:
- ```start_state```:
- ```result```:
- ```tra```:  

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
> [MotionGenConfig.load_from_robot_config](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenConfig.load_from_robot_config)  
> [MotionGen](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen)
> [MotionGen.plan_single](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen.plan_single)  
> [MotionGenResult](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult)  
> [MotionGenResult.get_interpolated_plan](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult.get_interpolated_plan)  

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