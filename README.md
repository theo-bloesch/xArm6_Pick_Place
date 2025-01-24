# This repository show how to use a robot with isaac sim and the curobo librairie cumotion

The application is sparated in two part :

1. The simulation with isaac sim omniverse (trajectory generation with rmpflow)
2. The trajectory generation with Curobo and the library Cumotion (with motion_gen)

[Official IsaacSim tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)


[Official IsaacSim core documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=basetask#)


[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

> [!WARNING] 
> **If you want to use a robot imported from the urdf importer of IsaacSim you need to make this modification to your .USD file.**


1. Open the **.usd** file of your robot with isaac sim

    ![Alt text](/img/usd_before%20modification.png "usd file before modification")

2. Place the frame of your and effector here **xarmlink_tcp** under the root joint

    ![Alt text](/img/frame_tree.png "usd file after modification")

> [!Note]
> **I adressed the problem to the Nvidia team you can check this discussion if there are any updates.** [Discussion on the Nvidia forum](https://forums.developer.nvidia.com/t/adding-a-new-manipulator-example-doesnt-work/319273/6)

## Simulation with IsaacSim and path generation with rmpflow
[Official IsaacSim documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)

Once this modification has been made we can begin to simulate our robot. For that we need several file or code part. Here we decided to make all the code in two file but in the Isaac sim exemple the split the code in several file.

We are going to make a simple pick and place task to verify that our robot works correctly.

First we need to define our robot description to pass it to the rmpflow motion generator.
For that we create a folder rmp and inside it a file named ```robot_descriptor.yaml```. Make sure that the joint are the same that the one define in your **.usd**

### rmp/robot_descriptor.yaml

```yaml
api_version: 1.0

cspace:
    - xarm6joint1
    - xarm6joint2
    - xarm6joint3
    - xarm6joint4
    - xarm6joint5
    - xarm6joint6

root_link: world

default_q: [

    0.00, 0.00, 0.00, 0.00, 0.00, 0.00

]

cspace_to_urdf_rules: []

composite_task_spaces: []
```

Then in the same folder create the file ```denso_rmpflow_common.yaml```

### rmp/denso_rmpflow_common.yaml

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
    - name: xarm6link_eef
      radius: .05
```
Then we are going to create the main programme in the file ```pick_place_test.py``` and we will need several classe for that :

### class PickPlace(tasks.PickPlace)

For this exemple we are going to use the **PickPlace task** from **IsaacSim**. 

we explaine the different argument and méthode:

```class PickPlace(tasks.PickPlace)```

The class inherits from tasks.PickPlace, meaning it extends the base pick-and-place functionality provided by that class.
- ```__init__()```: Initialisation of our own PickPlace task to extends the base pick-and-place functionality
    - ```name```:
    - ```cube_initial_position```: The pick position
    - ```cube_initial_orientation```: The initial orientation of the cube (default: [0.3, 0.3, 0.3]).
    - ```target_position```: The position where we want the cube to be
    - ```offset```: The end effector (gripper) offset position relative to the position of the cube. The cube position is located in its center.
    - ```tasks.PickPlace.__init__()```: Initialisation of the PickPlace task from IsaaSim
        - ...
        - ```cube_size```: Calls the parent class's constructor with the given arguments and adds a default cube_size (5.15 cm x 5.15 cm x 10 cm).



- ```set_robot()```: This method defines the robot and its components (like the gripper) that will perform the task. It sets the robot up in the simulation environment using an absolute path to the robot's asset file.
    - ```add_reference_to_stage```: Loads the robot's asset (USD file) into the simulation stage at /World/xarm6
    > [!Note]
    > - The path is absolute, and the user must ensure the file exists at the specified location.
    > - You can choose the prim_path where you want the robot in the stage but be carefule it must be the same everywhere (```add_reference_to_stage(usd_path=asset_path, prim_path="/World/xarm6"```)
    - ```ParallelGripper```: Defines a parallel gripper with:
        - ```end_effector_prim_path``` :The end-effector's path in the simulation (xarm6link_tcp).
        - ```joint_opened_positions```: joint_prim_namesGripper joint names for control.
        - ```joint_closed_positions``` : Joint positions for "opened" and "closed" states.
        - ```action_deltas```: Small movements applied to the gripper joints to perform opening/closing actions.
    - ```SingleManipulator```: Wraps the robot into a SingleManipulator object:
        - ```prim_path```: Path in the simulation for the robot.
        - ```name```: Name of the manipulator.
        - ```end_effector_prim_name```: Specifies the end-effector (xarm6link_eef).
        - ```gripper```: Associates the gripper defined earlier.
    - ```SingleManipulator.set_joints_default_state()```:Initializes all joints to 0.0.
Sets specific joints 7 and 8, gripper joints to 0.628
> [!Note] **Documentation :**  
>- [tasks.PickPlace](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=pickplace#pick-and-place)
>- [add_reference_to_stage](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=add_reference_to_stage#module-omni.isaac.core.utils.stage)
>- [ParallelGripper](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.manipulators/docs/index.html?highlight=parallelgripper#parallel-gripper)
>- [SingleManipulator](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.manipulators/docs/index.html?highlight=singlemanipulator#module-omni.isaac.manipulators.manipulators.SingleManipulator)  

```python
class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "denso_pick_place",
        cube_initial_position: Optional[np.ndarray] = [0.3,0.3,0.3],
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
            cube_size=np.array([0.0515, 0.0515, 0.1]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change the asset path here
        #absolute path needed
        asset_path = "/home/theobloesch/xArm6_Pick_Place/with_rmpflow/xarm6_cfg_files/xarm6/xarm6.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/xarm6")
        
        gripper = ParallelGripper(
            end_effector_prim_path="/World/xarm6/root_joint/xarm6link_tcp",
            joint_prim_names=["xarm6drive_joint", "xarm6right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.2, 0.2]) )
        manipulator = SingleManipulator(prim_path="/World/xarm6",
                                        name="xarm6",
                                        end_effector_prim_name="xarm6link_tcp",
                                        gripper=gripper)
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
```

### class PickPlaceController(manipulators_controllers.PickPlaceController)

```class PickPlace(tasks.PickPlace)```

```python
class PickPlaceController(manipulators_controllers.PickPlaceController):

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: Articulation,
        events_dt=None
    ) -> None:

        if events_dt is None:
            #These values needs to be tuned in general, you checkout each event in execution and slow it down or speed
            #it up depends on how smooth the movments are
            events_dt = [0.005, 0.002, 1, 0.05, 0.0008, 0.005, 0.0008, 0.1, 0.0008, 0.008]
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            events_dt=events_dt,
            #This value can be changed
            # start_picking_height=0.6
            end_effector_initial_height=0.6,
        )
        return
```

### RMPFlowController(mg.MotionPolicycontoller)
```python
class RMPFlowController(mg.MotionPolicyController):

    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # TODO: change the follow paths
        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path="rmpflow/robot_descriptor.yaml",
                                                        rmpflow_config_path="rmpflow/denso_rmpflow_common.yaml",
                                                        urdf_path="/home/theobloesch/xArm6_Pick_Place/with_rmpflow/xarm6_cfg_files/xarm6.urdf",
,
                                                        end_effector_frame_name="xarm6link_eef",
                                                        maximum_substep_size=0.00334)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)
        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return
```

### Main program

First we have to create a World with : 
```python
my_world = World(stage_units_in_meters=1.0)
```
> [!Note]
> **Documentation :** [World](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.world)

Then we can define our task with its name and the target_position :
```python

target_position = np.array([0.3, 0.3, 0])
target_position[2] = 0.0515 / 2.0

my_task = PickPlace(name="denso_pick_place", target_position=target_position)

```
the target potition define here is the position where the cube is going to be placed 

```python

my_world = World(stage_units_in_meters=1.0)

target_position = np.array([0.3, 0.3, 0])
target_position[2] = 0.0515 / 2.0

my_task = PickPlace(name="denso_pick_place", target_position=target_position)
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("denso_pick_place").get_params()
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)

#initialize the controller
my_controller = PickPlaceController(name="controller", robot_articulation=my_denso, gripper=my_denso.gripper)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()

while simulation_app.is_running():

    my_world.step(render=True)
    
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()

        #forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # This offset needs tuning as well
            end_effector_offset=np.array([0, 0, 0.15]),
        )

        if my_controller.is_done():
            print("done picking and placing")

        articulation_controller.apply_action(actions) 

simulation_app.close()

``` 





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