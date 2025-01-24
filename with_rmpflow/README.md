# Simulation with IsaacSim and path generation with rmpflow

> [!Note]
> **Documentation :**  
[Official IsaacSim tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)

Once this modification has been made we can begin to simulate our robot. For that we need several file or code part. Here we decided to make all the code in two file but in the Isaac sim exemple the split the code in several file.

We are going to make a simple pick and place task to verify that our robot works correctly. You can find the entire code [here](with_rmpflow/pick_place_test.py).

First we need to define our robot description to pass it to the rmpflow motion generator.
For that we create a folder rmp and inside it a file named ```robot_descriptor.yaml```. Make sure that the joint are the same that the one define in your **.usd**

## rmp/robot_descriptor.yaml

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

## rmp/denso_rmpflow_common.yaml

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

## class PickPlace(tasks.PickPlace)

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
    > - The ```asset_path``` is absolute, and the user must ensure the file exists at the specified location.
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
> [!Note]  
> **Documentation :**  
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

## class PickPlaceController(manipulators_controllers.PickPlaceController)

> [!Note]  
> **Documentation :** [PickPlaceController](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.manipulators/docs/index.html?highlight=pickplacecontroller#omni.isaac.manipulators.controllers.PickPlaceController)

```class PickPlace(tasks.PickPlace)```
- ```__init__()```
    - ```name```:
    - ```gripper```:
    - ```robot_articulation```:
    - ```event_dt```:
    - ```manipulators_controllers.PickPlaceController.__init__()```:
        - ...
        - ```cspace_controller```:
        - ```end_effector_initial_height```:

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

## RMPFlowController(mg.MotionPolicycontoller)

```class RMPFlowController(mg.MotionPolicyController)```
- ```__init__()```
    - ```name```:The name of the controller
    - ```gripper```: The gripper
    - ```robot_articulation```:Represents the articulated structure of the robot (joints and links).
    - ```physics_dt```: Time step for the simulation, defaulting to 1/60 seconds
    - ```mg.lula.motion_policies.RmpFlow```: Initializes the RMPFlow system, which generates smooth and compliant motions for the robot.
        - ```robot_description_path```: Path to a YAML file describing the robot's physical properties.
        - ```rmpflow_config_path```: Path to a configuration file containing parameters for RMPFlow, tailored for xArm6
        - ```urdf_path```: Absolute path to the robot's URDF (Unified Robot Description Format) file, which defines its structure, joints, and links.
        - ```end_effector_frame_name```: Specifies the name of the end-effector's frame (for motion planning and control).
        - ```maximum_substep_size```: Defines the maximum substep size for fine-grained motion planning.
        ...


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

## Main program

First we create the Simulation Environment with World : 
```python
my_world = World(stage_units_in_meters=1.0)
```
- ```World()``` represents the simulation environment.
- ```stage_units_in_meters=1.0``` means the units in the simulation are in meters.

> [!Note]
> **Documentation :** [World](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.world)

Then we can define our Pick-and-Place Task with its name and the target_position :
```python

target_position = np.array([0.3, 0.3, 0])
target_position[2] = 0.0515 / 2.0

my_task = PickPlace(name="denso_pick_place", target_position=target_position)
my_world.add_task(my_task)
my_world.reset()

```
- ```target_position``` : define here is the position where the cube is going to be placed.Be careful the reference prim for the cube position is in its center there is why the z-axis position can't be 0.
- ```my_task``` : PickPlace task is initialized with a name ("denso_pick_place") and the target position defined earlier.
- ```reset```: resets the simulation world to its initial state.
> [!Note]  
> - All tasks should be added before the first reset is called unless the clear method was called.
> - All articulations should be added before the first reset is called unless the clear method was called.
> - This method takes care of initializing articulation handles with the first reset called.
> - This will do one step internally regardless
> - Call post_reset on each object in the Scene
> - Call post_reset on each Task
> - Things like setting PD gains for instance should happen at a Task reset or a Robot reset since the defaults are restored after stop method is called.
> - Documentation : [World.reset()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world%20reset#omni.isaac.core.world.World.reset)



Then we can get the parameter of the tasks. This is defined differently for each task in order to access the task’s objects and values.
```python
task_params = my_world.get_task("denso_pick_place").get_params()
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)
```
- ```my_world.get_task("denso_pick_place").get_params()``` : task parameters are retrieved, which include names of elements like the robot and the cube.
- ```task_params["robot_name"]["value"]``` retrieves the name of the robot used in the task.
- ```my_world.scene.get_object(denso_name)``` returns the robot instance in the simulation.
> [!Note]
> Documentation : [BaseTask.get_params()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=pickplace#omni.isaac.core.tasks.BaseTask.get_params)

After we initialize the robot controller 

```python
#initialize the controller
my_controller = PickPlaceController(name="controller", robot_articulation=my_denso, gripper=my_denso.gripper)
#task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()
```
- ```PickPlaceController()```:  A PickPlaceController is created to handle the robot's actions. It is initialized with:
    - The robot (robot_articulation=my_denso).
    - The robot's gripper (gripper=my_denso.gripper).
- ```my_denso.get_articulation_controller()```: An articulation controller is retrieved from the robot to apply actions to its joints.

### While loop

```python    
while simulation_app.is_running():
    my_world.step(render=True)
        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                my_world.reset()
                my_controller.reset()
```
```simulation_app.is_running()```:  The main loop runs as long as the simulation app is running. (We have to close the window to shutdown IsaacSim) 
```my_world.step(render=True)```:  Advances the simulation by one step and renders the visuals
```my_world.is_playing()```:  If the simulation is started with the play button in IsaacSim GUI, it could also be stopped or paused.
```my_world.reset()```: If the simulation restarts (e.g., at the first timestep), the world and the controller are reset  
```my_controller.reset()```: If the simulation restarts (e.g., at the first timestep), the world and the controller are reset

> [!Note]  
> [World.get_observation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.world.World.get_observations)
```python    
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
```
- ```my_world.get_observations()```: The current state of the simulation is retrieved. This includes object positions, joint positions, and other data.
- ```my_controller.forward()```:  The controller calculates the actions required to:

    - Pick the object: Based on its current position (picking_position).
    - Place the object: Based on the target position (placing_position).
    - Account for the current joint positions (current_joint_positions).
    - Apply an offset for the end effector (end_effector_offset) to adjust the height.  
        
    It can get the pick and place position from :
    - ```observations[task_params["cube_name"]["value"]]["position"]```: observations is a dictionary that contains the current state of all objects in the simulation.  
    - It return the ```action``` that we have to pass to the ```.apply_action()``` methode of the ```articulation_controller```.
Each object (robot, cube, etc.) has its own entry, which provides data like positions, orientations, joint states, and more.
- ```my_controller.is_done()```:  The controller checks if the task is complete. If so, it prints a message.
- ```articulation_controller.apply_action(actions)```: The calculated actions are applied to the robot's joints via the articulation controller.



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
## Final result 

![Pick Place example with rmpflow motion generation](../img/PickPlaceRMP.gif)

The speed of the robot may be slower its normal.
