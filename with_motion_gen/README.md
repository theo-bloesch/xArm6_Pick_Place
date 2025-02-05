## Motion generation with Curobo

First we are going to explain the one of the Curobo example to generate a collision fre trajectory.

[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)

- **```world_config```**: This dictionary defines the environment configuration for the simulation or planning  
    - ```mesh``` : Specifies a mesh file for the base scene and its pose in the world
        - ```pose``` : The position (x, y, z) and orientation (qw, qx, qy, qz) of the scene's mesh.
        - ```file_path```: The file path for the 3D scene model
    - ```cuboid```: Describes a cuboid object (e.g., a table) in the environment
        - ```dims``` : The cuboid's dimensions (x, y, z).
        - ```pose``` : The pose of the cuboid in the environment.
- **```motion_gen_config = MotionGenConfig.load_from_robot_config()```**: Loads motion generation configuration from a robot-specific YAML.
    - ```"ur5e.yml"``` : Path to the robot's configuration YAML file. 
    - ```world_config``` : The environment configuration dictionary 
    - ```interpolation_dt=0.01``` : Specifies the time interval for interpolating the motion plan.  

    - **```Return type```**: 
        - ```MotionGenConfig``` : The loaded motion generation configuration object. 

- ```motion_gen = MotionGen(motion_gen_config)```: Initializes the motion generator using a 
```MotionGenConfig``` object.
  - ```motion_gen_config``` : The motion generation configuration loaded earlier.
  - **```Return type```**:  
    - ```MotionGen``` : The motion generator instance.
- **```motion_gen.warmup()```**: Prepares the motion generator for motion planning by initializing internal data structures.
    - **```Return type```**:
        - ```None``` : This method modifies internal state but doesn't return a value.
- **```goal_pose = Pose.from_list()```**: Creates a Pose object from a list of 7 elements representing position and orientation in quaternion format.
    - **```Return type```**:
        - ```Pose``` : A pose object encapsulating the position and orientation.
- **```start_state = JointState.from_position```**: Creates a JointState object with specified joint positions and joint names.
    - ```torch.zeros(1, 6).cuda()```: A tensor representing the initial joint positions (1×6 matrix of zeros, moved to the GPU).
    - ```joint_names``` : A list of strings containing joint names.
    - **```Return type```**:
        - ```JointState``` : An object containing the joint positions and names.
- **```result = motion_gen.plan_single```**: Plans a motion from the given start state to the goal pose
    - ```start_state```: A JointState object representing the initial joint configuration.
    - ```goal_pose```: A Pose object representing the target pose.
    - ```MotionGenPlanConfig(max_attempts=1)```: Configuration for the motion planning (e.g., number of attempts).
    - **```Return type```**:
        - ```MotionGenPlanResult``` : An object containing the result of the motion planning, including **success** status and **trajectory**.
- **```traj = result.get_interpolated_plan()```**:  Retrieves the interpolated trajectory from the planning result.
    - **```Return type```**:
        - A trajectory object (specific data type depends on cuRobo). Likely a time-indexed sequence of joint states.
- **```result.success```**: Indicates whether the motion planning was successful.
    - **```Return type```**:
        - ```bool``` : Returns True if the planning was successful, otherwise False.

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

> [!Note]  
> **Documentations :**  
> [MotionGenConfig](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenConfig)  
> [MotionGenConfig.load_from_robot_config()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenConfig.load_from_robot_config)  
> [MotionGen](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen)
> [MotionGen.plan_single()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen.plan_single)  
> [MotionGenResult](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult)  
> [MotionGenResult.get_interpolated_plan()](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenResult.get_interpolated_plan)  
> [Pose.from_list()](https://curobo.org/_api/curobo.types.math.html#curobo.types.math.Pose.from_list)  
> [JointState.from_position()](https://curobo.org/_api/curobo.types.state.html#curobo.types.state.JointState.from_position)

## Example with ufactory xarm6 robot (OOP : Based on the [simple stacking example](with_motion_gen/Exemple_curobo/simple_stacking.py))


For this example I created two classes :

1. ```class CuroboController(BaseController)```
    
    Responsible to initialize the scene with the robot its gripper and the scene and object needed for the task. It initialize also the configuratoin for the motion planning.
2. ```class CuroboPickPlaceTasks(BaseTask)```

    Responsible to initialize all the object and method to perform the task.


## class CuroboController(BaseController)

- ```def __init__()```:
    - ```my_task```: A task object (likely defines the pick-and-place task and objects involved).
    - ```name``` : Name of the controller.
    - ```usd_help```: Helper for managing USD files in the simulation.
    - ```constrain_grasp_approach``` : If True, constrains the robot's grasping motion.

- ```def setup_scene()``` : 
    - This method sets up the simulation scene by adding:
        1.  **Robot** : Loads a robot from a USD file (xarm6.usd).Configures gripper parameters and initializes a SingleManipulator object.
        2. **Scene and Obstacles**: Loads a scene configuration (scene2.yml) that defines obstacles (cuboids and meshes). Adds these obstacles to the simulation world as fixed objects.
        3. **Ground Plane**: Adds a default ground plane for the simulation.

        4. **World Configuration**: Uses CuRobo's WorldConfig to manage the robot's environment, including meshes and cuboids.

    - ```World(stage_units_in_meters=1.0)``` : Creates the simulation environment.   -
        - ```stage_units_in_meters=1.0``` ensures the scene uses meters as the unit of measurement. ```stage```Refers to the USD (Universal Scene Description) stage that represents the 3D simulation world.
    - ```add_reference_to_stage(usd_path=asset_path, prim_path="/World/UF_ROBOT")```: Add USD reference to the opened stage at specified prim path.
        - ```asset_path```: Specifies the file path to the USD file containing the robot's model (e.g., xArm6).
        - ```prim_path```: Loads the robot model from the USD file into the simulation at the path ```/World/UF_ROBOT```.

    - ```load_yaml()``` : Load the robot specification from a yaml file. In this case we load the robot configuration to use the motion generation from curobo and the world configuration.
        - ``` self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]``` : Store the joints name contained in the configuration file
        - ``` self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]``` : Save the "home" position contained in the configuration file.
    - ```WorldConfig()```: Configures the robot's world (including collision objects) use in Curobo. 
    - ```WorldConfig.from_dict()```: Creates a world configuration using a Python dictionary use in Curobo.
    - ```ParallelGripper```: Provides high level functions to set/ get properties and actions of a parllel gripper (a gripper that has two fingers).
    - ```World.scene.add()``` : ```World``` includes an instance of PhysicsContext which takes care of many physics related settings such as setting physics dt, solver type..etc. In addition to what is provided from SimulationContext, this class allows the user to add a task to the world and it contains a scene object. To control the default reset state of different objects easily, the object could be added to a Scene. Besides this, the object is bound to a short keyword that facilitates objects retrievals, like in a dict.
        - ```scene()``` : Provide methods to add objects of interest in the stage to retrieve their information and set their reset default state in an easy way
            - ```add()```: Add an object to the scene registry
    - ```SingleManipulator()``` : Provides high level functions to set/ get properties and actions of a manipulator with a single end effector and optionally a gripper.
    - ```World.scene.add_default_ground_plane()```: Create a ground plane (using the default asset for Isaac Sim environments) and add it to the scene registry
    - ```USD_Helper.load_stage()``` : Curobo function to load the stage into **??????????**
    - ```USD_Helper.add_world_to_stage()``` : Curobo function **??????????**

- ```def set_first_pose(self)```:
    - ```self.my_world.reset()```: Reset the stage to its initial state and each object included in the Scene to its default state 
    - ```self.robot._articulation_view.initialize()```
    - ```self.articulation_controller = self.robot.get_articulation_controller()```
    - ```self.idx_list = [self.robot.get_dof_index(x) for x in self.j_names]```
    - ```self.robot.set_joint_positions(self.default_config, self.idx_list)```
    - ```self.robot._articulation_view.set_max_efforts(values=np.array([5000 for i in range(len(self idx_list))]), joint_indices=self.idx_list)```

- ```def reset(self):```

- ```def config_motion_gen()```

    - ```TensorDeviceType()```
    - ```MotionGenConfig.load_from_robot_config()```
    - ```MotionGen()```
    - ```MotionGenPlanConfig()```

- ```def update_world_obstacles_before_taking()```

- ```def update_world_obstacles_after_taking()```

- ```def plan()```

    - ```Pose()```
    - ```JointState()```
    - ```result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())```

- ```def forward()```
    - ```ArticulationAction()```
    - ```curobo.articulation_controller.apply_action(art_action)```

- ```def get_current_eef_position(self):```

- ```is_target_reached```:

- ```def attach_object(self,cube_name):```

- ```def detach_object(self)```:

- ```def close_gripper(self)```:

- ```def open_gripper(self)```:

> [!Note]  
> **Documentations :**                  
> - [World()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#module-omni.isaac.core.world)
>   - [scene()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#scene)
>       - [add()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.scenes.Scene.add)   
>       -  [add_default_ground_plane()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world#omni.isaac.core.scenes.Scene.add_default_ground_plane)  
>   - [reset()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=world%20reset#omni.isaac.core.world.World.reset)
> 
> [add_reference_to_stage()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=add_reference_to_stage#omni.isaac.core.utils.stage.add_reference_to_stage)  
> [load_yaml]() 
> [WorldConfig](https://curobo.org/_api/curobo.geom.types.html#curobo.geom.types.WorldConfig) 
> [ParallelGripper()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.manipulators/docs/index.html?highlight=parallelgripper#parallel-gripper)   
> [SingleManipulator()](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.manipulators/docs/index.html?highlight=singlemanipulator#module-omni.isaac.manipulators.manipulators.SingleManipulator)  
> [MotionGenPlanconfig](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGenPlanConfig)  
> [USD_Helper](https://curobo.org/_api/curobo.util.usd_helper.html#curobo.util.usd_helper.UsdHelper) 





```python
class CuroboController(BaseController):
    def __init__(
        self,
        my_task: BaseTask,
        name: str = "curobo_controller",
        constrain_grasp_approach: bool = False,
    ) -> None:
        BaseController.__init__(self, name=name)
        self.my_task = my_task
        self.usd_help = UsdHelper()
        self.constraint_approach = constrain_grasp_approach
        
    def setup_scene(self):
        # Create a world with the robot and the object
        self.my_world= World(stage_units_in_meters=1.0)
        self.stage = self.my_world.stage
        
        asset_path = "/home/theobloesch/xArm6_Pick_Place/with_rmpflow/xarm6_cfg_files/xarm6/xarm6.usd"

        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UF_ROBOT")
        self.robot_prim_path = "/World/UF_ROBOT"
        
        ## Robot ##
        self.robot_cfg = load_yaml("xArm6Curobo/xArm6Curobo2.yaml")["robot_cfg"]
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]
        
        self.gripper = ParallelGripper(
            #We chose the following values while inspecting the articulation
            end_effector_prim_path="/World/UF_ROBOT/root_joint/xarm6link_tcp",
            joint_prim_names=["xarm6drive_joint", "xarm6right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.628, 0.628]),
        )
        #define the manipulator
        self.robot = self.my_world.scene.add(SingleManipulator(prim_path="/World/UF_ROBOT", name="UF_ROBOT",

                                                        end_effector_prim_name="xarm6link_tcp", gripper=self.gripper))
        #set the default positions of the other gripper joints to be opened so
        #that its out of the way of the joints we want to control when gripping an object for instance.
        ## Scene  config ##
        world_cfg_scene = WorldConfig.from_dict(
            load_yaml("scene2.yml")
        )
         # Ajouter chaque cuboid défini dans le fichier YAML à la scène
        for cuboid in world_cfg_scene.cuboid:
            # Extraire les propriétés du cuboid
            position = np.array(cuboid.pose[:3])  # x, y, z
            orientation = np.array(cuboid.pose[3:])  # qx, qy, qz, qw
            scale = np.array(cuboid.dims)  # Dimensions (largeur, hauteur, profondeur)

            # Créer et ajouter le FixedCuboid à la scène
            self.my_world.scene.add(
                FixedCuboid(
                    prim_path=f"/World/{cuboid.name}",  # Chemin unique dans la scène
                    name=cuboid.name,  # Nom de l'objet
                    position=position,  # Position initiale
                    orientation=[1,0,0,0],  # Orientation initiale
                    scale=scale,  # Dimensions
                )
            )

        world_cfg1 = WorldConfig.from_dict(
            load_yaml("scene2.yml")
        ).get_mesh_world()
        
        for i in range(len(world_cfg1.mesh)):
            world_cfg1.mesh[i].name += "_mesh"
        

        self.world_cfg = WorldConfig(cuboid=world_cfg_scene.cuboid, mesh=world_cfg1.mesh)
        self.my_world.scene.add_default_ground_plane()
        self.my_task.add_objet_to_pick(self.my_world)   
        self.usd_help.load_stage(self.my_world.stage)
        self.usd_help.add_world_to_stage(self.world_cfg, base_frame="/World")
            
  
        
    def set_first_pose(self):
        #set firt pose
        self.my_world.reset()
        self.robot._articulation_view.initialize()
        self.articulation_controller = self.robot.get_articulation_controller()
        self.idx_list = [self.robot.get_dof_index(x) for x in self.j_names]
        self.robot.set_joint_positions(self.default_config, self.idx_list)
        self.robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(self.idx_list))]), joint_indices=self.idx_list
            )
        
    def reset(self):
        self.my_world.reset()
        self.robot._articulation_view.initialize()
        self.articulation_controller = self.robot.get_articulation_controller()
        self.idx_list = [self.robot.get_dof_index(x) for x in self.j_names]
        self.robot.set_joint_positions(self.default_config, self.idx_list)
        self.robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(self.idx_list))]), joint_indices=self.idx_list
            )

    def motion_constraint(self):
        # add constraints to the motion gen here : linear movement along z axis of the end effector
        self.pose_metric = PoseCostMetric.create_grasp_approach_metric(offset_position=0.12,tstep_fraction=0.6, linear_axis=2)
        
    def config_motion_gen(self):
        
        self.cmd_plan = None
        
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 100
        trajopt_dt = None
        optimize_dt = True
        trajopt_tsteps = 32
        trim_steps = None
        max_attempts = 50
        interpolation_dt = 0.05
        
        self.tensor_args = TensorDeviceType()
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            interpolation_dt=interpolation_dt,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        print("warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
        self.pose_metric = None
        if self.constraint_approach == True :
            print("Contraint approach")
            self.motion_constraint()
        self.plan_config = MotionGenPlanConfig(
            enable_graph=True,
            need_graph_success=True,
            max_attempts=max_attempts,
            enable_graph_attempt=5,
            enable_finetune_trajopt=True,
            partial_ik_opt=False,
            parallel_finetune=True,
            pose_cost_metric=self.pose_metric,
            time_dilation_factor=0.5,
        )
        
        self.motion_gen_result = MotionGenResult()

        print("Curobo is Ready")
        
    def update_world_obstacles_before_taking(self):
            print("Updating world, reading w.r.t.", self.robot_prim_path)
            obstacles = self.usd_help.get_obstacles_from_stage(
                # only_paths="obstacles",
                reference_prim_path=self.robot_prim_path,
                ignore_substring=[
                    self.robot_prim_path,
                    #"/World/target",
                    #"/World/defaultGroundPlane",
                    #"/World/random_cube",
                    #"/curobo",
                    
                    
                ],
            ).get_collision_check_world()
            print("Obstacles read from stage",len(obstacles.objects))

            self.motion_gen.update_world(obstacles)
            print("Updated World")
            carb.log_info("Synced CuRobo world from stage.")
    def update_world_obstacles_after_taking(self):
            print("Updating world, reading w.r.t.", self.robot_prim_path)
            obstacles = self.usd_help.get_obstacles_from_stage(
                # only_paths="obstacles",
                reference_prim_path=self.robot_prim_path,
                ignore_substring=[
                    self.robot_prim_path,
                    #"/World/target",
                    "/World/defaultGroundPlane",
                    "/World/random_cube",
                    "/curobo",
                    "/World/table",
                    "/World/obstacle/table",
                    "/World/obstacles/table_mesh",
                    
                ],
            ).get_collision_check_world()
            print("Obstacles read from stage",len(obstacles.objects))

            self.motion_gen.update_world(obstacles)
            print("Updated World")
            carb.log_info("Synced CuRobo world from stage.")
        
    def plan(self,goal_position, goal_orientation):
        # Generate a plan to reach the goal
        ik_goal = Pose(
            position=self.tensor_args.to_device(goal_position),
            quaternion=self.tensor_args.to_device(goal_orientation),
        )
        sim_js = self.robot.get_joints_state()
        #sim_js_names = self.robot.dof_names
        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=self.j_names,
        )
        cu_js = cu_js.get_ordered_joint_state(self.motion_gen.kinematics.joint_names)
        result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())
        return result
        
    def forward(self,goal_position, goal_orientation):
        if self.cmd_plan is None:
            self.cmd_idx = 0
            self._step_idx = 0
            # Execute the plan
            result = self.plan(goal_position, goal_orientation)            
            succ = result.success.item()
            if succ:
                print("Plan converged to a solution.")
                cmd_plan = result.get_interpolated_plan()
                # print(cmd_plan)
                self.idx_list = [i for i in range(len(self.j_names))]
                self.cmd_plan = cmd_plan.get_ordered_joint_state(self.j_names)
            else:
                carb.log_warn("Plan did not converge to a solution.")
                return None
        
        if self._step_idx % 3 == 0 :
            cmd_state = self.cmd_plan[self.cmd_idx]
            self.cmd_idx += 1

            # get full dof state
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                cmd_state.velocity.cpu().numpy() * 0.0,
                joint_indices=self.idx_list,
            )
            if self.cmd_idx >= len(self.cmd_plan.position):
                self.cmd_idx = 0
                self.cmd_plan = None
        else:
            art_action = None
        self._step_idx += 1
        return art_action
    
    def get_current_eef_position(self):
        self.current_eef_position = self.robot.end_effector.get_world_pose()[0]
        self.current_eef_orientation = self.robot.end_effector.get_world_pose()[1]
        
    
    def is_target_reached(self, goal_position, goal_orientation)->bool:
        # Check if the target has been reached
        self.get_current_eef_position()
        if ((np.linalg.norm(goal_position - self.current_eef_position) < 0.02)and ((2*np.arccos(np.abs(np.dot(goal_orientation, self.current_eef_orientation))))< 0.02)):
            return True
        else:
            return False

    def attach_object(self,cube_name):
        # Attach the object to the robot and add the object to the motion generator
        sim_js = self.robot.get_joints_state()
        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=self.j_names,
        )
        self.motion_gen.attach_objects_to_robot(
            cu_js,
            [cube_name],
            link_name="attached_object",
            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
            world_objects_pose_offset=Pose.from_list([0, 0, -0.18, 1, 0, 0, 0], self.tensor_args),
            remove_obstacles_from_world_config = True,
            surface_sphere_radius = 0.01
        )
    
    def detach_object(self):
        # Detach the object from the robot and remove the object from the motion generator
        self.motion_gen.detach_object_from_robot()
        
    def close_gripper(self):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] < 0.628 :
            print(gripper_positions)
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] + 0.628, gripper_positions[1] - 0.628]))
            self.my_world.step(render=True)
    def open_gripper(self):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] > 1e-3 :
            print(gripper_positions)
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] - 0.628, gripper_positions[1] + 0.628]))
            self.my_world.step(render=True)
```

## class CuroboPickPlaceTasks(BaseTask):
- ```def __init__()```:
- ```def add_cube_to_pick()```:
- ```def get_cube_pos_orient(self):```:
- ```def set_cube_pos_orient(self,goal_position,goal_orientation):```:
- ```def get_observations(self,robot):```:

```python
class CuroboPickPlaceTasks(BaseTask):
    def __init__(self, name:str, offset: Optional[np.ndarray] = None):
        super().__init__(name, offset=None)
        self.offset = offset
        return
     
    def add_cube_to_pick(self,world):
        
        self.target_height = 0.05
        self.target_width = 0.05
        self.target_depth = 0.05
        
        self.random_cube = world.scene.add(  
        DynamicCuboid(
            prim_path="/World/random_cube",
            name="random_cube",
            position=np.array([0.4, -0.2, self.target_height/2.0]),
            scale=np.array([self.target_depth, self.target_width, self.target_height]),
            color=np.array([0, 1.0, 1.1]),
            orientation=np.array(euler_angles_to_quats([3.1415927 ,0,0 ])),
        ))
        self.cube_position = self.random_cube.get_world_pose()[0]
        self.cube_position[2] = self.target_height-0.030
        self.cube_orientation = self.random_cube.get_world_pose()[1]
        
        
    def get_cube_pos_orient(self):
        self.cube_position = self.random_cube.get_world_pose()[0]
        self.cube_position[2] = self.target_height-0.030
        self.cube_orientation = self.random_cube.get_world_pose()[1]
        
    def set_cube_pos_orient(self,goal_position,goal_orientation):
        self.random_cube.set_world_pose(goal_position,goal_orientation)
       
    #comprendre comment fonctionne observations
    def get_observations(self,robot):

        current_joint_positions = robot.get_joint_positions()
        observations = {
            robot.name: {
                "joint_positions": current_joint_positions,
            },         
        }
        return observations
    
    
    def get_params(self):
        return {
            "goal_position": self._goal_position
        }
```

## def visualize_sphere(motion_gen, cu_js, spheres)
```python
def visualize_sphere(motion_gen, cu_js, spheres=None):
    #########Affichage des spheres#################
    sph_list = motion_gen.kinematics.get_robot_as_spheres(cu_js.position)
    if spheres is None:
        spheres = []
        # create spheres:
        for si, s in enumerate(sph_list[0]):
            sp = sphere.VisualSphere(
                prim_path="/curobo/robot_sphere_" + str(si),
                position=np.ravel(s.position),
                radius=float(s.radius),
                color=np.array([0, 0.8, 0.2]),
            )
            spheres.append(sp)
    else:
        for si, s in enumerate(sph_list[0]):
            if not np.isnan(s.position[0]):
                spheres[si].set_world_pose(position=np.ravel(s.position))
                spheres[si].set_radius(float(s.radius))
    #########Affichage des spheres#################
```

## Main program




### Main loop



```python
def main():
    
    curobotask=CuroboPickPlaceTasks(name="pickplace")
    curobo = CuroboController(my_task=curobotask,constrain_grasp_approach=False)

    curobo.setup_scene()
    curobo.config_motion_gen()
    set_camera_view(eye=[1, -2, 1], target=[0.00, 0.00, 0.00], camera_prim_path="/OmniverseKit_Persp")
    art_action = None
    task_step=0
    result = False
    curobo.reset()
    curobo.update_world_obstacles_before_taking()
    
    position = curobotask.cube_position
    orientation = curobotask.cube_orientation
    curobo.open_gripper() 
    while simulation_app.is_running():
        task_step+=1
        curobo.my_world.step(render=True)    
        if not curobo.my_world.is_playing():
            curobo.set_first_pose()
            curobo.update_world_obstacles_before_taking()
            print("Robot type", type(curobo.robot))   
            continue
        
        curobotask.get_cube_pos_orient() 
        
        if task_step < 50:
            continue
               
        curobo.get_current_eef_position()
        # Attention don't let the orientation of the cube when it's taken with the gripper
        result_1 = curobo.is_target_reached(goal_position=curobotask.cube_position, goal_orientation=curobotask.cube_orientation)
        print("Result_1",result_1)
        if result_1==True:
            result_1=False
            print("Target reached******************************************************************************************")
            curobo.close_gripper()
            print("closed gripper")
            curobo.update_world_obstacles_before_taking()
            curobo.attach_object(cube_name="/World/random_cube")
            position = (curobotask.cube_position + np.array([0.0, 0.0, 0.35]))
            orientation = euler_angles_to_quats([3.1415927 ,0,0 ])
            print("Euler angle to quats :", euler_angles_to_quats([3.1415927 ,0,0 ]))
            curobo.update_world_obstacles_after_taking()
        result_2 = curobo.is_target_reached(goal_position=(curobotask.cube_position + np.array([0.0, 0.0, 0.3500])), goal_orientation=euler_angles_to_quats([3.1415927 ,0,0 ]))
        if result_2==True:
            result_2=False
            curobo.update_world_obstacles_before_taking()
            print("Target 2 reached")
            position = [0.43, 0.26, 0.43]
            orientation = [0, 1, 0, 0.0]
        result_3 = curobo.is_target_reached(goal_position=[0.43, 0.26, 0.43], goal_orientation=[0, 1, 0, 0.0])
        if result_3==True:
            result_3=False
            print("Target 3 reached")
            #curobo.open_gripper()
            position = [0.40, 0.40, 0.25]
            orientation = [0, 0.7071, 0, 0.7071]
            #target.set_world_pose(position=position, orientation=orientation)
        result_4 = curobo.is_target_reached(goal_position=[0.40, 0.40, 0.25], goal_orientation=[0, 0.7071, 0, 0.7071])
        
        if result_4==True:
            result_4=False
            print("Target 4 reached")
            #curobo.close_gripper()
            print("closed gripper")
            position = [0.43, 0.26, 0.43]
            orientation = [0, 1, 0, 0.0]
            #target.set_world_pose(position=position, orientation=orientation)
            
        art_action = curobo.forward(goal_position=position, goal_orientation=orientation)
        if art_action is not None:
            curobo.articulation_controller.apply_action(art_action)      
            
        sim_js = curobo.robot.get_joints_state()    
        sim_js_names = curobo.robot.dof_names
        cu_js = JointState(
        position=curobo.tensor_args.to_device(sim_js.positions),
        velocity=curobo.tensor_args.to_device(sim_js.velocities),  # * 0.0,
        acceleration=curobo.tensor_args.to_device(sim_js.velocities) * 0.0,
        jerk=curobo.tensor_args.to_device(sim_js.velocities) * 0.0,
        joint_names=sim_js_names,
        )

        cu_js.velocity *= 0.0
        cu_js.acceleration *= 0.0

        cu_js = cu_js.get_ordered_joint_state(curobo.motion_gen.kinematics.joint_names)
        visualize_sphere(curobo.motion_gen, cu_js, spheres=None)         
    simulation_app.close()
```
## Results

[![Watch the video](https://github.com/theo-bloesch/xArm6_Pick_Place/blob/main/img/ExamplePickPlace.png)](https://github.com/theo-bloesch/xArm6_Pick_Place/blob/main/img/ExamplePickPlace.mp4)



Some improvement need to be made 