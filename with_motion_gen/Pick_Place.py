#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
try:
    # Third Party
    import isaacsim
except ImportError:
    pass


# Third Party
import time
import torch
torch.cuda.empty_cache()
a = torch.zeros(
    4, device="cuda:0"
)  # this is necessary to allow isaac sim to use this torch instance
# Third Party
import numpy as np

np.set_printoptions(suppress=True)
# Standard Library

# Standard Library
import argparse

## import curobo:

parser = argparse.ArgumentParser()

parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)

parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)

parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)

args = parser.parse_args()

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)
# Standard Library
from typing import Optional

# Third Party
import carb
from helper import add_extensions
from omni.isaac.core import World
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.tasks import Stacking as BaseStacking
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.objects import cuboid, sphere
from omni.isaac.core.tasks import BaseTask
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_euler_angles
from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid, FixedCuboid



# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.sphere_fit import SphereFitType
from curobo.geom.types import WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenResult,
    PoseCostMetric,
)
from curobo.geom.types import Cuboid
from helper import add_extensions, add_robot_to_scene
import numpy as np

from xarm.wrapper import XArmAPI

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
        self.pose_list = []
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
        self.my_task.add_cube_to_pick(self.my_world)   
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
        self.cmd_idx = 0
        self.pose_list = []

    def motion_constraint(self):
        # add constraints to the motion gen here : linear movement along z axis of the end effector
        self.pose_metric = PoseCostMetric.create_grasp_approach_metric(offset_position=0.12,tstep_fraction=0.6, linear_axis=2)
        
    def config_motion_gen(self):
        
        self.cmd_plan = None
        
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 30
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
            num_trajopt_seeds=10,
            num_graph_seeds=10,
            interpolation_dt=interpolation_dt,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        print("warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=True)
        self.pose_metric = None
        if self.constraint_approach == True :
            print("Contraint approach")
            self.motion_constraint()
        self.plan_config = MotionGenPlanConfig(
            enable_graph=True,
            need_graph_success=True,
            max_attempts=max_attempts,
            enable_graph_attempt=10,
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
                    "/World/defaultGroundPlane",
                    #"/World/random_cube",
                    "/curobo",
                    "/World/table",
                    "/World/obstacles/table",
                    "/World/obstacles/table_mesh",
                    
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
                    "/World/obstacles/table",
                    "/World/obstacles/table_mesh",
                    
                ],
            ).get_collision_check_world()
            print("Obstacles read from stage",len(obstacles.objects))

            self.motion_gen.update_world(obstacles)
            print("Updated World")
            carb.log_info("Synced CuRobo world from stage.")
            
    def update_world_obstacles_after_rising(self):
        print("Updating world, reading w.r.t.", self.robot_prim_path)
        obstacles = self.usd_help.get_obstacles_from_stage(
            # only_paths="obstacles",
            reference_prim_path=self.robot_prim_path,
            ignore_substring=[
                self.robot_prim_path,
                #"/World/target",
                #"/World/defaultGroundPlane",
                "/World/random_cube",
                #"/curobo",
                #"/World/table",
                #"/World/obstacles/table",
                #"/World/obstacles/table_mesh",
                
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
                self.pose_list = cmd_plan.position.tolist()
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
            world_objects_pose_offset=Pose.from_list([0, 0, -0.17, 1, 0, 0, 0], self.tensor_args),
            remove_obstacles_from_world_config = True,
            surface_sphere_radius = 0.005
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
        
    
####Attention update collision checking with the cube before picking

class CuroboPickPlaceTasks(BaseTask):
    def __init__(self, name:str, offset: Optional[np.ndarray] = None):
        super().__init__(name, offset=None)
        self.offset = offset
        return
     
    def add_cube_to_pick(self,world):
        
        self.target_height = 0.07
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





def main():
    arm = None
    try:
        arm = XArmAPI("192.168.1.215", is_radian=True)
        arm.motion_enable(enable=True)
        arm.set_gripper_enable(1)
        arm.set_mode(6)
        arm.set_state(state=0)
    except Exception as e:
        print(e)
        print("Failed to connect to xArm")
    
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
            curobo.detach_object()
            curobo.open_gripper()
            curobo.reset()
            art_action = None
            position = curobotask.cube_position
            orientation = curobotask.cube_orientation
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
            curobo.update_world_obstacles_after_rising()
            print("Target 2 reached")
            position = [0.43, 0.26, 0.41]
            orientation = [0, 1, 0, 0.0]
        result_3 = curobo.is_target_reached(goal_position=[0.43, 0.26, 0.41], goal_orientation=[0, 1, 0, 0.0])
        if result_3==True:
            result_3=False
            curobo.update_world_obstacles_after_rising()
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
            curobo.update_world_obstacles_after_rising()
            position = [0.43, 0.26, 0.41]
            orientation = [0, 1, 0, 0.0]
            #target.set_world_pose(position=position, orientation=orientation)
            
        art_action = curobo.forward(goal_position=position, goal_orientation=orientation)
        check = False
        if art_action is not None:
            curobo.articulation_controller.apply_action(art_action)  
            check = True
        # if arm is not None and check:
        #     if len(curobo.pose_list) > 0:
        #         speed=10
        #         i=0
        #         #while i < len(curobo.pose_list):
        #         for i in range(len(curobo.pose_list)):
        #             print("Positon numero : ",i)
        #             arm.set_servo_angle(angle=curobo.pose_list[i],speed=speed,wait=False) 
        #             time.sleep(0.08)
        #             i+=1
        #         curobo.pose_list = []
    
    # #################################Uncomment to show spheres##############################        
    #     sim_js = curobo.robot.get_joints_state()    
    #     sim_js_names = curobo.robot.dof_names
    #     cu_js = JointState(
    #     position=curobo.tensor_args.to_device(sim_js.positions),
    #     velocity=curobo.tensor_args.to_device(sim_js.velocities),  # * 0.0,
    #     acceleration=curobo.tensor_args.to_device(sim_js.velocities) * 0.0,
    #     jerk=curobo.tensor_args.to_device(sim_js.velocities) * 0.0,
    #     joint_names=sim_js_names,
    #     )

    #     cu_js.velocity *= 0.0
    #     cu_js.acceleration *= 0.0

    #     cu_js = cu_js.get_ordered_joint_state(curobo.motion_gen.kinematics.joint_names)
    #     visualize_sphere(curobo.motion_gen, cu_js, spheres=None)    
    # #################################Uncomment to show spheres##############################       
    simulation_app.close()
    curobo.my_world.close()
    
             
        
if __name__ == "__main__":
    main()