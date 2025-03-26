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

import os
#os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"

# Third Party
import time
import torch
torch.cuda.empty_cache()
a = torch.zeros(
    4, device="cuda:0"
)  # this is necessary to allow isaac sim to use this torch instance
# Third Party
print(torch.version.cuda)
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
from abc import ABC,abstractmethod
import gc


# Third Party
import carb
from helper import add_extensions
import omni
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
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from helper import add_extensions, add_robot_to_scene
import numpy as np

from xarm.wrapper import XArmAPI



class ManageCuroboScene(BaseTask):
    def __init__(
         self,
         robotAPI = None,
         ) -> None:
        self.usd_helper = UsdHelper()
        self.robotAPI = robotAPI
        self.world = None
        self.world_cfg = None
        self.setup_scene()
    
    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage
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
            self.world.scene.add(
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
        self.world_cfg = WorldConfig(cuboid=world_cfg_scene.cuboid)#, mesh=world_cfg1.mesh
        self.world.scene.add_default_ground_plane()
        self.usd_helper.load_stage(self.world.stage) #Curobo
        self.usd_helper.add_world_to_stage(self.world_cfg, base_frame="/World")#Curobo
        self.robotAPI.add_robot_to_scene(self.world)
        
    def setup_scene_2(self):
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage

        # S'assurer que la scène est bien initialisée
        if self.world.scene is None:
            raise RuntimeError("La scène de 'self.world' n'est pas initialisée.")

        # Ajouter un sol par défaut avant de charger l'USD
        self.world.scene.add_default_ground_plane()

        # Charger la scène USD directement
        usd_path = "/chemin/vers/ta_scene.usd"
        if omni.usd.get_context().open_stage(usd_path):
            print(f"Scène USD chargée : {usd_path}")
        else:
            raise RuntimeError(f"Erreur lors du chargement du fichier USD : {usd_path}")

        # Ajouter la scène USD au stage
        self.usd_helper.load_stage(self.world.stage)  # Charger le stage existant

        # Ajouter le monde configuré à la scène
        self.usd_helper.add_world_to_stage(base_frame="/World")

        # Ajouter le robot à la scène
        self.robotAPI.add_robot_to_scene(self.world)
        
    def add_target(self, position = None, orientation = None):
        # Add a target to the scene
        self.target_height = 0.07
        self.target_width = 0.04
        self.target_depth = 0.05
        
        self.random_cube = self.world.scene.add(  
        VisualCuboid(
            prim_path="/World/random_cube",
            name="random_cube",
            position=np.array([0.4, -0.2, self.target_height/2.0]),
            scale=np.array([self.target_depth, self.target_width, self.target_height]),
            color=np.array([0, 1.0, 1.1]),
            orientation=np.array(euler_angles_to_quats([3.1415927 ,0,0 ])),
        ))
        self.cube_position = self.random_cube.get_world_pose()[0]
        self.cube_position[2] += self.target_height/2-0.030
        self.cube_orientation = self.random_cube.get_world_pose()[1]
    
    def get_cube_grip_pos_orient(self):
        self.cube_position = self.random_cube.get_world_pose()[0]
        self.cube_position[2] += self.target_height/2-0.030
        self.cube_orientation = self.random_cube.get_world_pose()[1]
        
    def set_cube_pos_orient(self,goal_position,goal_orientation):
        self.random_cube.set_world_pose(goal_position,goal_orientation)



class CuroboBaseSetup():
    def __init__(
         self,
         my_task: BaseTask,
         name: str = "curobo_controller",
         robotAPI = None,
         motion_planner = None,
         world = None,
         ) -> None:
        self.my_task = my_task
        self.robotAPI = robotAPI
        self.world = world
        self.motion_planner = motion_planner
        

    def execute_motion_traj_real(self,goal_position, goal_orientation, is_sim_enable):
        self.motion_planner.forward_real(goal_position, goal_orientation)
        self.motion_planner.pose_list
        self.robotAPI.joint_move_to_real_traj(self.motion_planner.pose_list, is_sim_enable)
        self.motion_planner.pose_list = []

    def execute_motion_traj_sim(self,goal_position, goal_orientation):
        print("Executing motion")
        art_action = self.motion_planner.forward_sim(goal_position, goal_orientation)
        self.robotAPI.joint_move_to_sim_traj(art_action,self.motion_planner.pose_list)
        self.motion_planner.pose_list = []
        #self.world.step(render=True) 
    
    def reset(self):
        self.world.reset()
        self.robotAPI.manipulator._articulation_view.initialize()#maybe not required because already added to world see : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#omni.isaac.core.articulations.Articulation.initialize
        self.articulation_controller = self.robotAPI.manipulator.get_articulation_controller()
        self.idx_list = [self.robotAPI.manipulator.get_dof_index(x) for x in self.robotAPI.j_names]
        self.robotAPI.manipulator.set_joint_positions(self.robotAPI.default_config, self.idx_list)
        self.robotAPI.manipulator._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(self.idx_list))]), joint_indices=self.idx_list
            )
        self.cmd_idx = 0
        self.pose_list = []
    

class MotionPlanning(ABC):
    @abstractmethod
    def __init__(self):
        pass
        
    @abstractmethod
    def motion_gen_cfg(self):
        pass
    
    @abstractmethod
    def update_world_obstacles(self):
        pass 
    
    @abstractmethod
    def forward_sim():
        pass
    
    @abstractmethod
    def forward_real():
        pass
    
    @abstractmethod       
    def plan_sim():
        pass
    @abstractmethod       
    def plan_real():
        pass
    
    @abstractmethod
    def attach_object():    
        pass
    
    @abstractmethod
    def detach_object():
        pass
    

class CuroboMotionGen(MotionPlanning):
    def __init__(self,arm_api, world_manager, constraint_approach=False):
        self.tensor_args = TensorDeviceType()
        self.cmd_plan = None
        self.pose_metric = None
        self.robot_cfg = arm_api.robot_cfg
        self.world_cfg = world_manager.world_cfg
        self.constraint_approach = constraint_approach
        self.real_arm = arm_api.real_arm
        self.sim_arm = arm_api.sim_arm
        self.pose_list = []
        if self.sim_arm is None and self.real_arm is None:
            raise Exception("No robot API provided")
        self.motion_gen_cfg()
        self.motion_gen_plan_cfg()
        self.robot_prim_path = "/World/UF_ROBOT" # attention same as add_reference_to_stage(usd_path=asset_path, prim_path="/World/UF_ROBOT")
        self.usd_help = world_manager.usd_helper
        self.joint_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        
    def motion_gen_cfg(self):
        
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 30
        trajopt_dt = None
        optimize_dt = True
        trajopt_tsteps = 32
        trim_steps = None
        interpolation_dt = 0.05
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=24,
            num_graph_seeds=24,
            interpolation_dt=interpolation_dt,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
            use_cuda_graph = True,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.update_batch_size(seeds=20,batch=2)
        print("warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    
    def motion_constraint(self):
        # add constraints to the motion gen here : linear movement along z axis of the end effector
        self.pose_metric = PoseCostMetric.create_grasp_approach_metric(offset_position=0.12,tstep_fraction=0.6, linear_axis=2)
        
    def motion_gen_plan_cfg(self):
        
        max_attempts = 50
        
        if self.constraint_approach == True :
            print("Contraint approach")
            self.motion_constraint()
        self.plan_config = MotionGenPlanConfig(
            enable_graph=True,
            need_graph_success=True,
            enable_opt = True,
            max_attempts=max_attempts,
            enable_graph_attempt=50,
            enable_finetune_trajopt=True,
            partial_ik_opt=False,
            parallel_finetune=True,
            pose_cost_metric=self.pose_metric,
            time_dilation_factor=0.5,
            timeout = 50,
            finetune_attempts = 20
        )
        
        #self.motion_gen_result = MotionGenResult() ###

        print("Curobo is Ready")
        
    def plan_real(self,goal_position, goal_orientation):
        # Generate a plan to reach the goal
        ik_goal = Pose(
            position=self.tensor_args.to_device(goal_position),
            quaternion=self.tensor_args.to_device(goal_orientation),
        )
        sim_js = self.sim_arm.get_joints_state()
        angles = self.real_arm.angles + [0.0, 0.0, 0.0, 0.0, 0.0]
        angles_array = torch.tensor(angles, dtype=torch.float32,device='cuda:0')
        cu_js = JointState(
            position=angles_array,
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=self.robot_cfg["kinematics"]["cspace"]["joint_names"],
        )
        cu_js = cu_js.get_ordered_joint_state(self.robot_cfg["kinematics"]["cspace"]["joint_names"])##########################################Changer la position de depart ici
        # see : https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen.plan_single
        result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())
        return result
    
    def forward_real(self,goal_position, goal_orientation):
        if self.pose_list == []:
            result = self.plan_real(goal_position, goal_orientation)            
            succ = result.success.item()
            if succ:
                print("Plan converged to a solution.")
                cmd_plan = result.get_interpolated_plan()
                self.cmd_plan = cmd_plan.get_ordered_joint_state(self.robot_cfg["kinematics"]["cspace"]["joint_names"])
                self.pose_list = cmd_plan.position.tolist()
            else:
                carb.log_warn("Plan did not converge to a solution.")
                return None
    
    def plan_sim(self,goal_position, goal_orientation):
        # Generate a plan to reach the goal
        ik_goal = Pose(
            position=self.tensor_args.to_device(goal_position),
            quaternion=self.tensor_args.to_device(goal_orientation),
        )
        sim_js = self.sim_arm.get_joints_state()

        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=self.robot_cfg["kinematics"]["cspace"]["joint_names"],
        )
        cu_js = cu_js.get_ordered_joint_state(self.motion_gen.kinematics.joint_names)##########################################Changer la position de depart ici
        # see : https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html#curobo.wrap.reacher.motion_gen.MotionGen.plan_single
        result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())
        return result
    
    def forward_sim(self,goal_position, goal_orientation):### De we need this methode or can we passe the whole movement like for the real robot ?
        if self.cmd_plan is None:
            self.cmd_idx = 0
            self._step_idx = 0
            # Execute the plan
            result = self.plan_sim(goal_position, goal_orientation)            
            succ = result.success.item()
            if succ:
                print("Plan converged to a solution.")
                cmd_plan = result.get_interpolated_plan()
                # print(cmd_plan)
                self.idx_list = [i for i in range(len(self.robot_cfg["kinematics"]["cspace"]["joint_names"]))]
                self.cmd_plan = cmd_plan.get_ordered_joint_state(self.robot_cfg["kinematics"]["cspace"]["joint_names"])
                self.pose_list = cmd_plan.position.tolist()
            else:
                carb.log_warn("Plan did not converge to a solution.")
                return None
        if self._step_idx % 1 == 0 :
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
    
     
    
    def update_world_obstacles(self,ignore_substring):
        print("Updating world, reading w.r.t.", self.robot_prim_path)
        obstacles = self.usd_help.get_obstacles_from_stage(
            reference_prim_path=self.robot_prim_path,
            ignore_substring=ignore_substring,
        ).get_collision_check_world()
        self._world_cfg = obstacles
        self.motion_gen.update_world(obstacles)
           
    def attach_object_real(self,cube_name,robot_api):    
        # Attach the object to the robot and add the object to the motion generator
        sim_js = robot_api.robot.get_joints_state()
        # Ajouter 5 zeros au angles
        angles = self.real_arm.angles + [0.0, 0.0, 0.0, 0.0, 0.0]
        angles_array = torch.tensor(angles, dtype=torch.float32,device='cuda:0')
        cu_js = JointState(
            position=angles_array,
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=robot_api,
        )
        self.motion_gen.attach_objects_to_robot(
            cu_js,
            [cube_name],
            link_name="attached_object",
            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
            world_objects_pose_offset=Pose.from_list([0, 0, -0.17, 1, 0, 0, 0], self.tensor_args),
            remove_obstacles_from_world_config = False,
            surface_sphere_radius = 0.005
        )
    def attach_object_sim(self,cube_name,robot_api):    
        # Attach the object to the robot and add the object to the motion generator
        sim_js = robot_api.sim_arm.get_joints_state()
        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=robot_api,
        )
        self.motion_gen.attach_objects_to_robot(
            cu_js,
            [cube_name],
            link_name="attached_object",
            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
            world_objects_pose_offset=Pose.from_list([0, 0, -0.17, 1, 0, 0, 0], self.tensor_args),
            remove_obstacles_from_world_config = False,
            surface_sphere_radius = 0.005
        )
    def attach_object():
        pass
    def detach_object(self):
        # Detach the object from the robot and remove the object from the motion generator
        self.motion_gen.detach_object_from_robot(link_name="attached_object",)
    
#  ###### A Completer ######   
class MPC():
    def __init__(self,arm_api,world_manager):
        self.robot_cfg = arm_api.robot_cfg
        self.world_cfg = world_manager.world_cfg
        self.usd_help = world_manager.usd_helper
        self.world = world_manager.world
    def motion_gen_cfg(self):
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 10
        mpc_config = MpcSolverConfig.load_from_robot_config(
        self.robot_cfg,
        self.world_cfg,
        use_cuda_graph=True,
        use_cuda_graph_metrics=True,
        use_cuda_graph_full_step=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        use_mppi=True,
        use_lbfgs=False,
        use_es=False,
        store_rollouts=True,
        step_dt=0.02,
        )

        mpc = MpcSolver(mpc_config)
        
        retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        joint_names = mpc.rollout_fn.joint_names

        state = mpc.rollout_fn.compute_kinematics(
            JointState.from_position(retract_cfg, joint_names=joint_names)
        )
        
        current_state = JointState.from_position(retract_cfg, joint_names=joint_names)
        retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)
        
        goal = Goal(
            current_state=current_state,
            goal_state=JointState.from_position(retract_cfg, joint_names=joint_names),
            goal_pose=retract_pose,
        )

        goal_buffer = mpc.setup_solve_single(goal, 1)
        mpc.update_goal(goal_buffer)
        mpc_result = mpc.step(current_state, max_attempts=2)

        self.usd_help.load_stage(self.world.stage)
        init_world = False
        cmd_state_full = None
        step = 0
        add_extensions(simulation_app, args.headless_mode)
        

class ArmAPI():
    pass

class XArm6API():
    def __init__(self,is_real_robot_enable=False):
        self.is_real_robot_enable = is_real_robot_enable
        self.is_api_ok = False
        self.pose_list = []
        self.real_arm = None
        self.sim_arm = None
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
        
        self.manipulator = SingleManipulator(prim_path="/World/UF_ROBOT", name="UF_ROBOT",
                                                end_effector_prim_name="xarm6link_tcp", gripper=self.gripper)
        
    def add_robot_to_scene(self, world):# a mettre dans la classe de plus haut au dessus
        self.sim_arm = world.scene.add(self.manipulator)
        self.articulation_controller = self.sim_arm.get_articulation_controller()
        
    def set_first_pose_real(self):# a mettre dans la classe de plus haut au dessus
        self.default_config
        self.real_arm.set_servo_angle(angle=self.default_config,speed=100,wait=False)
        
    def set_first_pose_sim(self):
        self.sim_arm._articulation_view.initialize() #High level wrapper to deal with prims (one or many) that have the Root Articulation API applied and their attributes/properties
        self.articulation_controller = self.sim_arm.get_articulation_controller()
        self.idx_list = [self.sim_arm.get_dof_index(x) for x in self.j_names]
        self.sim_arm.set_joint_positions(self.default_config, self.idx_list)
        self.sim_arm._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(self.idx_list))]), joint_indices=self.idx_list
            )
    
    def sync_real_to_sim(self):# a mettre dans la classe de plus haut au dessus
        pass
    
    def go_home(self):
        pass
        
    def connect_to_arm(self):
        try:
            self.real_arm = XArmAPI("192.168.1.215", is_radian=True)
            self.real_arm.motion_enable(enable=True)
            self.real_arm.set_gripper_enable(1)
            self.real_arm.set_mode(6)
            self.real_arm.set_state(state=0)
            self.is_api_ok = True
        except Exception as e:
            print("Failed to connect to xArm")
            self.is_api_ok = False
        
    def close_gripper_real(self):
        if self.real_arm is not None and self.real_arm.get_gripper_position()[1] > 0.01:
            self.real_arm.set_gripper_position(0, wait=True)
            
                
    def close_gripper_sim(self,world):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] < 0.688 :
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] + 0.628, gripper_positions[1] - 0.628]))
            world.step(render=True)
            
    def open_gripper_real(self):
        if self.real_arm is not None:
            self.real_arm.set_gripper_position(850, wait=True)
            

    def open_gripper_sim(self,world):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] > 1e-3 :
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] - 0.628, gripper_positions[1] + 0.628]))
            world.step(render=True)
            
    def joint_move_to_real_traj(self, pose_list, is_sim_enable, speed=100):
        
        if self.real_arm is not None:
            #print("Real arm is moving pose list", pose_list)
            for i in range(len(pose_list)):
                print("Positon numero : ",i)
                self.real_arm .set_servo_angle(angle=pose_list[i],speed=speed,wait=False) 
                time.sleep(0.04)
                if is_sim_enable:
                        art_action = ArticulationAction(
                            joint_positions=pose_list[i]+np.array([0,0,0,0,0,0]),
                            joint_velocities=np.zeros(6),
                            joint_indices=[0,1,2,3,4,5],
                        )
                        if art_action is not None:
                            print("Applying action")
                            self.articulation_controller.apply_action(art_action)  #See : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=apply_action#omni.isaac.core.controllers.ArticulationController.apply_action
                        else:
                            print("No action") 
                
    def joint_move_to_sim_traj(self, art_action, pose_list):
        #print("Art action xarm api",art_action)
        if art_action is not None:
            print("Applying action")
            self.articulation_controller.apply_action(art_action)
        if self.is_real_robot_enable and pose_list != []:
            if self.is_api_ok :
                #print("Real arm is moving", pose_list)
                self.joint_move_to_real_traj(pose_list,False,speed=100)
            else:
                print("Error no communication with real robot")
        
    
    def is_target_reached_sim(self, goal_position, goal_orientation,real_arm:XArmAPI=None)->bool:
        # Check if the target has been reached
        current_eef_position = self.sim_arm.end_effector.get_world_pose()[0]
        current_eef_orientation = self.sim_arm.end_effector.get_world_pose()[1]
        if ((np.linalg.norm(goal_position - current_eef_position) < 0.015)and ((2*np.arccos(np.abs(np.dot(goal_orientation, current_eef_orientation))))< 0.015)):
            return True
        else:
            return False
        
            
        
    def is_target_reached_real(self, goal_position, goal_orientation)->bool:
        xyz_rpy = np.array(self.real_arm.position) 
        xyz = xyz_rpy[:3]/1000.0
        rpy = xyz_rpy[3:]
        w_qx_qy_qz=euler_angles_to_quats(rpy) 
        # print("Real arm xyz",xyz)
        # print("real_arm error in position",np.linalg.norm(goal_position - xyz))
        # print("real_arm error in orientation",2*np.arccos(np.abs(np.dot(goal_orientation, w_qx_qy_qz))))  
        # print("Real arm w_qx_qy_qz",w_qx_qy_qz)
        xyz_w_qx_qy_qz = np.append(xyz,w_qx_qy_qz)
        # print((np.linalg.norm(goal_position - xyz) < 0.01))
        # print((2*np.arccos(np.abs(np.dot(goal_orientation, w_qx_qy_qz))))< 0.02)
        if ((np.linalg.norm(goal_position - xyz) < 0.01) and ((2*np.arccos(np.abs(np.dot(goal_orientation, w_qx_qy_qz))))< 0.022)):
            print("Real_Terget_reached")
            return True
        else:
            return False
    
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


        
    
    

####Attention update collision checking with the cube before picking
def main():
    #1 Robot définition
    #2 World definition
    #3 Motion planning
    

    arm_API = XArm6API(True)
    arm_API.connect_to_arm()
    world_manager = ManageCuroboScene(robotAPI=arm_API)
    world_manager.add_target()
    motion_planner = CuroboMotionGen(arm_API, world_manager)
    mpc = MPC(arm_API,world_manager)
    mpc.motion_gen_cfg()
    curobo = CuroboBaseSetup("test",robotAPI=arm_API,motion_planner=motion_planner,world= world_manager.world)
    
    position = world_manager.cube_position
    orientation = world_manager.cube_orientation
    prog_step = 0
    curobo.reset()
    curobo.robotAPI.open_gripper_sim(world_manager.world)
    #curobo.robotAPI.open_gripper_real()
    curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT"])
    while simulation_app.is_running():
        
        world_manager.world.step(render=True)
        
        if not world_manager.world.is_playing():  
            #world_manager.world.reset()
            curobo.robotAPI.set_first_pose_sim()
            curobo.robotAPI.open_gripper_sim(world_manager.world)
            curobo.robotAPI.open_gripper_real()
            curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT"])
            continue   
        
        curobo.execute_motion_traj_sim(position,orientation)
        
        if prog_step == 0:
            position = world_manager.cube_position
            orientation = world_manager.cube_orientation
            if motion_planner.cmd_plan is None and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                curobo.motion_planner.attach_object_sim(cube_name="/World/random_cube",robot_api=arm_API) 
                curobo.motion_planner.update_world_obstacles(["UF_ROBOT","random_cube","DefaultGroundPlane","table"])
                prog_step=1
                curobo.robotAPI.close_gripper_sim(world_manager.world)
                curobo.robotAPI.close_gripper_real()
                world_manager.get_cube_grip_pos_orient() 
        if prog_step ==1:
            position = (world_manager.cube_position.copy() + np.array([0.0, 0.0, 0.35]))
            orientation = euler_angles_to_quats([3.1415927 ,0,0 ])
            if motion_planner.cmd_plan is None  and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                curobo.motion_planner.update_world_obstacles(["UF_ROBOT","random_cube"])
                prog_step=2
                
        if prog_step ==2:
            position = [0.43, 0.26, 0.42]
            orientation = [0, 1, 0, 0.0]
            if motion_planner.cmd_plan is None and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                prog_step=3
                
        if prog_step ==3:
            position = [0.40, 0.40, 0.25]
            orientation = [0, 0.7071, 0, 0.7071]
            if motion_planner.cmd_plan is None  and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                prog_step=4
                visualize_sphere(curobo.motion_planner.motion_gen, cu_js, spheres=None) 
                
        if prog_step == 4:
            position = [0.43, 0.27, 0.42]
            orientation = [0, 1, 0, 0.0]
            if motion_planner.cmd_plan is None  and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT"])
                curobo.robotAPI.open_gripper_sim(world_manager.world)
                curobo.robotAPI.open_gripper_real()
                curobo.motion_planner.detach_object()
                prog_step=5
                
        if prog_step == 5:
            position = [0.40, 0.40, 0.25]
            orientation = [0, 0.7071, 0, 0.7071]
            if motion_planner.cmd_plan is None  and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                world_manager.set_cube_pos_orient([0.43, 0.27, 0.38],[0,1,0,0])
                world_manager.get_cube_grip_pos_orient()
                curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT"])
                prog_step=6
                
        if prog_step == 6:
            position = (world_manager.cube_position)
            orientation = [0, 1, 0, 0.0]
            #print("Curobo target reached : ",curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API))
            if curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                prog_step = 7
                world_manager.get_cube_grip_pos_orient()
                
        if prog_step == 7:
            
            position = world_manager.cube_position - np.array([0.0,0,0.01])
            orientation = [0,1,0,0]
            if motion_planner.cmd_plan is None  and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                curobo.robotAPI.close_gripper_sim(world_manager.world)
                curobo.robotAPI.close_gripper_real()
                curobo.motion_planner.attach_object_sim(cube_name="/World/random_cube",robot_api=arm_API)
                curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT","random_cube","rack_rp","obstacles/rack_rp"])
                prog_step = 8
                world_manager.get_cube_grip_pos_orient()
                
        if prog_step == 8:
             
            position =(world_manager.cube_position + np.array([0.0,0,0.02]))
            orientation = [0,1,0,0]
            if motion_planner.cmd_plan is None and curobo.robotAPI.is_target_reached_sim(goal_position=position, goal_orientation=orientation,real_arm=arm_API):
                curobo.motion_planner.update_world_obstacles(ignore_substring=["UF_ROBOT","random_cube"])
                prog_step = 3
  
        world_manager.world.step(render=True)
        
        #################################Uncomment to show spheres##############################        
        sim_js = curobo.robotAPI.sim_arm.get_joints_state()    
        sim_js_names = curobo.robotAPI.sim_arm.dof_names
        cu_js = JointState(
        position=curobo.motion_planner.tensor_args.to_device(sim_js.positions),
        velocity=curobo.motion_planner.tensor_args.to_device(sim_js.velocities),  # * 0.0,
        acceleration=curobo.motion_planner.tensor_args.to_device(sim_js.velocities) * 0.0,
        jerk=curobo.motion_planner.tensor_args.to_device(sim_js.velocities) * 0.0,
        joint_names=sim_js_names,
        )

        cu_js.velocity *= 0.0
        cu_js.acceleration *= 0.0

        cu_js = cu_js.get_ordered_joint_state(curobo.motion_planner.joint_names)
        
        #visualize_sphere(curobo.motion_planner.motion_gen, cu_js, spheres=None)    
        #################################Uncomment to show spheres############################## 
        
        

    simulation_app.close()
             
    
if __name__ == "__main__":
    main()