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

class CuroboBaseSetup():
    def __init__(
         self,
         my_task: BaseTask,
         name: str = "curobo_controller",
         robotAPI = None,
         ) -> None:
        self.my_task = my_task
        self.usd_helper = UsdHelper()
        self.robotAPI = robotAPI
        self.world = None
    
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
        self.usd_helper.load_stage(self.world.stage)
        self.usd_helper.add_world_to_stage(self.world_cfg, base_frame="/World")
        self.robotAPI.add_robot_to_scene(self.world)
    def reset():
        pass
    

class MotionPlanning(ABC):
    def __init__(self):
        pass
    def config_motion_gen(self):
        pass
    def update_world_obstacles(self):
        pass        
    def plan():
        pass
    def attach_object():    
        pass
    def detach_object():
        pass
    

class CuroboMotionGen(MotionPlanning):
    def __init__(self):
        pass
    def config_motion_gen(self):
        pass
    def update_world_obstacles(self):
        pass        
    def plan():
        pass
    def attach_object():    
        pass
    def detach_object():
        pass
    
    
# class RMPFlow():
#     pass

# class ArmAPI():
#     pass

class XArm6API():
    def __init__(self):
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
        
        self.manipulator = SingleManipulator(prim_path="/World/UF_ROBOT", name="UF_ROBOT",
                                                end_effector_prim_name="xarm6link_tcp", gripper=self.gripper)
        
    def add_robot_to_scene(self, world):
        self.robot = world.scene.add(self.manipulator)
        
    def set_first_pose_real(self):
        self.default_config
        self.real_arm.set_servo_angle(angle=self.default_config,speed=100,wait=False)
        
    def set_first_pose_sim(self):
        self.robot._articulation_view.initialize() #High level wrapper to deal with prims (one or many) that have the Root Articulation API applied and their attributes/properties
        self.articulation_controller = self.robot.get_articulation_controller()
        self.idx_list = [self.robot.get_dof_index(x) for x in self.j_names]
        self.robot.set_joint_positions(self.default_config, self.idx_list)
        self.robot._articulation_view.set_max_efforts(
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
        except Exception as e:
            print(e)
            print("Failed to connect to xArm")
        
    def close_gripper_real(self):
        if self.real_arm is not None:
            self.real_arm.set_gripper_position(0, wait=True)
            while code!=0 :
                code = self.real_arm.get_gripper_position()[0]
                
    def close_gripper_sim(self):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] < 0.628 :
            print(gripper_positions)
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] + 0.628, gripper_positions[1] - 0.628]))
            self.my_world.step(render=True)
            
    def open_gripper_real(self):
        if self.real_arm is not None:
            self.real_arm.set_gripper_position(850, wait=True)
            while code!=0 :
                code = self.real_arm.get_gripper_position()[0]

    def open_gripper_sim(self):
        gripper_positions = self.gripper.get_joint_positions()
        while gripper_positions[0] > 1e-3 :
            print(gripper_positions)
            gripper_positions = self.gripper.get_joint_positions()
            self.gripper.apply_action(ArticulationAction(joint_positions=[gripper_positions[0] - 0.628, gripper_positions[1] + 0.628]))
            self.my_world.step(render=True)
            
    def move_to_real(self, pose, speed=100):
        if self.real_arm is not None:
            self.real_arm.set_servo_angle(angle=pose,speed=speed,wait=False)
            
    def move_to_real_traj(self, pose_list, speed=100):
        if self.real_arm is not None:
            i=0
            #while i < len(curobo.pose_list):
            for i in range(len(pose_list)):
                print("Positon numero : ",i)
                self.real_arm .set_servo_angle(angle=pose_list[i],speed=speed,wait=False) 
                #time.sleep(0.04)
                time.sleep(0.1)
        
        
    
    

####Attention update collision checking with the cube before picking

class ManageCuroboScene(BaseTask):
    pass
        

def main():
    arm = XArm6API()
    curobo = CuroboBaseSetup("test",robotAPI=arm)
    curobo.setup_scene()
    
    while simulation_app.is_running():
        time.sleep(0.01)
        curobo.world.step(render=True)
        if not curobo.world.is_playing():  
            curobo.world.reset()
            curobo.robotAPI.set_first_pose_sim()
            continue
    simulation_app.close()
             
    
if __name__ == "__main__":
    main()