from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


from omni.isaac.core import World

import numpy as np
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np
import omni.isaac.manipulators.controllers as manipulators_controllers
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.articulations import Articulation
import omni.isaac.motion_generation as mg




class RMPFlowController(mg.MotionPolicyController):

    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:

        # TODO: change the follow paths

        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path="rmpflow/robot_descriptor.yaml",

                                                        rmpflow_config_path="rmpflow/denso_rmpflow_common.yaml",

                                                        urdf_path="/home/theobloesch/xArm6_Pick_Place/with_rmpflow/xarm6_cfg_files/xarm6.urdf",

                                                        end_effector_frame_name="xarm6link_tcp",

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


    def reset(self):

        mg.MotionPolicyController.reset(self)

        self._motion_policy.set_robot_base_pose(

            robot_position=self._default_position, robot_orientation=self._default_orientation

        )






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




class PickPlace(tasks.PickPlace):

    def __init__(

        self,

        name: str = "denso_pick_place",

        cube_initial_position: Optional[np.ndarray] = [0.3,0.3,0.2],

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

            cube_size=np.array([0.0515, 0.0515, 0.08]),

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


my_world = World(stage_units_in_meters=1.0)

target_position = np.array([0.3, -0.3, 0])

target_position[2] = 0.0515 / 2.0

my_task = PickPlace(name="denso_pick_place", target_position=target_position)
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("denso_pick_place").get_params()

denso_name = task_params["robot_name"]["value"]

my_denso = my_world.scene.get_object(denso_name)

#initialize the controller

my_controller = PickPlaceController(name="controller", robot_articulation=my_denso, gripper=my_denso.gripper)

#task_params = my_world.get_task("denso_pick_place").get_params()

articulation_controller = my_denso.get_articulation_controller()

# articulation_controller = my_denso.get_articulation_controller()

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

            end_effector_offset=np.array([0, 0, 0]),

        )

        if my_controller.is_done():

            print("done picking and placing")

        articulation_controller.apply_action(actions)

simulation_app.close()


