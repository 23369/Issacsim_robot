# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.controllers import BaseController
import carb
import numpy as np
# Can be used to create a new cube or to point to an already existing cube in stage.
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html
"""
class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
"""

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube", # The prim path of the cube in the USD stage
                name="fancy_cube", # The unique name used to retrieve the object from the scene later on
                position=np.array([1.5, 0, 1.0]), # Using the current stage units which is in meters by default.
                scale=np.array([0.5015, 0.5015, 0.5015]), # most arguments accept mainly numpy arrays.
                color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
            ))

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            # Use carb to log warnings, errors, and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )
        )
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("fancy_robot")
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        # Initialize our controller after load and the first reset
        self._my_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                    is_holonomic=False)
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return

    def send_keyboard_actions(self, step_size):
        return
    
    def send_robot_actions(self, step_size):
        if self._jetbot is None:
            print("Error: jetbot is None")

        #print("DOFs:", self._jetbot.dof_names)
        position, orientation = self._jetbot.get_world_pose()
        goal = np.array([0.8, 0.8])
        # 计算距离误差（只看 X-Y）
        distance_to_goal = np.linalg.norm(position[:2] - goal)
        # 误差小于一定阈值 → 停止
        if distance_to_goal < 0.05:
            print("[INFO] Reached goal, stopping.")
            action = ArticulationAction(joint_velocities=[0.0, 0.0])
        else:
            action = self._my_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=goal
            )
            print(f"[DEBUG] Distance to goal: {distance_to_goal:.3f}, action: {action.joint_velocities}")

        # 应用动作
        self._jetbot.apply_action(action)
        return
    