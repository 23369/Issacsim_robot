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
import carb
import numpy as np
# Can be used to create a new cube or to point to an already existing cube in stage.
from isaacsim.core.api.objects import DynamicCuboid
# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


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
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
        jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
        print("Num of degrees of freedom before first reset: " + str(jetbot_robot.num_dof)) # prints None
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("fancy_robot")
        self._elapsed_time = 0.0
        self._jetbot_articulation_controller = self._jetbot.get_articulation_controller()
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return

    def send_robot_actions(self, step_size):
            # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
            # as optional args. It accepts numpy arrays of floats OR lists of floats and None
            # 每一步都会调用，step_size 是时间间隔，通常是 1/60 秒
        self._elapsed_time += step_size  # 累加时间

        if self._elapsed_time < 2.0:
            # 0 ~ 2 秒：后退
            velocities = [-2.0, -2.0]
            print("[Action] Backward")
        elif self._elapsed_time < 4.0:
            # 2 ~ 4 秒：右转（左轮转，右轮停）
            velocities = [2.0, 0.0]
            print("[Action] Turn Right")
        elif self._elapsed_time < 5.0:
            # 4 ~ 5 秒：继续转
            velocities = [1.0, -1.0]
            print("[Action] Sharp Right")
        else:
            # 5 秒后停止
            velocities = [0.0, 0.0]
            print("[Action] Stop")


        self._jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                                joint_efforts=None,
                                                                                joint_velocities=velocities))
        return