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
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.controllers import BaseController
import carb
import numpy as np
# Can be used to create a new cube or to point to an already existing cube in stage.
from isaacsim.core.api.objects import DynamicCuboid


# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._step_counter = 0  # 新增：用于计数 step
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )

        
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._fancy_cube = self._world.scene.get_object("fancy_cube")
        # Initialize a pick and place controller
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        # World has pause, stop, play..etc
        # Note: if async version exists, use it in any async function is this workflow
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        #await self._world.play_async()    # 这个相当于play simulation
        # 获取当前夹爪关节值
        gripper_positions = self._franka.gripper.get_joint_positions()
        # 估算夹爪的开口宽度（两边夹指的总距离）
        gripper_opening = sum(gripper_positions)
        print(f"🔧 当前夹爪开口宽度估计为：{gripper_opening:.4f} 米 单个的{gripper_positions}")
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        self._controller.reset()
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return

    def world_cleanup(self):
        return

    def send_keyboard_actions(self, step_size):
        return
    
    def physics_step(self, step_size):
        # Step 计数器 +1
        self._step_counter += 1
        cube_position, _ = self._fancy_cube.get_world_pose()
        goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        current_joint_positions = self._franka.get_joint_positions()
        actions = self._controller.forward(
            picking_position=cube_position,
            placing_position=goal_position,
            current_joint_positions=current_joint_positions,
        )
        self._franka.apply_action(actions)
         # 每 5 步打印一次夹爪 position
        if self._step_counter % 5 == 0:
            gripper_pos = self._franka.gripper.get_joint_positions()
            print(f"[Step {self._step_counter}] 🤖 当前夹爪位置：{gripper_pos}，总开口：{sum(gripper_pos):.4f} 米")
        # Only for the pick and place controller, indicating if the state
        # machine reached the final state.
        if self._controller.is_done():  
            self._world.pause()   # 这个相当于pause simulation
        return 
    
    