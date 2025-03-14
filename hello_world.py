# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.core.api.tasks import BaseTask
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid


# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html

class FrankaPlaying(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        self._task_achieved = False
        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._cube = scene.add(DynamicCuboid(
                    prim_path="/World/random_cube",
                    name="fancy_cube",
                    position=np.array([0.3, 0.3, 0.3]),
                    scale=np.array([0.0515, 0.0515, 0.0515]),
                    color=np.array([0, 0, 1.0]),
                ))
        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka",
                                         name="fancy_franka"))
        return
    
    def get_observations(self):
        cube_position, _ = self._cube.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()
        observations = {
            self._franka.name: {
                "joint_positions": current_joint_positions,
            },
            self._cube.name: {
                "position": cube_position,
                "goal_position": self._goal_position
            }
        }
        return observations
    
    def pre_step(self, control_index, simulation_time):
        cube_position, _ = self._cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._goal_position - cube_position)) < 0.02:
            # Visual Materials are applied by default to the cube
            # in this case the cube has a visual material of type
            # PreviewSurface, we can set its color once the target is reached.
            self._cube.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))
            self._task_achieved = True
        return
    
    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
        self._task_achieved = False
        return

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._step_counter = 0  # 新增：用于计数 step
        return

    def setup_scene(self):
        world = self.get_world()
        world.add_task(FrankaPlaying(name="franka_playing"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
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
        await self._world.play_async()    # 这个相当于play simulation
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
        current_observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations["fancy_cube"]["position"],
            placing_position=current_observations["fancy_cube"]["goal_position"],
            current_joint_positions=current_observations["fancy_franka"]["joint_positions"],
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
    
    