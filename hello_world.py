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
from omni.isaac.core.utils.types import ArticulationAction
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np
import asyncio
from isaacsim.examples.interactive.hello_world.python_receiver import UnityUDPReceiver


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.receiver = UnityUDPReceiver()
        self.goal_position = np.array([0.3, 0.0, 0.3])  # 初始化安全位置
        self.goal_rotation = np.array([0.0, 0.0, 0.0, 1.0])
        self.finger_distance = 0.13  # 初始不夹取状态
        self.sim_running = False
        self.receiver_task = None

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([0.3, 0.3, 0.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._fancy_cube = self._world.scene.get_object("fancy_cube")
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        # 不自动启动仿真与接收线程，等用户点击 Play 再进入 setup_post_reset

    async def setup_post_reset(self):
        if not self.sim_running:
            self.sim_running = True
            asyncio.get_event_loop().run_in_executor(None, self.update_goal_pose)
        await self._world.play_async()

    def update_goal_pose(self):
        while self.sim_running:
            hand_position, hand_rotation, finger_distance = self.receiver.receive_from_unity()
            self.goal_position, self.goal_rotation = self.receiver.unity_to_isaac_transform(hand_position, hand_rotation)
            self.finger_distance = finger_distance

    def physics_step(self, step_size):
        if not self.sim_running:
            return

        # 使用逆运动学解算
        target_joint_positions = self._franka.compute_inverse_kinematics(
            target_position=self.goal_position,
            target_orientation=self.goal_rotation
        )

        # 根据手指距离判断夹取状态
        if self.finger_distance <= 0.04:
            gripper_joint_positions = self._franka.gripper.joint_closed_positions  # 夹取
        else:
            gripper_joint_positions = self._franka.gripper.joint_opened_positions  # 不夹取

        # 应用动作
        actions = ArticulationAction(
            joint_positions=np.concatenate((target_joint_positions, gripper_joint_positions))
        )

        self._franka.apply_action(actions)

    async def cleanup(self):
        self.sim_running = False
