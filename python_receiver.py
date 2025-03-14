# python_receiver.py
import socket
from scipy.spatial.transform import Rotation as R
import numpy as np

class UnityUDPReceiver:
    def __init__(self, receive_ip="0.0.0.0", receive_port=8001, unity_ip="127.0.0.1", send_port=8000):
        self.receive_ip = receive_ip
        self.receive_port = receive_port
        self.unity_ip = unity_ip
        self.send_port = send_port

        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口快速重用
        self.sock_receive.bind((self.receive_ip, self.receive_port))

        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print(f"[INFO] UnityUDPReceiver 已启动，监听地址: {self.receive_ip}:{self.receive_port}")

    def send_to_unity(self, message: str):
        self.sock_send.sendto(message.encode('utf-8'), (self.unity_ip, self.send_port))

    def receive_from_unity(self):
        data, _ = self.sock_receive.recvfrom(1024)
        message = data.decode('utf-8')

        print(f"[DEBUG] 收到来自 Unity 的原始消息：\n{message}")
        lines = message.strip().split('\n')

        hand_position, hand_rotation, finger_distance = None, None, None

        for line in lines:
            header, values = line.split('\t')
            if header == "_hand_position":
                hand_position = [float(x) for x in values.split(',')]
            elif header == "_hand_rotation":
                hand_rotation = [float(x) for x in values.split(',')]
            elif header == "_finger_distance":
                finger_distance = float(values)

        return hand_position, hand_rotation, finger_distance

    def unity_to_isaac_transform(self, hand_position, hand_rotation):
        unity_pos = np.array(hand_position)
        isaac_pos = [unity_pos[2], -unity_pos[0], unity_pos[1]]

        unity_quat = [hand_rotation[0], hand_rotation[1], hand_rotation[2], hand_rotation[3]]
        unity_rot = R.from_quat(unity_quat)
        unity_euler = unity_rot.as_euler('xyz', degrees=True)

        isaac_euler = [unity_euler[2], -unity_euler[0], unity_euler[1]]
        isaac_rot = R.from_euler('xyz', isaac_euler, degrees=True)
        isaac_quat = isaac_rot.as_quat()

        return isaac_pos, isaac_quat
