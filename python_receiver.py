import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 8001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Python端UDP通信启动，等待数据中……")

while True:
    data, addr = sock.recvfrom(1024)
    message = data.decode('utf-8')

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

    print("\n收到Unity数据:")
    print(f"手腕位置: {hand_position}")
    print(f"手腕旋转(四元数): {hand_rotation}")
    print(f"拇指与食指距离: {finger_distance:.5f}")
