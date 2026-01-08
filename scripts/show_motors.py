#!/usr/bin/env python3
import time
import rclpy
from xixilowcmd.msg import LowCmd

rclpy.init()

node = rclpy.create_node('show_motor_info')
subscription = node.create_subscription(
    LowCmd,
    '/lowcmd',
    lambda msg: show_motors(msg),
    10
)

count = [0]

def show_motors(msg):
    count[0] += 1

    # 每5条显示一次
    if count[0] % 5 == 0:
        print(f"\n[消息 #{count[0]}] 总电机数: {len(msg.motor_cmd)}")

        # 显示活跃电机
        for m in msg.motor_cmd:
            if m.mode != 0:
                print(f"  id: {m.id}")
                print(f"    mode: {m.mode}")
                print(f"    q: {m.q}")
                print(f"    dq: {m.dq}")
                print(f"    kp: {m.kp}")
                print(f"    kd: {m.kd}")
                print(f"    tau: {m.tau}")
                print()

print("监听 /lowcmd，显示活跃电机...")
print("按 Ctrl+C 停止\n")

try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    print(f"\n总计接收 {count[0]} 条消息")

node.destroy_node()
rclpy.shutdown()
