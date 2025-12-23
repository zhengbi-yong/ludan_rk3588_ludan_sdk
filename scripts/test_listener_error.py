#!/usr/bin/env python3
"""
模拟listener中的错误情况
"""
import rclpy
from rclpy.node import Node
from xixilowcmd.msg import LowCmd, MotorCmd
import json

def main():
    print("🔍 测试 listener 错误重现")

    # 初始化ROS2
    rclpy.init()

    try:
        # 创建ROS2节点
        node = Node('test_listener_error')

        # 创建发布器
        lowcmd_pub = node.create_publisher(LowCmd, '/lowcmd', 10)

        # 模拟接收到的消息
        test_message = {
            'timestamp': 1234567890,
            'sequence': 1,
            'motor_cmd': {
                '4': {'q': 0.1, 'dq': 0.0, 'tau': 0.0, 'mode': 1, 'kp': 20.0, 'kd': 2.0},
                '5': {'q': 0.2, 'dq': 0.0, 'tau': 0.0, 'mode': 1, 'kp': 20.0, 'kd': 2.0}
            }
        }

        print("📨 模拟接收到的消息:")
        print(json.dumps(test_message, indent=2))

        # 尝试创建 LowCmd 消息（和listener中的代码一样）
        try:
            motor_cmd_dict = test_message['motor_cmd']
            print(f"处理xixilowcmd格式，包含{len(motor_cmd_dict)}个电机")

            lowcmd_msg = LowCmd()

            # 创建 MotorCmd 数组 (30个电机) - 完全按照listener的代码
            ros_motor_cmds = []
            for i in range(30):
                motor_cmd = MotorCmd()
                motor_cmd.id = i  # 这里可能出现错误

                # 从字典中查找对应ID的电机
                motor_key = str(i)  # JSON中的键是字符串格式
                if motor_key in motor_cmd_dict:
                    # 从UDP数据获取电机信息
                    jetson_motor = motor_cmd_dict[motor_key]
                    if isinstance(jetson_motor, dict):
                        motor_cmd.mode = jetson_motor.get('mode', 0)
                        motor_cmd.q = jetson_motor.get('q', 0.0)
                        motor_cmd.dq = jetson_motor.get('dq', 0.0)
                        motor_cmd.tau = jetson_motor.get('tau', 0.0)
                        motor_cmd.kp = jetson_motor.get('kp', 0.0)
                        motor_cmd.kd = jetson_motor.get('kd', 0.0)
                        print(f"✅ Motor {i}: q={motor_cmd.q:.4f}, dq={motor_cmd.dq:.3f}, tau={motor_cmd.tau:.3f}, mode={motor_cmd.mode}, kp={motor_cmd.kp:.1f}, kd={motor_cmd.kd:.1f}")
                    else:
                        print(f"电机{i}数据格式错误: {type(jetson_motor)}")
                        motor_cmd.mode = 0
                        motor_cmd.q = 0.0
                        motor_cmd.dq = 0.0
                        motor_cmd.tau = 0.0
                        motor_cmd.kp = 0.0
                        motor_cmd.kd = 0.0
                else:
                    # 该电机ID不在消息中，使用默认值
                    motor_cmd.mode = 0
                    motor_cmd.q = 0.0
                    motor_cmd.dq = 0.0
                    motor_cmd.tau = 0.0
                    motor_cmd.kp = 0.0
                    motor_cmd.kd = 0.0

                ros_motor_cmds.append(motor_cmd)

            lowcmd_msg.motor_cmd = ros_motor_cmds
            lowcmd_pub.publish(lowcmd_msg)
            print("✅ 成功发布 xixilowcmd/LowCmd 消息")

        except Exception as e:
            print(f"❌ 创建 xixilowcmd/LowCmd 消息失败: {e}")
            import traceback
            traceback.print_exc()

    except Exception as e:
        print(f"❌ 总体错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()