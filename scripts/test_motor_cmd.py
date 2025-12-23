#!/usr/bin/env python3
"""
测试 MotorCmd 对象的属性
"""
import rclpy
from xixilowcmd.msg import MotorCmd, LowCmd

def main():
    print("🔍 测试 MotorCmd 对象属性")

    # 初始化ROS2
    rclpy.init()

    try:
        # 创建 MotorCmd 对象
        motor_cmd = MotorCmd()
        print(f"✅ 成功创建 MotorCmd 对象")

        # 测试所有属性
        print(f"📋 MotorCmd 属性测试:")

        # 测试 id 属性
        try:
            motor_cmd.id = 1
            print(f"   ✅ id: {motor_cmd.id} (类型: {type(motor_cmd.id)})")
        except AttributeError as e:
            print(f"   ❌ id 属性错误: {e}")

        # 测试 mode 属性
        try:
            motor_cmd.mode = 1
            print(f"   ✅ mode: {motor_cmd.mode} (类型: {type(motor_cmd.mode)})")
        except AttributeError as e:
            print(f"   ❌ mode 属性错误: {e}")

        # 测试 q 属性
        try:
            motor_cmd.q = 0.5
            print(f"   ✅ q: {motor_cmd.q} (类型: {type(motor_cmd.q)})")
        except AttributeError as e:
            print(f"   ❌ q 属性错误: {e}")

        # 测试 dq 属性
        try:
            motor_cmd.dq = 1.0
            print(f"   ✅ dq: {motor_cmd.dq} (类型: {type(motor_cmd.dq)})")
        except AttributeError as e:
            print(f"   ❌ dq 属性错误: {e}")

        # 测试 kp 属性
        try:
            motor_cmd.kp = 20.0
            print(f"   ✅ kp: {motor_cmd.kp} (类型: {type(motor_cmd.kp)})")
        except AttributeError as e:
            print(f"   ❌ kp 属性错误: {e}")

        # 测试 kd 属性
        try:
            motor_cmd.kd = 2.0
            print(f"   ✅ kd: {motor_cmd.kd} (类型: {type(motor_cmd.kd)})")
        except AttributeError as e:
            print(f"   ❌ kd 属性错误: {e}")

        # 测试 tau 属性
        try:
            motor_cmd.tau = 0.1
            print(f"   ✅ tau: {motor_cmd.tau} (类型: {type(motor_cmd.tau)})")
        except AttributeError as e:
            print(f"   ❌ tau 属性错误: {e}")

        # 测试创建 LowCmd 对象并添加 MotorCmd
        print(f"\n🔧 测试 LowCmd 对象:")
        lowcmd = LowCmd()
        print(f"   ✅ 成功创建 LowCmd 对象")

        # 创建 MotorCmd 数组
        motor_cmds = []
        for i in range(3):
            motor = MotorCmd()
            motor.id = i
            motor.mode = 1
            motor.q = float(i) * 0.1
            motor.dq = float(i) * 0.2
            motor.kp = 10.0 + float(i)
            motor.kd = 1.0 + float(i)
            motor.tau = 0.1 * float(i)
            motor_cmds.append(motor)

        lowcmd.motor_cmd = motor_cmds
        print(f"   ✅ 成功设置 LowCmd.motor_cmd (包含 {len(motor_cmds)} 个电机)")

        # 检查结果
        for i, motor in enumerate(lowcmd.motor_cmd):
            print(f"      Motor {motor.id}: q={motor.q:.2f}, dq={motor.dq:.2f}, tau={motor.tau:.2f}, mode={motor.mode}, kp={motor.kp:.1f}, kd={motor.kd:.1f}")

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()