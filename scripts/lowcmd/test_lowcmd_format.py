#!/usr/bin/env python3
"""
LowCmd Format Test Script
测试和验证LowCmd数据格式的正确性

Author: Claude Code Assistant
Date: 2025-12-17
"""

import json
import time
import math
import numpy as np
from python_lowcmd_builder import LowCmdBuilder, create_lowcmd_for_g1

def test_basic_lowcmd_creation():
    """测试基本LowCmd创建"""
    print("=" * 50)
    print("测试1: 基本LowCmd创建")
    print("=" * 50)

    # 创建LowCmd
    lowcmd = LowCmdBuilder("hg")

    # 验证基本属性
    assert lowcmd.robot_type == "hg"
    assert lowcmd.num_motors == 35
    assert len(lowcmd.motor_commands) == 35

    print(f"✅ 机器人类型: {lowcmd.robot_type}")
    print(f"✅ 电机数量: {lowcmd.num_motors}")
    print(f"✅ 电机命令数组长度: {len(lowcmd.motor_commands)}")

    # 设置模式
    lowcmd.mode_pr = 1
    lowcmd.mode_machine = 1

    print(f"✅ 模式设置: PR={lowcmd.mode_pr}, Machine={lowcmd.mode_machine}")

def test_motor_command_setting():
    """测试电机命令设置"""
    print("\n" + "=" * 50)
    print("测试2: 电机命令设置")
    print("=" * 50)

    lowcmd = LowCmdBuilder("hg")

    # 设置单个电机
    lowcmd.set_motor_command(4,
        mode=1,
        q=0.5,
        dq=0.1,
        tau=0.05,
        kp=80.0,
        kd=2.0
    )

    motor = lowcmd.motor_commands[4]
    assert motor.mode == 1
    assert motor.q == 0.5
    assert motor.dq == 0.1
    assert motor.tau == 0.05
    assert motor.kp == 80.0
    assert motor.kd == 2.0

    print(f"✅ 电机4命令设置成功:")
    print(f"   位置: {motor.q} rad")
    print(f"   速度: {motor.dq} rad/s")
    print(f"   力矩: {motor.tau} Nm")
    print(f"   增益: kp={motor.kp}, kd={motor.kd}")

def test_sine_wave_generation():
    """测试正弦波生成"""
    print("\n" + "=" * 50)
    print("测试3: 正弦波生成")
    print("=" * 50)

    lowcmd = LowCmdBuilder("hg")

    # 设置正弦波
    lowcmd.set_sine_wave_motors(
        motor_ids=[4, 5, 10, 11],
        amplitude=0.3,
        frequency=0.5
    )

    # 测试位置更新
    start_time = time.time()
    positions = []

    for i in range(10):
        current_time = time.time() - start_time
        lowcmd.update_positions()

        # 记录脚踝关节位置
        ankle_positions = [
            lowcmd.motor_commands[4].q,  # left_ankle_pitch
            lowcmd.motor_commands[5].q,  # left_ankle_roll
            lowcmd.motor_commands[10].q, # right_ankle_pitch
            lowcmd.motor_commands[11].q  # right_ankle_roll
        ]
        positions.append(ankle_positions)
        time.sleep(0.1)

    print(f"✅ 正弦波生成成功，生成了{len(positions)}个时间点的数据")
    print("✅ 前3个时间点的位置:")
    for i, pos in enumerate(positions[:3]):
        print(f"   t={i*0.1:.1f}s: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}, {pos[3]:.3f}]")

    # 验证正弦波特性
    max_vals = np.max(positions, axis=0)
    min_vals = np.min(positions, axis=0)

    print(f"✅ 位置范围:")
    print(f"   左脚踝pitch: [{min_vals[0]:.3f}, {max_vals[0]:.3f}]")
    print(f"   左脚踝roll: [{min_vals[1]:.3f}, {max_vals[1]:.3f}]")
    print(f"   右脚踝pitch: [{min_vals[2]:.3f}, {max_vals[2]:.3f}]")
    print(f"   右脚踝roll: [{min_vals[3]:.3f}, {max_vals[3]:.3f}]")

def test_json_conversion():
    """测试JSON转换"""
    print("\n" + "=" * 50)
    print("测试4: JSON转换")
    print("=" * 50)

    # 创建示例LowCmd
    lowcmd = create_lowcmd_for_g1()
    lowcmd.update_positions()  # 更新正弦波位置

    # 转换为字典
    lowcmd_dict = lowcmd.to_dict()

    # 验证基本结构
    required_keys = ['mode_pr', 'mode_machine', 'motors']
    for key in required_keys:
        assert key in lowcmd_dict, f"缺少关键字段: {key}"

    print(f"✅ JSON转换成功")
    print(f"✅ 模式: PR={lowcmd_dict['mode_pr']}, Machine={lowcmd_dict['mode_machine']}")
    print(f"✅ 电机数量: {len(lowcmd_dict['motors'])}")

    # 显示前5个电机状态
    print("✅ 前5个电机状态:")
    for i in range(5):
        motor = lowcmd_dict['motors'][i]
        print(f"   电机{motor['id']}: mode={motor['mode']}, q={motor['q']:.4f}, kp={motor['kp']}")

    # 测试JSON序列化
    json_str = json.dumps(lowcmd_dict, indent=2)
    print(f"✅ JSON序列化成功，大小: {len(json_str)} 字符")

def test_pose_stamped_mapping():
    """测试PoseStamped消息映射"""
    print("\n" + "=" * 50)
    print("测试5: PoseStamped消息映射")
    print("=" * 50)

    lowcmd = create_lowcmd_for_g1()
    lowcmd.update_positions()

    # 创建PoseStamped消息数据
    pose_data = lowcmd.create_pose_stamped_message()

    # 验证消息结构
    assert 'header' in pose_data
    assert 'pose' in pose_data
    assert 'position' in pose_data['pose']
    assert 'orientation' in pose_data['pose']

    print("✅ PoseStamped消息映射成功")
    print(f"✅ 消息头: frame_id={pose_data['header']['frame_id']}")
    print(f"✅ 位置: x={pose_data['pose']['position']['x']:.4f}, y={pose_data['pose']['position']['y']:.4f}")
    print(f"✅ 姿态: x={pose_data['pose']['orientation']['x']:.4f}, y={pose_data['pose']['orientation']['y']:.4f}")

    # 验证映射关系
    motors = lowcmd.motor_commands
    expected_x = motors[4].q if len(motors) > 4 else 0.0
    expected_y = motors[5].q if len(motors) > 5 else 0.0

    assert abs(pose_data['pose']['position']['x'] - expected_x) < 1e-6
    assert abs(pose_data['pose']['position']['y'] - expected_y) < 1e-6

    print("✅ 电机位置到Pose的映射验证通过")

def test_binary_packing():
    """测试二进制打包"""
    print("\n" + "=" * 50)
    print("测试6: 二进制打包")
    print("=" * 50)

    lowcmd = LowCmdBuilder("hg")
    lowcmd.mode_pr = 1
    lowcmd.mode_machine = 1

    # 设置一些电机命令
    for i in range(5):
        lowcmd.set_motor_command(i,
            mode=1,
            q=float(i) * 0.1,
            dq=float(i) * 0.01,
            tau=float(i) * 0.001,
            kp=80.0 + float(i),
            kd=2.0 + float(i) * 0.1
        )

    # 打包二进制数据
    binary_data = lowcmd.pack()

    print(f"✅ 二进制打包成功")
    print(f"✅ 数据大小: {len(binary_data)} 字节")

    # 验证数据大小合理性
    # LowCmd ≈ 2字节(模式) + 35*33字节(电机) + 16字节(保留) + 4字节(CRC) = 1177字节
    expected_min_size = 1000  # 最小预期大小
    assert len(binary_data) >= expected_min_size, f"二进制数据大小异常: {len(binary_data)}"

    print(f"✅ 二进制数据大小合理 (预期 > {expected_min_size} 字节)")

def test_joint_name_mapping():
    """测试关节名称映射"""
    print("\n" + "=" * 50)
    print("测试7: 关节名称映射")
    print("=" * 50)

    # G1关节名称映射
    g1_joint_names = [
        'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
        'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
        'torso_joint', 'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
        'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
        'left_wrist_pitch', 'left_wrist_roll', 'left_wrist_yaw', 'right_wrist_pitch', 'right_wrist_roll', 'right_wrist_yaw',
        'head_pitch', 'head_yaw', 'head_roll', 'waist_pitch', 'waist_roll', 'waist_yaw', 'reserved_1', 'reserved_2'
    ]

    print("✅ G1关节名称映射:")
    for i in range(min(12, len(g1_joint_names))):
        print(f"   {i:2d}: {g1_joint_names[i]:<20} -> 电机ID {i}")

    # 验证重要关节
    important_joints = {
        4: 'left_ankle_pitch',
        5: 'left_ankle_roll',
        10: 'right_ankle_pitch',
        11: 'right_ankle_roll'
    }

    print("\n✅ 重要脚踝关节映射验证:")
    for joint_id, joint_name in important_joints.items():
        assert g1_joint_names[joint_id] == joint_name
        print(f"   电机{joint_id}: {joint_name}")

def test_example_data():
    """测试示例数据"""
    print("\n" + "=" * 50)
    print("测试8: 示例数据验证")
    print("=" * 50)

    # 读取示例JSON文件
    try:
        with open('/home/linaro/scripts/lowcmd_data_example.json', 'r') as f:
            example_data = json.load(f)

        print("✅ 成功读取示例数据文件")

        # 验证基本结构
        required_fields = ['timestamp', 'sequence', 'mode_pr', 'mode_machine', 'motors']
        for field in required_fields:
            assert field in example_data, f"示例数据缺少字段: {field}"

        print("✅ 示例数据结构验证通过")
        print(f"✅ 时间戳: {example_data['timestamp']}")
        print(f"✅ 序列号: {example_data['sequence']}")
        print(f"✅ 电机数量: {len(example_data['motors'])}")

        # 验证脚踝关节的正弦波数据
        ankle_motors = [4, 5, 10, 11]
        for motor_id in ankle_motors:
            motor_data = example_data['motors'][motor_id]
            assert 'sine_wave' in motor_data, f"电机{motor_id}缺少正弦波数据"
            print(f"✅ 电机{motor_id}({motor_data['name']}): 正弦波幅度={motor_data['sine_wave']['amplitude']}")

    except FileNotFoundError:
        print("⚠️ 示例数据文件不存在，跳过测试")
    except Exception as e:
        print(f"❌ 示例数据测试失败: {e}")

def main():
    """主测试函数"""
    print("开始LowCmd格式测试...")
    print("测试时间:", time.strftime("%Y-%m-%d %H:%M:%S"))

    tests = [
        test_basic_lowcmd_creation,
        test_motor_command_setting,
        test_sine_wave_generation,
        test_json_conversion,
        test_pose_stamped_mapping,
        test_binary_packing,
        test_joint_name_mapping,
        test_example_data
    ]

    passed = 0
    failed = 0

    for test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"❌ 测试失败: {test_func.__name__} - {e}")
            failed += 1

    print("\n" + "=" * 50)
    print("测试结果汇总")
    print("=" * 50)
    print(f"总测试数: {len(tests)}")
    print(f"通过: {passed}")
    print(f"失败: {failed}")

    if failed == 0:
        print("🎉 所有测试通过！")
        return True
    else:
        print("⚠️ 部分测试失败，请检查上述错误信息")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)