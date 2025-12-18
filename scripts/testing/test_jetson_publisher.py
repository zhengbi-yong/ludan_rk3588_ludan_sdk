#!/usr/bin/env python3
"""
Jetson Publisher Test Script
测试Jetson上的/lowcmd话题发布功能

Author: Claude Code Assistant
Date: 2025-12-17
"""

import rospy
import time
import threading
import signal
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JetsonPublisherTester:
    """Jetson发布器测试器"""

    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('jetson_publisher_tester', anonymous=True)

        # 统计数据
        self.lowcmd_count = 0
        self.joint_state_count = 0
        self.debug_count = 0
        self.test_start_time = time.time()
        self.last_lowcmd_time = 0
        self.last_joint_state_time = 0

        # 话题数据存储
        self.latest_lowcmd = None
        self.latest_joint_state = None

        # 设置订阅者
        self.setup_subscribers()

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        rospy.loginfo("="*60)
        rospy.loginfo("    Jetson Publisher Test Suite")
        rospy.loginfo("="*60)

    def setup_subscribers(self):
        """设置话题订阅者"""
        # 订阅/lowcmd话题
        self.lowcmd_sub = rospy.Subscriber(
            '/lowcmd',
            PoseStamped,
            self.lowcmd_callback,
            queue_size=10
        )

        # 订阅/joint_states话题
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=10
        )

        # 订阅/debug_info话题
        self.debug_sub = rospy.Subscriber(
            '/debug_info',
            Header,
            self.debug_callback,
            queue_size=10
        )

        rospy.loginfo("✅ Subscribed to topics:")
        rospy.loginfo("   - /lowcmd")
        rospy.loginfo("   - /joint_states")
        rospy.loginfo("   - /debug_info")

    def lowcmd_callback(self, msg):
        """LowCmd消息回调"""
        self.lowcmd_count += 1
        self.last_lowcmd_time = time.time()
        self.latest_lowcmd = msg

        if self.lowcmd_count <= 5:  # 显示前5条消息
            rospy.loginfo(f"📡 LowCmd #{self.lowcmd_count}:")
            rospy.loginfo(f"   Frame: {msg.header.frame_id}")
            rospy.loginfo(f"   Position: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
            rospy.loginfo(f"   Orientation: x={msg.pose.orientation.x:.4f}, y={msg.pose.orientation.y:.4f}, z={msg.pose.orientation.z:.4f}")

    def joint_state_callback(self, msg):
        """关节状态消息回调"""
        self.joint_state_count += 1
        self.last_joint_state_time = time.time()
        self.latest_joint_state = msg

        if self.joint_state_count <= 5:  # 显示前5条消息
            rospy.loginfo(f"🦴 JointState #{self.joint_state_count}:")
            rospy.loginfo(f"   Joint count: {len(msg.name)}")
            if len(msg.name) > 0:
                rospy.loginfo(f"   Names: {', '.join(msg.name[:4])}")
                rospy.loginfo(f"   Positions: {[f'{p:.3f}' for p in msg.position[:4]]}")

    def debug_callback(self, msg):
        """调试信息回调"""
        self.debug_count += 1
        if self.debug_count <= 3:  # 显示前3条消息
            rospy.loginfo(f"🔍 Debug #{self.debug_count}: {msg.frame_id}")

    def monitor_topics(self):
        """监控话题状态"""
        def monitor_thread():
            rate = rospy.Rate(1)  # 1Hz

            while not rospy.is_shutdown():
                self.print_status()
                rate.sleep()

        monitor_thread = threading.Thread(target=monitor_thread)
        monitor_thread.daemon = True
        monitor_thread.start()

    def print_status(self):
        """打印状态信息"""
        elapsed_time = time.time() - self.test_start_time

        # 计算频率
        lowcmd_rate = self.lowcmd_count / elapsed_time if elapsed_time > 0 else 0
        joint_state_rate = self.joint_state_count / elapsed_time if elapsed_time > 0 else 0

        # 检查话题活跃度
        current_time = time.time()
        lowcmd_active = (current_time - self.last_lowcmd_time) < 2.0 if self.last_lowcmd_time > 0 else False
        joint_state_active = (current_time - self.last_joint_state_time) < 2.0 if self.last_joint_state_time > 0 else False

        print(f"\n📊 Test Status (Runtime: {elapsed_time:.1f}s)")
        print(f"   /lowcmd: {self.lowcmd_count} msgs ({lowcmd_rate:.1f} Hz) {'✅' if lowcmd_active else '❌'}")
        print(f"   /joint_states: {self.joint_state_count} msgs ({joint_state_rate:.1f} Hz) {'✅' if joint_state_active else '❌'}")
        print(f"   /debug_info: {self.debug_count} msgs")

        # 显示最新数据
        if self.latest_lowcmd:
            print(f"   Latest position: ({self.latest_lowcmd.pose.position.x:.3f}, {self.latest_lowcmd.pose.position.y:.3f}, {self.latest_lowcmd.pose.position.z:.3f})")

        if self.latest_joint_state and len(self.latest_joint_state.position) >= 4:
            ankle_positions = [f'{p:.3f}' for p in self.latest_joint_state.position[:4]]
            print(f"   Ankle positions: [{', '.join(ankle_positions)}]")

        print("-" * 50)

    def test_topic_existence(self):
        """测试话题是否存在"""
        rospy.loginfo("🔍 Checking topic existence...")

        # 等待话题列表更新
        time.sleep(2)

        # 获取话题列表
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]

        expected_topics = ['/lowcmd', '/joint_states', '/debug_info']
        found_topics = []

        for topic in expected_topics:
            if topic in topic_names:
                found_topics.append(topic)
                rospy.loginfo(f"✅ Found topic: {topic}")
            else:
                rospy.logwarn(f"❌ Missing topic: {topic}")

        rospy.loginfo(f"Topics found: {len(found_topics)}/{len(expected_topics)}")

        return len(found_topics) == len(expected_topics)

    def test_message_frequency(self):
        """测试消息频率"""
        rospy.loginfo("📏 Testing message frequency...")

        # 等待消息累积
        test_duration = 10.0  # 10秒测试
        rospy.loginfo(f"Collecting messages for {test_duration} seconds...")

        start_time = time.time()
        start_lowcmd_count = self.lowcmd_count
        start_joint_state_count = self.joint_state_count

        time.sleep(test_duration)

        end_time = time.time()
        end_lowcmd_count = self.lowcmd_count
        end_joint_state_count = self.joint_state_count

        # 计算频率
        actual_duration = end_time - start_time
        lowcmd_freq = (end_lowcmd_count - start_lowcmd_count) / actual_duration
        joint_state_freq = (end_joint_state_count - start_joint_state_count) / actual_duration

        rospy.loginfo(f"📊 Frequency Test Results:")
        rospy.loginfo(f"   /lowcmd: {lowcmd_freq:.1f} Hz (expected: ~50 Hz)")
        rospy.loginfo(f"   /joint_states: {joint_state_freq:.1f} Hz (expected: ~50 Hz)")

        # 频率评估
        lowcmd_ok = 10 <= lowcmd_freq <= 100  # 允许10-100Hz范围
        joint_state_ok = 10 <= joint_state_freq <= 100

        rospy.loginfo(f"   /lowcmd frequency: {'✅ OK' if lowcmd_ok else '❌ Too low/high'}")
        rospy.loginfo(f"   /joint_states frequency: {'✅ OK' if joint_state_ok else '❌ Too low/high'}")

        return lowcmd_ok and joint_state_ok

    def test_data_consistency(self):
        """测试数据一致性"""
        rospy.loginfo("🔗 Testing data consistency...")

        if not self.latest_lowcmd or not self.latest_joint_state:
            rospy.logwarn("❌ No data available for consistency test")
            return False

        # 检查时间戳
        time_diff = abs(self.latest_lowcmd.header.stamp.to_sec() - self.latest_joint_state.header.stamp.to_sec())
        time_ok = time_diff < 0.1  # 100ms内

        rospy.loginfo(f"   Timestamp difference: {time_diff:.3f}s {'✅' if time_ok else '❌'}")

        # 检查数据范围
        pos_ok = True
        for i, pos in enumerate(self.latest_joint_state.position):
            if abs(pos) > 3.14159:  # 超过π弧度
                rospy.logwarn(f"   Joint {i} position out of range: {pos}")
                pos_ok = False

        rospy.loginfo(f"   Position ranges: {'✅ OK' if pos_ok else '❌ Out of range'}")

        return time_ok and pos_ok

    def run_tests(self):
        """运行所有测试"""
        rospy.loginfo("🧪 Starting publisher tests...")
        rospy.loginfo("Please ensure the Jetson publisher is running!")
        rospy.loginfo("Run: ./start_jetson_lowcmd.sh")
        rospy.loginfo("")

        # 启动监控
        self.monitor_topics()

        # 等待发布器启动
        rospy.loginfo("⏳ Waiting for publisher to start...")
        time.sleep(5)

        # 运行测试
        tests = [
            ("Topic Existence", self.test_topic_existence),
            ("Message Frequency", self.test_message_frequency),
            ("Data Consistency", self.test_data_consistency)
        ]

        results = []

        for test_name, test_func in tests:
            rospy.loginfo(f"\n🧪 Running test: {test_name}")
            try:
                result = test_func()
                results.append((test_name, result))
                rospy.loginfo(f"✅ Test '{test_name}' completed: {'PASS' if result else 'FAIL'}")
            except Exception as e:
                rospy.logerr(f"❌ Test '{test_name}' failed with error: {e}")
                results.append((test_name, False))

        # 显示测试结果
        self.print_test_results(results)

        # 持续监控
        rospy.loginfo("\n👁️  Continuous monitoring... Press Ctrl+C to stop")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def print_test_results(self, results):
        """打印测试结果"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("           TEST RESULTS SUMMARY")
        rospy.loginfo("="*60)

        passed = 0
        total = len(results)

        for test_name, result in results:
            status = "✅ PASS" if result else "❌ FAIL"
            rospy.loginfo(f"  {test_name:<20} : {status}")
            if result:
                passed += 1

        rospy.loginfo("-"*60)
        rospy.loginfo(f"  Total Tests: {passed}/{total}")
        rospy.loginfo(f"  Success Rate: {passed/total*100:.1f}%")

        if passed == total:
            rospy.loginfo("🎉 All tests PASSED! Publisher is working correctly.")
        else:
            rospy.logwarn("⚠️ Some tests FAILED. Please check the publisher configuration.")

        rospy.loginfo("="*60)

    def signal_handler(self, sig, frame):
        """信号处理函数"""
        rospy.loginfo("\n🛑 Stopping test...")
        self.print_final_summary()
        sys.exit(0)

    def print_final_summary(self):
        """打印最终总结"""
        elapsed_time = time.time() - self.test_start_time

        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("              FINAL TEST SUMMARY")
        rospy.loginfo("="*60)
        rospy.loginfo(f"  Test Duration: {elapsed_time:.1f} seconds")
        rospy.loginfo(f"  Total Messages Received:")
        rospy.loginfo(f"    /lowcmd: {self.lowcmd_count}")
        rospy.loginfo(f"    /joint_states: {self.joint_state_count}")
        rospy.loginfo(f"    /debug_info: {self.debug_count}")

        if elapsed_time > 0:
            rospy.loginfo(f"  Average Rates:")
            rospy.loginfo(f"    /lowcmd: {self.lowcmd_count/elapsed_time:.1f} Hz")
            rospy.loginfo(f"    /joint_states: {self.joint_state_count/elapsed_time:.1f} Hz")

        rospy.loginfo("="*60)

def main():
    """主函数"""
    try:
        tester = JetsonPublisherTester()
        tester.run_tests()
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()