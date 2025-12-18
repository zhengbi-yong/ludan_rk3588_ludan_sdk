#!/usr/bin/env python3
"""
监听8888端口接收Jetson发送的LowCmd数据
"""

import socket
import json
import time
import signal
import sys

class LowCmdUDPListener:
    def __init__(self, port=8888):
        self.port = port
        self.running = True
        self.message_count = 0

        # 创建UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', port))
        self.socket.settimeout(1.0)  # 1秒超时

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        print(f"🎯 启动UDP监听器，端口: {port}")
        print(f"📍 绑定地址: 0.0.0.0:{port}")
        print("⏳ 等待Jetson发送数据...")
        print("🔊 按 Ctrl+C 停止监听")
        print("=" * 60)

    def signal_handler(self, signum, frame):
        print("\n🛑 接收到停止信号，正在关闭...")
        self.running = False

    def start(self):
        start_time = time.time()

        while self.running:
            try:
                # 接收数据
                data, addr = self.socket.recvfrom(4096)

                if data:
                    self.message_count += 1
                    current_time = time.time()
                    elapsed_time = current_time - start_time

                    try:
                        # 尝试解析JSON数据
                        message = json.loads(data.decode('utf-8'))

                        print(f"📨 消息 #{self.message_count}")
                        print(f"   来源: {addr[0]}:{addr[1]}")
                        print(f"   时间戳: {message.get('timestamp', 0):.3f}")
                        print(f"   序列号: {message.get('sequence', 0)}")
                        print(f"   类型: {message.get('type', 'unknown')}")

                        # 显示关节信息
                        positions = message.get('positions', {})
                        if positions:
                            print("   关节位置:")
                            for joint_id, pos_data in positions.items():
                                print(f"     关节{joint_id}: q={pos_data.get('q', 0):.3f}, "
                                      f"dq={pos_data.get('dq', 0):.3f}, "
                                      f"kp={pos_data.get('kp', 0):.1f}")

                        # 计算频率
                        if self.message_count > 0:
                            frequency = self.message_count / elapsed_time
                            print(f"   频率: {frequency:.1f} Hz")

                        print("-" * 40)

                    except json.JSONDecodeError:
                        print(f"📨 原始数据 #{self.message_count}")
                        print(f"   来源: {addr[0]}:{addr[1]}")
                        print(f"   长度: {len(data)} 字节")
                        print(f"   内容: {data[:100]}...")
                        print("-" * 40)

            except socket.timeout:
                # 定期显示状态
                if self.message_count == 0 and time.time() - start_time > 5:
                    elapsed = time.time() - start_time
                    print(f"⏳ 已等待 {elapsed:.0f} 秒，暂无数据...")

                    # 检查网络连接
                    print("💡 检查网络连接:")
                    print(f"   - 本机IP: {self.get_local_ip()}")
                    print(f"   - 监听端口: {self.port}")
                    print(f"   - 需要Jetson发送到: {self.get_local_ip()}:{self.port}")
                    print("   - 确认Jetson上的--rk3588_ip参数正确")
                    print()

                continue
            except Exception as e:
                print(f"❌ 错误: {e}")
                break

        # 显示统计信息
        self.show_statistics(start_time)

    def get_local_ip(self):
        """获取本机IP地址"""
        try:
            # 连接到外部地址来获取本机IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "未知"

    def show_statistics(self, start_time):
        total_time = time.time() - start_time
        print("=" * 60)
        print("📊 监听统计:")
        print(f"   总时间: {total_time:.1f} 秒")
        print(f"   接收消息: {self.message_count} 条")
        if total_time > 0:
            print(f"   平均频率: {self.message_count/total_time:.1f} Hz")
        print("🔚 监听器已停止")

def main():
    print("🚀 RK3588 LowCmd UDP监听器")
    print("🎯 专为接收Jetson发送的8888端口数据设计")
    print()

    # 检查端口
    import sys
    port = 8888
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("❌ 端口参数必须是数字")
            return 1

    listener = LowCmdUDPListener(port)
    listener.start()

    return 0

if __name__ == "__main__":
    exit(main())