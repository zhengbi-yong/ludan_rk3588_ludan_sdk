#include <fstream>
#include <iostream>
#include <thread>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#include "wav.hpp"

// 音频文件路径定义
#define AUDIO_FILE_PATH "../example/g1/audio/test.wav"      // 测试音频文件路径
#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"                // ASR语音识别消息订阅主题
#define GROUP_IP "239.168.123.161"                         // 多播组IP地址
#define PORT 5555                                           // UDP端口号

// 录音参数定义
#define WAV_SECOND 5        // 录音时长（秒）
#define WAV_LEN (16000 * 2 * WAV_SECOND)  // 音频数据长度（16kHz采样率，16位，5秒）
#define CHUNK_SIZE 96000    // 音频流播放块大小（3秒的数据量）
int sock;  // 全局UDP套接字描述符

/**
 * @brief ASR（自动语音识别）消息处理回调函数
 * @param msg 接收到的语音识别结果消息
 *
 * 当机器人完成语音识别后，会通过DDS发布识别结果到"rt/audio_msg"主题
 * 此函数作为订阅者的回调函数，处理识别到的文本内容
 */
void asr_handler(const void *msg) {
  std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
  std::cout << "Topic:\"rt/audio_msg\" recv: " << resMsg->data() << std::endl;
}

/**
 * @brief 获取用于多播的本地IP地址
 * @return 匹配192.168.123.*网段的本地IP地址字符串
 *
 * 遍历所有网络接口，查找符合特定网段（192.168.123.*）的IP地址
 * 这个IP地址将用于加入多播组接收音频流数据
 */
std::string get_local_ip_for_multicast() {
  struct ifaddrs *ifaddr, *ifa;
  char host[NI_MAXHOST];
  std::string result = "";

  // 获取所有网络接口地址
  getifaddrs(&ifaddr);

  // 遍历所有网络接口
  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    // 跳过无效接口和非IPv4地址
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;

    // 将IP地址转换为字符串格式
    getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST,
                NULL, 0, NI_NUMERICHOST);
    std::string ip(host);

    // 查找匹配192.168.123.*网段的IP地址
    if (ip.find("192.168.123.") == 0) {
      result = ip;
      break;
    }
  }

  // 释放网络接口地址链表内存
  freeifaddrs(ifaddr);
  return result;
}

/**
 * @brief 音频录制线程函数
 *
 * 通过UDP多播接收来自机器人的实时音频流数据
 * 将接收到的PCM音频数据保存为WAV文件
 */
void thread_mic(void) {
  // 创建UDP套接字
  sock = socket(AF_INET, SOCK_DGRAM, 0);

  // 配置本地绑定地址
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(PORT);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (sockaddr *)&local_addr, sizeof(local_addr));

  // 配置多播组成员信息
  ip_mreq mreq{};
  inet_pton(AF_INET, GROUP_IP, &mreq.imr_multiaddr);  // 设置多播组地址

  // 获取本地IP并设置多播接口
  std::string local_ip = get_local_ip_for_multicast();
  std::cout << "local ip: " << local_ip << std::endl;
  mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());

  // 加入多播组
  setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

  // 准备音频数据接收缓冲区
  int total_bytes = 0;
  std::vector<int16_t> pcm_data;
  pcm_data.reserve(WAV_LEN / 2);  // 预分配空间
  std::cout << "start record!" << std::endl;

  // 接收音频数据循环，直到达到预设的录音长度
  while (total_bytes < WAV_LEN) {
    char buffer[2048];  // UDP数据接收缓冲区
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);

    if (len > 0) {
      // 计算接收到的音频样本数量（每个样本2字节）
      size_t sample_count = len / 2;
      const int16_t *samples = reinterpret_cast<const int16_t *>(buffer);

      // 将音频样本数据添加到PCM数据向量中
      pcm_data.insert(pcm_data.end(), samples, samples + sample_count);
      total_bytes += len;
    }
  }

  // 将PCM数据写入WAV文件
  WriteWave("record.wav", 16000, pcm_data.data(), pcm_data.size(), 1);
  std::cout << "record finish! save to record.wav " << std::endl;
}

/**
 * @brief 主函数 - G1音频客户端示例程序
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组，argv[1]为网络接口名（如eth0）
 * @return 程序退出状态码
 *
 * 演示G1机器人的完整音频功能：
 * 1. 音量控制和查询
 * 2. 文本转语音(TTS)
 * 3. 音频文件播放
 * 4. LED控制
 * 5. 实时音频录制
 * 6. 语音识别(ASR)
 */
int main(int argc, char const *argv[]) {
  // 检查命令行参数
  if (argc < 2) {
    std::cout << "Usage: audio_client_example [NetWorkInterface(eth0)]"
              << std::endl;
    exit(0);
  }

  int32_t ret;

  /*
   * 初始化DDS通信通道工厂
   * 参数0: 使用默认域ID
   * 参数argv[1]: 网络接口名（如eth0）
   */
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  // 创建并初始化G1音频客户端
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);  // 设置API调用超时时间为10秒

  /* ============ ASR语音识别消息订阅示例 ============ */
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel(asr_handler);  // 设置消息处理回调函数

  /* ============ 音量控制示例 ============ */
  uint8_t volume;

  // 获取当前音量
  ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;

  // 设置音量为100%
  ret = client.SetVolume(100);
  std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

  /* ============ TTS文本转语音示例 ============ */

  // 中文TTS - 自动播放
  ret = client.TtsMaker("你好。我是宇树科技的机器人。例程启动成功",
                        0);  // 参数0: 自动播放
  std::cout << "TtsMaker API ret:" << ret << std::endl;
  unitree::common::Sleep(5);  // 等待中文语音播放完成

  // 英文TTS - 自动播放
  ret = client.TtsMaker(
      "Hello. I'm a robot from Unitree Robotics. The example has started "
      "successfully. ",
      1);  // 参数1: 英文TTS
  std::cout << "TtsMaker API ret:" << ret << std::endl;
  unitree::common::Sleep(8);  // 等待英文语音播放完成

  /* ============ 音频文件播放示例 ============ */
  int32_t sample_rate = -1;
  int8_t num_channels = 0;
  bool filestate = false;

  // 读取WAV音频文件
  std::vector<uint8_t> pcm =
      ReadWave(AUDIO_FILE_PATH, &sample_rate, &num_channels, &filestate);

  // 输出音频文件信息
  std::cout << "wav file sample_rate = " << sample_rate
            << " num_channels =  " << std::to_string(num_channels)
            << " filestate =" << filestate << "filesize = " << pcm.size()
            << std::endl;

  // 检查音频文件格式：要求16kHz采样率、单声道
  if (filestate && sample_rate == 16000 && num_channels == 1) {
    size_t total_size = pcm.size();
    size_t offset = 0;

    // 生成唯一的流ID（基于当前时间戳）
    std::string stream_id =
        std::to_string(unitree::common::GetCurrentTimeMillisecond());

    // 分块播放音频数据（流式播放）
    while (offset < total_size) {
      size_t remaining = total_size - offset;
      size_t current_chunk_size =
          std::min(static_cast<size_t>(CHUNK_SIZE), remaining);

      // 提取当前音频块数据
      std::vector<uint8_t> chunk(pcm.begin() + offset,
                                 pcm.begin() + offset + current_chunk_size);

      // 播放当前音频块
      client.PlayStream("example", stream_id, chunk);
      unitree::common::Sleep(1);  // 块间延迟
      std::cout << "Playing size: " << offset << std::endl;
      offset += current_chunk_size;
    }

    // 停止播放流
    ret = client.PlayStop(stream_id);

  } else {
    std::cout << "audio file format error, please check!" << std::endl;
  }

  /* ============ LED控制示例 ============ */
  client.LedControl(0, 255, 0);    // 绿色LED
  unitree::common::Sleep(1);
  client.LedControl(0, 0, 0);      // 关闭LED
  unitree::common::Sleep(1);
  client.LedControl(0, 0, 255);    // 蓝色LED

  std::cout << "AudioClient api test finish , asr start..." << std::endl;

  /* ============ 启动音频录制线程 ============ */
  std::thread mic_t(thread_mic);

  // 主线程等待ASR语音识别消息
  while (1) {
    sleep(1);  // 等待语音识别消息
  }

  mic_t.join();  // 等待录音线程结束
  return 0;
}