#ifndef CANFDMASTER_H
#define CANFDMASTER_H
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

// 回调函数类型定义
using ReceiveCallback = std::function<void(
    uint32_t id, const std::vector<uint8_t>& data, uint64_t timestamp)>;

class CANFDMaster {
 public:
  CANFDMaster();
  ~CANFDMaster();

  // 扫描可用设备
  std::vector<std::string> scanDevices();

  // 连接设备
  bool connect(int deviceIndex, int nomBaud, int dataBaud);

  // 断开连接
  bool disconnect();

  bool attemptReconnect();

  // 发送数据（默认64字节）
  bool sendData(unsigned int id, const unsigned char* data, unsigned int size,
                int externflag = 0);
  bool sendData(uint32_t id, const std::vector<uint8_t>& data,
                int externflag = 0);

  // 设置接收数据回调函数
  void setReceiveCallback(ReceiveCallback callback);

  // 检查是否已连接
  bool isConnected() const;

  // 获取当前设备索引
  int getCurrentDeviceIndex() const;

  // 获取丢帧统计
  uint64_t getLostFrames() const;

  // 重置丢帧统计
  void resetLostFrames();

 private:
  // 设置CAN接口参数
  bool setupCANInterface(const std::string& ifname, int nomBaud, int dataBaud);
  
  // 接收线程函数
  void receiveThreadFunc();

 private:
  int dev_index_;                             // 当前设备索引
  int nom_baud_;                              // 当前仲裁段波特率
  int data_baud_;                             // 当前数据段波特率
  std::atomic<bool> is_connected_;            // 连接状态
  std::atomic<bool> receive_thread_running_;  // 接收线程运行状态
  std::thread receive_thread_;                // 接收线程

  ReceiveCallback receive_callback_;  // 接收回调函数

  uint64_t lost_frames_{0};  // 丢帧计数器
  int socket_fd_;            // Socket文件描述符
  std::string current_interface_;  // 当前使用的CAN接口名称
};

#endif  // CANFDMASTER_H
