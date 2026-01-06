#include "CANFDMaster.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <poll.h>
#include <time.h>

CANFDMaster::CANFDMaster()
    : dev_index_(-1), is_connected_(false), receive_thread_running_(false), socket_fd_(-1) {}

// 设置CAN接口的波特率和模式
bool CANFDMaster::setupCANInterface(const std::string& ifname, int nomBaud, int dataBaud) {
    // 重置USB CAN设备
    system("modprobe -r gs_usb");
    system("modprobe gs_usb");
    system("echo 'a8fa 8598' | sudo tee /sys/bus/usb/drivers/gs_usb/new_id");
    
    // 设置CAN接口参数
    std::string cmd = "ip link set " + ifname + " down";
    system(cmd.c_str());
    
    cmd = "ip link set " + ifname + " type can bitrate " + std::to_string(nomBaud * 1000) + 
          " dbitrate " + std::to_string(dataBaud * 1000) + " fd on loopback off listen-only off";
    system(cmd.c_str());
    
    cmd = "ip link set " + ifname + " up";
    system(cmd.c_str());
    
    return true;
}

CANFDMaster::~CANFDMaster() {
  if (is_connected_) {
    disconnect();
  }
}

std::vector<std::string> CANFDMaster::scanDevices() {
  std::vector<std::string> devices;
  DIR *dir;
  struct dirent *ent;

  // 扫描/sys/class/net目录查找CAN接口
  if ((dir = opendir("/sys/class/net")) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      std::string ifname = ent->d_name;
      // 检查是否为CAN接口
      if (ifname.substr(0, 3) == "can") {
        devices.push_back(ifname);
      }
    }
    closedir(dir);
  }

  return devices;
}

bool CANFDMaster::connect(int deviceIndex, int nomBaud, int dataBaud) {
  if (is_connected_) {
    disconnect();
  }

  // 扫描可用的CAN接口
  std::vector<std::string> devices = scanDevices();
  if (deviceIndex < 0 || deviceIndex >= devices.size()) {
    return false;
  }

  std::string ifname = devices[deviceIndex];
  current_interface_ = ifname;

  // 设置CAN接口参数
  if (!setupCANInterface(ifname, nomBaud, dataBaud)) {
    std::cerr << "Failed to setup CAN interface" << std::endl;
    return false;
  }

  // 创建CAN原始套接字
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    perror("socket");
    return false;
  }

  // 设置CAN_FD_FRAME选项以支持CAN FD
  int enable_canfd = 1;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
    perror("setsockopt CAN_RAW_FD_FRAMES");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 绑定到CAN接口
  struct ifreq ifr;
  strcpy(ifr.ifr_name, ifname.c_str());
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl SIOCGIFINDEX");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  dev_index_ = deviceIndex;
  nom_baud_ = nomBaud;
  data_baud_ = dataBaud;

  is_connected_ = true;

  // 启动接收线程
  receive_thread_running_ = true;
  receive_thread_ = std::thread(&CANFDMaster::receiveThreadFunc, this);

  resetLostFrames();
  return true;
}

bool CANFDMaster::disconnect() {
  if (!is_connected_) {
    return true;
  }

  // 停止接收线程
  receive_thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }

  dev_index_ = -1;
  is_connected_ = false;
  current_interface_.clear();
  resetLostFrames();
  return true;
}

bool CANFDMaster::attemptReconnect() {
  disconnect();
  return connect(dev_index_, nom_baud_, data_baud_);
}

bool CANFDMaster::sendData(unsigned int id, const unsigned char* data,
                           unsigned int size, int externflag) {
  return sendData(static_cast<uint32_t>(id),
                  std::vector<uint8_t>(data, data + size), externflag);
}

bool CANFDMaster::sendData(uint32_t id, const std::vector<uint8_t>& data,
                           int externflag) {
  if (!is_connected_ || socket_fd_ < 0) {
    return false;
  }

  // 使用canfd_frame结构体发送CAN FD数据
  struct canfd_frame frame;
  memset(&frame, 0, sizeof(frame));

  // 设置帧ID
  if (externflag) {
    frame.can_id = id | CAN_EFF_FLAG;
  } else {
    frame.can_id = id;
  }

  // 设置DLC和数据
  frame.len = 64;  // 固定64字节
  frame.flags = CANFD_BRS;  // 使用比特率切换

  // 填充数据
  size_t copySize = std::min(data.size(), size_t(64));
  memcpy(frame.data, data.data(), copySize);

  // 发送数据
  int ret = write(socket_fd_, &frame, sizeof(frame));
  bool success = (ret == sizeof(frame));
  if (!success) {
    perror("write");
    lost_frames_++;
  }
  return success;
}



void CANFDMaster::setReceiveCallback(ReceiveCallback callback) {
  receive_callback_ = callback;
}

void CANFDMaster::receiveThreadFunc() {
  while (receive_thread_running_) {
    if (!is_connected_ || socket_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 使用canfd_frame结构体接收CAN FD数据
    struct canfd_frame frame;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;  // 50ms超时

    // 设置套接字超时
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);

    int ret = select(socket_fd_ + 1, &readfds, NULL, NULL, &timeout);
    if (ret > 0 && FD_ISSET(socket_fd_, &readfds)) {
      ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
      if (nbytes == sizeof(frame)) {
        // 处理接收到的帧
        if (receive_callback_) {
          // 提取ID（移除标志位）
          uint32_t id = frame.can_id & CAN_EFF_MASK;
          if (!(frame.can_id & CAN_EFF_FLAG)) {
            id = frame.can_id & CAN_SFF_MASK;
          }

          // 构造数据向量
          int dataLength = frame.len;
          std::vector<uint8_t> data(dataLength);
          memcpy(data.data(), frame.data, dataLength);

          // 获取时间戳
          uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();

          // 调用回调函数
          receive_callback_(id, data, timestamp);
        }
      } else {
        lost_frames_++;
      }
    } else if (ret < 0) {
      lost_frames_++;
    }

    // 短暂休眠，避免过度占用CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool CANFDMaster::isConnected() const {
  return is_connected_;
}

int CANFDMaster::getCurrentDeviceIndex() const {
  return dev_index_;
}

uint64_t CANFDMaster::getLostFrames() const {
  return lost_frames_;
}

void CANFDMaster::resetLostFrames() {
  lost_frames_ = 0;
}