# LHandPro ROS2 示例工程

这是一个基于 ROS2 的 LHandPro 示例工程，支持 EtherCAT 和 CANFD 两种通讯方式。

## 系统要求

- Ubuntu 22.04 或更高版本
- ROS2 Humble 或更高版本
- C++17 兼容编译器
- CMake 3.16 或更高版本

## 安装

1. 克隆仓库：

   ```bash
   git clone <repository_url>
   ```

2. 构建项目：

   ```bash
   cd lhandpro_ws
   colcon build
   ```

3. 激活工作空间：

   ```bash
   source install/setup.bash
   ```

4. 配置环境

   ```bash
   # 复制so到系统库目录,方便后续使用
   sudo cp src/lhandpro_service/thirdparty/lib/libLHandProLib.so /usr/local/lib/ 
   # 刷新动态链接器缓存
   sudo ldconfig       
   # 赋予原始套接字 + 网络管理权限
   sudo setcap cap_net_raw,cap_net_admin+ep ./install/lhandpro_service/lib/lhandpro_service/lhandpro_service
   ```

   打开lhandpro_ws/src/ros2_lhandpro.conf, 编辑conf文件
   ```bash
   # ros2_lhandpro.conf 文件内容, 修改为本机的实际路径
   /home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_service/lib
   /home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_interfaces/lib
   /home/plf/Project/RosProject/lhandpro_ws/install/lhandpro_description/lib
   /home/plf/Project/RosProject/lhandpro_ws/install/sequence_demo_cpp/lib
   /home/plf/Project/RosProject/lhandpro_ws/install/sequence_demo_py/lib
   ```
   
   配置执行环境
   ```bash
   # 复制conf文件到系统目录
   sudo cp src/ros2_lhandpro.conf /etc/ld.so.conf.d/
   # 刷新动态链接器缓存
   sudo ldconfig       
   ```

## 使用方法

### 启动服务

   ```bash
   ros2 launch lhandpro_service lhandpro.launch.py
   ```

### 启动示例控制

   ```bash
   # 启动后则会自动执行准备好的动作序列
   ros2 launch sequence_demo_py sequence.launch.py
   # 或者执行C++版本的示例程序
   ros2 launch sequence_demo_cpp sequence.launch.py
   ```

### 启动rviz2的仿真显示

   ```bash
   ros2 launch lhandpro_description display_lhandpro.launch.py
   # 执行该脚本, 会启动robot_state_publisher节点
   # 启动lhandpro_state_publisher节点, 用来通过服务监控角度变化, 并通过/joint_states发布给rviz2更新显示
   # 启动rviz2节点, 并加载准备好的rviz配置config
   # 加载完成顺利的话则能在rviz2中看到灵巧手的模型
   # 此时执行4步骤中的控制示例, 则能看到实际灵巧手在运动, 并且rviz2中的模型也在同时运动
   ```


### 通讯方式切换

在 `src/lhandpro_service/src/hand_control_service.hpp` 文件中修改 `COMMUNICATION_MODE` 宏：

- `#define COMMUNICATION_MODE 0` - 使用 EtherCAT 通讯
- `#define COMMUNICATION_MODE 1` - 使用 CANFD 通讯

### 灵巧手型号切换

在 `src/lhandpro_service/src/hand_control_service.hpp` 文件中修改 `HAND_TYPE` 宏：

- `#define HAND_TYPE lhplib::LAC_DOF_6` - DH116灵巧手
- `#define HAND_TYPE lhplib::LAC_DOF_6_S` - DH116S灵巧手



### 服务列表

| 服务名称 | 功能描述 |
|---------|----------|
| `set_enable` | 设置电机使能状态 |
| `get_now_alarm` | 获取当前报警状态 |
| `set_position` | 设置目标位置 |
| `get_position` | 获取目标位置 |
| `get_now_angle` | 获取当前角度 |
| `get_now_position` | 获取当前位置 |
| `get_now_position_velocity` | 获取当前速度 |
| `get_now_current` | 获取当前电流 |
| `get_position_velocity` | 获取目标速度 |
| `set_position_velocity` | 设置目标速度 |
| `get_max_current` | 获取最大电流 |
| `set_max_current` | 设置最大电流 |
| `get_control_mode` | 获取控制模式 |
| `set_control_mode` | 设置控制模式 |
| `home_motors` | 电机回零 |
| `move_motors` | 驱动电机运动 |

## 示例调用

### 设置电机使能

```bash
ros2 service call /set_enable lhandpro_interfaces/srv/SetEnable "{joint_id: 0, enable: 1}"
```

### 设置目标位置

```bash
ros2 service call /set_position lhandpro_interfaces/srv/SetPosition "{joint_id: 1, position: 5000}"
```

### 驱动电机运动

```bash
ros2 service call /move_motors lhandpro_interfaces/srv/MoveMotors "{joint_id: 0}"
```

## 维护者

- Sorrowfeng

## 许可证

本项目使用 Apache-2.0 许可证，详情见 LICENSE 文件。