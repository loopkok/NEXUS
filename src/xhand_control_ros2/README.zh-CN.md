# XHandControlROS2 使用说明

<p align="center">
  <a target="_blank" href="/README.md">English</a>
</p>

## 介绍
XHandControlROS2 是一个 ROS2 包，用于与 XHand 机械手进行 EtherCAT 通信，控制机械手的关节和读取传感器数据。该包通过 ROS2 话题、服务和回调函数提供对 XHand 的全面控制接口。

## 安装
确保已经安装了以下依赖：

- ROS2 (支持 ROS2 foxy)
```bash
sudo apt update && sudo apt install -y \
    cmake \
    g++ \
    libcurl4-openssl-dev \
    libssl-dev \
    nlohmann-json3-dev
```
创建工作空间
```bash
mkdir -p ~/ros2_ws/src
```

克隆下载的 ROS2 包到你的 ros2_ws/src 工作空间并进行编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## 节点

### xhand_control_ros2 节点
该节点负责与 XHand 设备建立连接，处理手的状态和指令。

### 启动节点
使用 `root用户` 通过运行以下命令启动节点：（ `su` 进入 `root用户`，首次使用 `su` 需要设置密码 `sudo passwd` ）

```bash
source install/setup.bash
ros2 launch xhand_control_ros2 xhand_control.launch.py
```
## 话题

### 发布的话题

- `/xhand_control/xhand_state`：发布 XHand 的关节状态及传感器数据。
  - 消息类型：`xhand_control_msgs::msg::XHandStateArray`
  - XHandStateArray 消息内容：
    - `header`：消息头。
    - `hand_id[]`：手 ID。
    - `hand_name[]`：手名称。
    - `hand_states[]`：每个手的关节状态。
      - `name[]`：关节名称。
      - `position[]`：关节位置。
      - `effort[]`：关节力矩。
      - `temperature[]`：关节温度。
      - `error_code[]`：关节错误代码。
    - `sensor_states[]`：每个手的传感器状态。
      - `finger_sensor_states[]`：单个手的传感器状态。
        - `location`：传感器位置。
        - `calc_force[]`：传感器合力
        - `raw_force[]`：传感器原始力数据。
        - `calc_temperature[]`：传感器扭矩。
        - `raw_temperature[]`：传感器原始扭矩数据。
### 查看消息示例

```bash
ros2 topic echo /xhand_control/xhand_state
```

### 订阅的话题

- `/xhand_control/xhand_command`：订阅用户发送的 XHand 控制命令。
  - 消息类型：`xhand_control_msgs::msg::XHandCommand`
  - XHandCommand 消息内容：
    - `hand_id`：指定要控制的手ID。
    - `name[]`：指定要控制的关节名称列表。
    - `position[]`：各个关节目标位置。
    - `kp[]`, `kd[]`, `ki[]`：PID 控制参数。
    - `effort_limit[]`：力矩限制。
    - `mode`：控制模式。

### 发布消息示例

```bash
ros2 topic pub --once /xhand_control/xhand_command xhand_control_interfaces/msg/XHandCommand 'hand_id: 0
name: ['thumb_bend_joint', 'index_bend_joint']
position: [0.5, 0.18]
kp: [100, 100]
ki: [0, 0]
kd: [0, 0]
effort_limit: [350, 350]
mode: 3'
```

## 服务

### 提供的服务

- `/xhand_control/read_hand_info`：读取 XHand 的信息。
  - 服务类型：`xhand_control_msgs::srv::ReadHandInfo`
  - 请求字段：
    - `hand_id`：要查询的手的 ID。
    - `info_type`：查询类型 (如手的类型、序列号、版本等)。
  - 响应字段：
    - `success`：操作是否成功。
    - `info`：返回的手信息。

- `/xhand_control/set_hand_id`：设置 XHand 的 ID。
  - 服务类型：`xhand_control_msgs::srv::SetHandId`
  - 请求字段：
    - `current_id`：当前的手 ID。
    - `new_id`：要设置的新 ID。
  - 响应字段：
    - `success`：操作是否成功,成功后需要重新启动节点。

- `/xhand_control/set_hand_name`：设置 XHand 的名称。
  - 服务类型：`xhand_control_msgs::srv::SetHandName`
  - 请求字段：
    - `hand_id`：要设置的手的 ID。
    - `hand_name`：要设置的新名称。
  - 响应字段：
    - `success`：操作是否成功,成功后需要重新启动节点。

- `/xhand_control/reset_hand_sensor`：重置 XHand 的传感器。
  - 服务类型：`xhand_control_msgs::srv::ResetSensor`
  - 请求字段：
    - `hand_id`：要重置的手的 ID。
    - `sensor_id`：要重置的传感器 ID (17 到 21)。
  - 响应字段：
    - `success`：操作是否成功。

### 服务调用示例

```bash
ros2 service call /xhand_control/read_hand_info xhand_control_msgs/srv/ReadHandInfo '{hand_id: 0, info_type: 0}'
ros2 service call /xhand_control/set_hand_name xhand_control_msgs/srv/SetHandName '{hand_id: 0, hand_name: "test_hand"}'
ros2 service call /xhand_control/set_hand_id xhand_control_msgs/srv/SetHandId '{current_id: 1, new_id: 2}'
ros2 service call /xhand_control/reset_hand_sensor xhand_control_msgs/srv/ResetSensor '{hand_id: 0, sensor_id: 17}'
```


## 常见问题
1. **无法检测到设备**  
   请确认 EtherCAT 设备连接正常，并且正确配置了接口名。

2. **无法发布或订阅消息**  
   确保你已正确启动节点，并且话题命名与实际订阅或发布的一致。

3. **未配置环境**
  - **问题现象：**
    - 无法启动相关ROS2包
    - 无法使用命令 pub 话题或者 call 服务
  - **解决办法：** 需要在工作空间执行 `source install/setup.bash`

## 手指关节模式
- 在手指类 **FingerCommand_t** 中，通过修改参数 **mode** 来控制手指关节的模式。
- 手指关节模式以及参数含义如下表所示： 

| 名称   | 数值   | 备注        |
|--------|:--------:|-------------|
| 无力模式  | 0    |    |
| 位控模式  | 3    |    |
| 力控模式  | 5    |    |

## 重要声明
用户使用本产品即视为已充分阅读、理解并同意接受用户协议的全部条款约束。在使用过程中，用户应严格遵守该协议约定，合理合法地使用本产品。用户协议详见【<a target="_blank" href="./用户协议.md">**用户协议**</a>】。