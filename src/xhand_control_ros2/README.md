
# XHandControlROS2 User Guide

<p align="center">
  <a target="_blank" href="/README.zh-CN.md">中文</a>
</p>

## Introduction

**XHandControlROS2** is a ROS 2 package designed for EtherCAT communication with the XHand robotic hand. It provides comprehensive control interfaces via ROS 2 topics, services, and callback functions to control the hand’s joints and read sensor data.

## Installation

Ensure the following dependencies are installed:

- ROS 2 (supports ROS 2 Foxy)
```bash
sudo apt update && sudo apt install -y \
    cmake \
    g++ \
    libcurl4-openssl-dev \
    libssl-dev \
    nlohmann-json3-dev
```

### Create a Workspace
```bash
mkdir -p ~/ros2_ws/src
```

Clone the downloaded ROS2 package into the `ros2_ws/src` workspace and compile:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Nodes

### xhand_control_ros2 Node

This node is responsible for establishing a connection with the XHand device and managing the hand’s state and commands.

### Launching the Node

**启动前检查串口**：

```bash
ls /dev/ttyUSB*
sudo chmod 777 /dev/ttyUSB*
```

正常情况下应看到 `/dev/ttyUSB0`（左手）和 `/dev/ttyUSB1`（右手）。

Run the following command as the `root` user to start the node (`su` to switch to the `root` user; for first-time `su` usage, set a password using `sudo passwd`):

```bash
source install/setup.bash
ros2 launch xhand_control_ros2 xhand_control.launch.py
```

## Topics

### Published Topics

- **`/xhand_control/xhand_state`**: Publishes the joint states and sensor data of the XHand.
  - **Message Type**: `xhand_control_msgs::msg::XHandStateArray`
  - **XHandStateArray Message Content**:
    - `header`: Message header.
    - `hand_id[]`: IDs of the hands.
    - `hand_name[]`: Names of the hands.
    - `hand_states[]`: Joint states of each hand.
      - `name[]`: Names of the joints.
      - `position[]`: Positions of the joints.
      - `effort[]`: Joint torques.
      - `temperature[]`: Joint temperatures.
      - `error_code[]`: Joint error codes.
    - `sensor_states[]`: Sensor states of each hand.
      - `finger_sensor_states[]`: Sensor state of individual fingers.
        - `location`: Sensor location.
        - `calc_force[]`: Calculated force.
        - `raw_force[]`: Raw force data.
        - `calc_temperature[]`: Calculated temperature.
        - `raw_temperature[]`: Raw temperature data.

### View Message Example

```bash
ros2 topic echo /xhand_control/xhand_state
```

### Subscribed Topics

- **`/xhand_control/xhand_command`**: Subscribes to control commands for the XHand.
  - **Message Type**: `xhand_control_msgs::msg::XHandCommand`
  - **XHandCommand Message Content**:
    - `hand_id`: ID of the hand to control.
    - `name[]`: List of joint names to control.
    - `position[]`: Target positions for each joint.
    - `kp[]`, `kd[]`, `ki[]`: PID control parameters.
    - `effort_limit[]`: Torque limits.
    - `mode`: Control mode.

### Publish Message Example

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

## Services

### Provided Services

- **`/xhand_control/read_hand_info`**: Reads information from the XHand.
  - **Service Type**: `xhand_control_msgs::srv::ReadHandInfo`
  - **Request Fields**:
    - `hand_id`: ID of the hand to query.
    - `info_type`: Type of information to query (e.g., hand type, serial number, version).
  - **Response Fields**:
    - `success`: Whether the operation was successful.
    - `info`: Returned hand information.

- **`/xhand_control/set_hand_id`**: Sets the ID of the XHand.
  - **Service Type**: `xhand_control_msgs::srv::SetHandId`
  - **Request Fields**:
    - `current_id`: Current hand ID.
    - `new_id`: New ID to set.
  - **Response Fields**:
    - `success`: Whether the operation was successful. A node restart is required after a successful operation.

- **`/xhand_control/set_hand_name`**: Sets the name of the XHand.
  - **Service Type**: `xhand_control_msgs::srv::SetHandName`
  - **Request Fields**:
    - `hand_id`: ID of the hand to set.
    - `hand_name`: New name to set.
  - **Response Fields**:
    - `success`: Whether the operation was successful. A node restart is required after a successful operation.

- **`/xhand_control/reset_hand_sensor`**: Resets the sensors of the XHand.
  - **Service Type**: `xhand_control_msgs::srv::ResetSensor`
  - **Request Fields**:
    - `hand_id`: ID of the hand whose sensors to reset.
    - `sensor_id`: ID of the sensor to reset (17 to 21).
  - **Response Fields**:
    - `success`: Whether the operation was successful.

### Service Call Examples

```bash
ros2 service call /xhand_control/read_hand_info xhand_control_msgs/srv/ReadHandInfo '{hand_id: 0, info_type: 0}'
ros2 service call /xhand_control/set_hand_name xhand_control_msgs/srv/SetHandName '{hand_id: 0, hand_name: "test_hand"}'
ros2 service call /xhand_control/set_hand_id xhand_control_msgs/srv/SetHandId '{current_id: 1, new_id: 2}'
ros2 service call /xhand_control/reset_hand_sensor xhand_control_msgs/srv/ResetSensor '{hand_id: 0, sensor_id: 17}'
```

## FAQ

1. **Device not detected**  
   Ensure the EtherCAT device is connected properly and the interface name is configured correctly.

2. **Unable to publish or subscribe to topics**  
   Ensure the node has been started correctly, and the topic names match between the publisher and subscriber.

3. **Environment not configured**
   - **Issue**:
     - Unable to launch related ROS 2 packages.
     - Unable to publish topics or call services.
   - **Solution**: Run `source install/setup.bash` in the workspace.

## Fingure Joint Mode
- In the **FingerCommand_t** class, the **mode** parameter is used to control the mode of finger joints.
- The modes of finger joints and the meanings of the parameters are shown in the table below:

| Name   | Value   | Notes        |
|--------|:--------:|-------------|
| Passive Mode  | 0    |    |
| Position Control Mode  | 3    |    |
| Force Control Mode  | 5    |    |

## Important Statement
By using the product, the user shall be deemed to have read, understood and agreed to be bound by all provisions in the User Agreement. During the use, the user shall strictly abide by the provisions in the User Agreement and use the product reasonably and lawfully. Refer to [<a target="_blank" href="./USER_AGREEMENT.md">**USER_AGREEMENT**</a>] for the User Agreement.

---

> 本项目使用方式：左手 `/dev/ttyUSB0` (namespace `left_hand`)，右手 `/dev/ttyUSB1` (namespace `right_hand`)。详见顶层 [README](../README.md)。