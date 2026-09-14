"""astral_policy_inference launch: policy_node + optional policy_keyboard.

用法：
  ros2 launch astral_policy_inference policy_inference.launch.py \
      params_file:=<path> backend_type:=remote host:=192.168.x.x \
      camera_image_size:=480 keyboard:=true

backend_type = 传输（remote | inproc | stub）；model = 模型族（act | pi05 | ...），
默认回落 yaml。顶层 robot/schema 参数通过 `policy.robot.<key>` 形如
--ros-args -p arms:="['left']" 在 launch 里用 extra_args 传入（见 config yaml 注释）。

注意：模型 preprocessor 无 resize，`camera_image_size` 必须与模型输入一致
（本机 pickup_act_480 = 480）；launch 未传时回落 yaml 默认值 224。
`engine_mode` 默认 queue_async 即 30Hz（方案 A 后无需改）；仅时序融合 checkpoint
（`temporal_ensemble_coeff` 非空）才需 `engine_mode:=queue_sync`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params() -> str:
    return os.path.join(
        get_package_share_directory("astral_policy_inference"),
        "config",
        "policy_inference.yaml",
    )


def _node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    backend_type = LaunchConfiguration("backend_type").perform(context)
    model = LaunchConfiguration("model").perform(context)
    jpeg_transport = LaunchConfiguration("jpeg_transport").perform(context)
    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    checkpoint_dir = LaunchConfiguration("checkpoint_dir").perform(context)
    cmd_topic = LaunchConfiguration("cmd_topic").perform(context)
    camera_image_size = LaunchConfiguration("camera_image_size").perform(context)
    engine_mode = LaunchConfiguration("engine_mode").perform(context)
    temporal_ensemble_coeff = LaunchConfiguration("temporal_ensemble_coeff").perform(context)
    chunk_anchor_tol = LaunchConfiguration("chunk_anchor_tol").perform(context)
    chunk_anchor_blend = LaunchConfiguration("chunk_anchor_blend").perform(context)
    async_prefetch_ahead = LaunchConfiguration("async_prefetch_ahead").perform(context)
    metrics_log_file = LaunchConfiguration("metrics_log_file").perform(context)
    joint_stream_log_file = LaunchConfiguration("joint_stream_log_file").perform(context)
    parameters = [params_file]
    if backend_type:
        parameters.append(
            {
                "backend_type": backend_type,
                "host": host,
                "port": int(port),
                "checkpoint_dir": checkpoint_dir,
                "cmd_topic": cmd_topic,
            }
        )
    if model:
        parameters.append({"model": model})
    if jpeg_transport:
        parameters.append({"jpeg_transport": jpeg_transport.lower() in ("1", "true")})
    if camera_image_size:
        parameters.append({"camera_image_size": int(camera_image_size)})
    if engine_mode:
        parameters.append({"engine_mode": engine_mode})
    if temporal_ensemble_coeff:
        parameters.append({"temporal_ensemble_coeff": float(temporal_ensemble_coeff)})
    if chunk_anchor_tol:
        parameters.append({"chunk_anchor_tol": float(chunk_anchor_tol)})
    if chunk_anchor_blend:
        parameters.append({"chunk_anchor_blend": int(chunk_anchor_blend)})
    if async_prefetch_ahead:
        parameters.append({"async_prefetch_ahead": int(async_prefetch_ahead)})
    if metrics_log_file:
        parameters.append({"metrics_log_file": metrics_log_file})
    if joint_stream_log_file:
        parameters.append({"joint_stream_log_file": joint_stream_log_file})
    node = Node(
        package="astral_policy_inference",
        executable="policy_node",
        name="policy_node",
        output="screen",
        parameters=parameters,
    )
    kb = LaunchConfiguration("keyboard").perform(context).lower() in ("1", "true")
    actions = [node]
    if kb:
        actions.append(
            Node(
                package="astral_policy_inference",
                executable="policy_keyboard",
                name="policy_keyboard",
                output="screen",
                parameters=[{"cmd_topic": cmd_topic, "state_topic": LaunchConfiguration("state_topic")}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=_default_params()),
            DeclareLaunchArgument("backend_type", default_value=""),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument(
                "jpeg_transport", default_value="",
                description="true 上行 camera 槽位发 JPEG（载荷 ~7x 小）；false 发 RGB"),
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8000"),
            DeclareLaunchArgument("checkpoint_dir", default_value=""),
            DeclareLaunchArgument("camera_image_size", default_value=""),
            DeclareLaunchArgument("engine_mode", default_value=""),
            DeclareLaunchArgument(
                "temporal_ensemble_coeff", default_value="",
                description=">0 开启 ACT 时序融合（换 chunk 加权平均，消切换跳变；0=关）"),
            DeclareLaunchArgument(
                "chunk_anchor_tol", default_value="",
                description=">0 换 chunk 切换平滑（续播起点偏离旧 command >tol 时前 blend 行过渡；0=关）"),
            DeclareLaunchArgument(
                "chunk_anchor_blend", default_value="",
                description="切换平滑过渡行数（配合 chunk_anchor_tol；@30Hz 每行 33ms）"),
            DeclareLaunchArgument(
                "async_prefetch_ahead", default_value="",
                description="重规划/融合重叠行数（每 action_chunk−N 步融合一次；0=自动 chunk//2）"),
            DeclareLaunchArgument(
                "metrics_log_file", default_value="",
                description="非空则节点把 state JSON（延迟/引擎指标）追加写该文件并终端打印"),
            DeclareLaunchArgument(
                "joint_stream_log_file", default_value="",
                description="非空则节点把每次下发的关节指令流（30Hz JSON 行）追加写该文件"),
            DeclareLaunchArgument("keyboard", default_value="false"),
            DeclareLaunchArgument("cmd_topic", default_value="/policy_inference/cmd"),
            DeclareLaunchArgument("state_topic", default_value="/policy_inference/state"),
            OpaqueFunction(function=_node),
        ]
    )
