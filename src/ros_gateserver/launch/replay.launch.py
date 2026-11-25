import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rosbag2_py import Info


def report_playback_exit(event, context):
    if context.is_shutdown:
        return []

    if event.returncode != 0:
        # 事件处理异常交给 Launch 统一收尾，并使启动命令返回失败状态。
        raise RuntimeError(f'Bag playback failed with exit code {event.returncode}.')

    # 播放结束后保留网关，便于观察 fresh 变为 false；由用户 Ctrl+C 关闭。
    return [LogInfo(msg='Playback finished. Gateway remains running; press Ctrl+C to stop.')]


def launch_replay(context):
    bag_path = Path(LaunchConfiguration('bag_path').perform(context)).expanduser().resolve()
    if not bag_path.is_dir() or not (bag_path / 'metadata.yaml').is_file():
        raise RuntimeError(f'Expected a finalized bag directory with metadata.yaml: {bag_path}')

    rate = float(LaunchConfiguration('rate').perform(context))
    start_delay = float(LaunchConfiguration('start_delay').perform(context))
    clock_rate = float(LaunchConfiguration('clock_rate').perform(context))
    sim_time = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
    loop = IfCondition(LaunchConfiguration('loop')).evaluate(context)
    if not math.isfinite(rate) or rate <= 0:
        raise RuntimeError('Playback rate must be finite and greater than zero.')
    if not math.isfinite(start_delay) or start_delay < 0:
        raise RuntimeError('Playback start_delay must be finite and nonnegative.')
    if not math.isfinite(clock_rate) or clock_rate <= 0:
        raise RuntimeError('clock_rate must be finite and positive.')
    if loop and not sim_time:
        raise RuntimeError('loop requires use_sim_time:=true so TF and caches reset on rewind.')

    package_share = Path(get_package_share_directory('ros_gateserver'))
    # 回放只使用 bag 中的安装关系，不能让演示 TF 掩盖录制缺失。
    gateway_arguments = {'mock': 'false', 'demo_tf': 'false'}
    for name in (
        'ws_host',
        'ws_port',
        'ws_format',
        'grpc_enabled',
        'grpc_host',
        'grpc_port',
        'self_check',
        'image_topic',
        'scan_topic',
        'odom_topic',
        'target_frame',
        'use_sim_time',
        'tf_wait_timeout_sec',
        'image_reliability',
        'scan_reliability',
        'odom_reliability',
        'image_quality_enabled',
        'image_quality_max_side',
        'image_quality_max_pixels',
        'image_dark_threshold',
        'image_bright_threshold',
        'image_contrast_threshold',
        'image_sharpness_threshold',
    ):
        gateway_arguments[name] = LaunchConfiguration(name).perform(context)

    metadata = Info().read_metadata(str(bag_path), 'mcap')
    has_static_tf = any(
        item.topic_metadata.name == '/tf_static' and item.message_count > 0
        for item in metadata.topics_with_message_count
    )
    topics = [gateway_arguments[name] for name in ('image_topic', 'scan_topic', 'odom_topic')]
    notices = []
    if has_static_tf:
        topics.append('/tf_static')
    else:
        notices.append(
            LogInfo(
                msg=(
                    'Bag has no /tf_static; sensor summaries remain available, '
                    'but coordinate conversion needs an external TF source.'
                ),
            )
        )

    has_dynamic_tf = any(
        item.topic_metadata.name == '/tf' and item.message_count > 0
        for item in metadata.topics_with_message_count
    )
    if has_dynamic_tf:
        topics.append('/tf')

    # 复用现有网关入口，并关闭其中的模拟发布者。
    gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / 'launch' / 'mvp.launch.py')),
        launch_arguments=gateway_arguments.items(),
    )
    command = [
        'ros2',
        'bag',
        'play',
        str(bag_path),
        '--rate',
        str(rate),
        '--delay',
        str(start_delay),
        '--disable-keyboard-controls',
        '--qos-profile-overrides-path',
        str(package_share / 'config' / 'rosbag_qos.yaml'),
        '--topics',
        *topics,
    ]
    # 同一个选项同时控制播放器 /clock 和网关 ROS 时钟，避免两边配置不一致。
    if sim_time:
        command.extend(['--clock', str(clock_rate)])
    if loop:
        command.append('--loop')
    player = ExecuteProcess(
        cmd=command,
        output='screen',
    )

    # 延迟播放给网关启动和 DDS 发现留出时间；best-effort QoS 不保证逐条送达。
    # 先注册退出事件，确保能处理播放器在启动阶段发生的错误。
    playback_exit = RegisterEventHandler(
        OnProcessExit(target_action=player, on_exit=report_playback_exit)
    )
    return [*notices, playback_exit, gateway, player]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('bag_path', description='Finalized bag directory to replay.'),
            DeclareLaunchArgument('rate', default_value='1.0'),
            DeclareLaunchArgument('start_delay', default_value='2.0'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('clock_rate', default_value='50.0'),
            DeclareLaunchArgument('loop', default_value='false'),
            DeclareLaunchArgument('tf_wait_timeout_sec', default_value='0.0'),
            DeclareLaunchArgument('image_reliability', default_value='best_effort'),
            DeclareLaunchArgument('scan_reliability', default_value='best_effort'),
            DeclareLaunchArgument('odom_reliability', default_value='best_effort'),
            DeclareLaunchArgument('image_quality_enabled', default_value='false'),
            DeclareLaunchArgument('image_quality_max_side', default_value='320'),
            DeclareLaunchArgument('image_quality_max_pixels', default_value='8294400'),
            DeclareLaunchArgument('image_dark_threshold', default_value='30.0'),
            DeclareLaunchArgument('image_bright_threshold', default_value='225.0'),
            DeclareLaunchArgument('image_contrast_threshold', default_value='10.0'),
            DeclareLaunchArgument('image_sharpness_threshold', default_value='50.0'),
            DeclareLaunchArgument('ws_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('ws_port', default_value='8765'),
            DeclareLaunchArgument('ws_format', default_value='json'),
            DeclareLaunchArgument('grpc_enabled', default_value='false'),
            DeclareLaunchArgument('grpc_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('grpc_port', default_value='50051'),
            DeclareLaunchArgument('self_check', default_value='false'),
            DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('odom_topic', default_value='/odom'),
            DeclareLaunchArgument('target_frame', default_value='base_link'),
            OpaqueFunction(function=launch_replay),
        ]
    )
