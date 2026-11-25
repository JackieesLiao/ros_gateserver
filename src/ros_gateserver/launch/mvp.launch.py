import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_demo_tf(context):
    if (
        IfCondition(LaunchConfiguration('demo_motion')).evaluate(context)
        and not IfCondition(LaunchConfiguration('mock')).evaluate(context)
    ):
        raise RuntimeError('demo_motion requires mock:=true.')
    if not IfCondition(LaunchConfiguration('demo_tf')).evaluate(context):
        return []

    arguments = []
    for component in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        value = float(LaunchConfiguration(f'laser_{component}').perform(context))
        if not math.isfinite(value):
            raise RuntimeError(f'laser_{component} must be finite.')
        arguments.extend([f'--{component}', str(value)])

    # 演示安装关系由独立发布器提供；真实机器人已有 TF 时不要开启此选项。
    arguments.extend(['--frame-id', 'base_link', '--child-frame-id', 'laser_link'])
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='demo_laser_tf',
            output='screen',
            arguments=arguments,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('mock', default_value='true'),
            DeclareLaunchArgument('ws_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('ws_port', default_value='8765'),
            DeclareLaunchArgument('ws_format', default_value='json'),
            DeclareLaunchArgument('grpc_enabled', default_value='false'),
            DeclareLaunchArgument('grpc_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('grpc_port', default_value='50051'),
            DeclareLaunchArgument('self_check', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('demo_motion', default_value='false'),
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
            DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('odom_topic', default_value='/odom'),
            DeclareLaunchArgument('target_frame', default_value='base_link'),
            DeclareLaunchArgument(
                'demo_tf',
                default_value='false',
                description='Publish the fixed base_link -> laser_link demo transform.',
            ),
            DeclareLaunchArgument('laser_x', default_value='0.2'),
            DeclareLaunchArgument('laser_y', default_value='0.0'),
            DeclareLaunchArgument('laser_z', default_value='0.0'),
            DeclareLaunchArgument('laser_roll', default_value='0.0'),
            DeclareLaunchArgument('laser_pitch', default_value='0.0'),
            DeclareLaunchArgument('laser_yaw', default_value='0.0'),
            OpaqueFunction(function=launch_demo_tf),
            Node(
                package='ros_gateserver',
                executable='sensor_check_server',
                output='screen',
                condition=IfCondition(LaunchConfiguration('self_check')),
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package='ros_gateserver',
                executable='mock_sensors',
                output='screen',
                condition=IfCondition(LaunchConfiguration('mock')),
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool,
                        ),
                        'publish_odom_tf': ParameterValue(
                            LaunchConfiguration('demo_motion'), value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package='ros_gateserver',
                executable='gateway',
                output='screen',
                parameters=[
                    {
                        'ws_host': LaunchConfiguration('ws_host'),
                        'grpc_enabled': ParameterValue(
                            LaunchConfiguration('grpc_enabled'), value_type=bool,
                        ),
                        'grpc_host': ParameterValue(
                            LaunchConfiguration('grpc_host'), value_type=str,
                        ),
                        'grpc_port': ParameterValue(
                            LaunchConfiguration('grpc_port'), value_type=int,
                        ),
                        'image_quality_enabled': ParameterValue(
                            LaunchConfiguration('image_quality_enabled'), value_type=bool,
                        ),
                        'image_quality_max_side': ParameterValue(
                            LaunchConfiguration('image_quality_max_side'), value_type=int,
                        ),
                        'image_quality_max_pixels': ParameterValue(
                            LaunchConfiguration('image_quality_max_pixels'), value_type=int,
                        ),
                        'image_dark_threshold': ParameterValue(
                            LaunchConfiguration('image_dark_threshold'), value_type=float,
                        ),
                        'image_bright_threshold': ParameterValue(
                            LaunchConfiguration('image_bright_threshold'), value_type=float,
                        ),
                        'image_contrast_threshold': ParameterValue(
                            LaunchConfiguration('image_contrast_threshold'), value_type=float,
                        ),
                        'image_sharpness_threshold': ParameterValue(
                            LaunchConfiguration('image_sharpness_threshold'), value_type=float,
                        ),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool,
                        ),
                        'tf_wait_timeout_sec': ParameterValue(
                            LaunchConfiguration('tf_wait_timeout_sec'), value_type=float,
                        ),
                        'image_reliability': ParameterValue(
                            LaunchConfiguration('image_reliability'), value_type=str,
                        ),
                        'scan_reliability': ParameterValue(
                            LaunchConfiguration('scan_reliability'), value_type=str,
                        ),
                        'odom_reliability': ParameterValue(
                            LaunchConfiguration('odom_reliability'), value_type=str,
                        ),
                        'ws_format': ParameterValue(
                            LaunchConfiguration('ws_format'),
                            value_type=str,
                        ),
                        'ws_port': ParameterValue(
                            LaunchConfiguration('ws_port'),
                            value_type=int,
                        ),
                        'image_topic': LaunchConfiguration('image_topic'),
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'odom_topic': LaunchConfiguration('odom_topic'),
                        'target_frame': ParameterValue(
                            LaunchConfiguration('target_frame'),
                            value_type=str,
                        ),
                    },
                ],
            ),
        ]
    )
