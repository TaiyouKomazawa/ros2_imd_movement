import os, math

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    ld = LaunchDescription()

    l_id_arg = DeclareLaunchArgument(
        'l_mcp2210_id', default_value=TextSubstitution(text='0001310808')
    )
    l_cs_arg = DeclareLaunchArgument(
        'l_mcp2210_cs', default_value=TextSubstitution(text='1')
    )
    r_id_arg = DeclareLaunchArgument(
        'r_mcp2210_id', default_value=TextSubstitution(text='0001312251')
    )
    r_cs_arg = DeclareLaunchArgument(
        'r_mcp2210_cs', default_value=TextSubstitution(text='0')
    )

    cmd_vel_arg = DeclareLaunchArgument(
        'command_topic', default_value=TextSubstitution(text='/cmd_vel')
    )

    rotation_radius_arg = DeclareLaunchArgument(
        'rotation_radius', default_value=TextSubstitution(text='0.07980')
    )
    rr = LaunchConfiguration('rotation_radius')

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value=TextSubstitution(text='0.02540')
    )
    wr = LaunchConfiguration('wheel_radius')

    wheel_params = os.path.join(
        get_package_share_directory('ros2_imd_movement'),
        'config',
        '4wheels_omni_params.yaml'
    )

    imd1_motor_node = Node(
        package='ros2_imd_driver',
        namespace='imd1',
        executable='imd_node',
        parameters=[
            wheel_params,
            {'mcp2210_serial_number' : ['i', LaunchConfiguration('l_mcp2210_id')]},
            {'mcp2210_cs_pin' : LaunchConfiguration('l_mcp2210_cs')},
        ],
        output='screen',
        remappings=[
            ('m1/command', '/LF/command'),
            ('m2/command', '/LB/command'),
            ('m1/feedback', '/LF/feedback'),
            ('m2/feedback', '/LB/feedback')
        ]
    )
    imd2_motor_node = Node(
        package='ros2_imd_driver',
        namespace='imd2',
        executable='imd_node',
        parameters=[
            wheel_params,
            {'mcp2210_serial_number' : ['i', LaunchConfiguration('r_mcp2210_id')]},
            {'mcp2210_cs_pin' : LaunchConfiguration('r_mcp2210_cs')},
        ],
        output='screen',
        remappings=[
            ('m1/command', '/RF/command'),
            ('m2/command', '/RB/command'),
            ('m1/feedback', '/RF/feedback'),
            ('m2/feedback', '/RB/feedback')
        ]
    )

    left_flont_wheel_node = Node(
        package='ros2_imd_movement',
        namespace='LF',
        executable='vector_wheel',
        parameters=[wheel_params,
            {'wheel_cmd_topic' : '/cmd_vel'},
            {'wheel.fixed_angle' : -math.pi*1/4},
            {'wheel.footprint' : [[rr], [rr]]},
            {'wheel.radius' : wr}],
    )
    left_back_wheel_node = Node(
        package='ros2_imd_movement',
        namespace='LB',
        executable='vector_wheel',
        parameters=[wheel_params,
            {'wheel_cmd_topic' : '/cmd_vel'},
            {'wheel.fixed_angle' : math.pi*1/4},
            {'wheel.footprint' : [['-',rr], [rr]]},
            {'wheel.radius' : wr}]
    )
    right_back_wheel_node = Node(
        package='ros2_imd_movement',
        namespace='RB',
        executable='vector_wheel',
        parameters=[wheel_params,
            {'wheel_cmd_topic' : '/cmd_vel'},
            {'wheel.fixed_angle' : math.pi*3/4},
            {'wheel.footprint' : [['-',rr], ['-',rr]]},
            {'wheel.radius' : wr}]
    )
    right_flont_wheel_node = Node(
        package='ros2_imd_movement',
        namespace='RF',
        executable='vector_wheel',
        parameters=[wheel_params,
            {'wheel_cmd_topic' : '/cmd_vel'},
            {'wheel.fixed_angle' : -math.pi*3/4},
            {'wheel.footprint' : [[rr], ['-',rr]]},
            {'wheel.radius' : wr}]
    )

    topic_merger_node = Node(
        package='ros2_imd_movement',
        executable='topic_synchronizer',
        parameters=[
            {'wheel_odom_topics':['LF/wheel/feedback','LB/wheel/feedback','RB/wheel/feedback','RF/wheel/feedback']},
            {'calc_matrix_once': True},
            {'sync_hz' : 100.0}
        ]
    )

    ld.add_action(l_id_arg)
    ld.add_action(l_cs_arg)
    ld.add_action(r_id_arg)
    ld.add_action(r_cs_arg)
    ld.add_action(cmd_vel_arg)
    ld.add_action(rotation_radius_arg)
    ld.add_action(wheel_radius_arg)

    ld.add_action(imd1_motor_node)
    ld.add_action(imd2_motor_node)
    ld.add_action(left_flont_wheel_node)
    ld.add_action(left_back_wheel_node)
    ld.add_action(right_back_wheel_node)
    ld.add_action(right_flont_wheel_node)

    ld.add_action(topic_merger_node)
    return ld