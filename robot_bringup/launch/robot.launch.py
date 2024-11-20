import datetime
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, 
    Shutdown, SetEnvironmentVariable, IncludeLaunchDescription
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, 
    PathJoinSubstitution, AndSubstitution, NotSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('namespace', default_value='ur'),
        DeclareLaunchArgument('record', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5',
            choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 'ur10e', 'ur16e', 'ur20', 'ur30']
        ),
        DeclareLaunchArgument('safety_limits', default_value='false'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15'),
        DeclareLaunchArgument('safety_k_position', default_value='20'),
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution([FindPackageShare('robot_bringup'), 'urdf', 'ur.urdf.xacro'])
        ),
        DeclareLaunchArgument('tf_prefix', default_value='""'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    use_rviz = LaunchConfiguration('use_rviz')
    ur_type = LaunchConfiguration('ur_type')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    description_file = LaunchConfiguration('description_file')
    tf_prefix = LaunchConfiguration('tf_prefix')

    package_path = get_package_share_directory('robot_bringup')
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_path = os.path.join('/ros2_ws/src/records/', timestamp)
    rosbag_path = os.path.join(log_path, 'rosbag')
    robot_controllers = os.path.join(
        package_path,
        'config',
        'ur.yaml'
    )
    
    controller_manager_timeout = ['--controller-manager-timeout', '30']
    controller_manager_node_name = ['--controller-manager', 'controller_manager']

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', description_file, ' ',
        'safety_limits:=', safety_limits, ' ',
        'safety_pos_margin:=', safety_pos_margin, ' ',
        'safety_k_position:=', safety_k_position, ' ',
        'name:=', 'ur', ' ',
        'ur_type:=', ur_type, ' ',
        'tf_prefix:=', tf_prefix, ' ',
        'yaml_path:=', robot_controllers
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
        arguments=['-name', 'robot', '-x', '0.0', '-y', '0.0', '-z', '0.0',
                  '-Y', '0.0', '-topic', 'robot_description']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', ['-r v 4 empty.sdf'])],
        condition=IfCondition(use_sim_time)
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['joint_state_broadcaster'] + controller_manager_node_name + 
                 controller_manager_timeout
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['arm_controller'] + controller_manager_node_name + 
                 controller_manager_timeout
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(package_path, 'rviz', 'ur.rviz')],
        condition=IfCondition(use_rviz)
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )

    rosbag_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', 'rosbag_recorder.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbag_storage_dir': rosbag_path
        }.items(),
        condition=IfCondition(record)
    )



    delay_rosbag_recorder = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rosbag_recorder]
        )
    )

    nodes = [
        gz_spawn_entity,
        gazebo,
        bridge,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz,
        delay_arm_controller,
        delay_rosbag_recorder
    ]

    return LaunchDescription(declared_arguments + nodes)