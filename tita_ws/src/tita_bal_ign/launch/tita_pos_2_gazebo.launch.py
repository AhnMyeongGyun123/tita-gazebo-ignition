import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    # Launch Arguments
    pkg_share = FindPackageShare(package='tita_bal_ign').find('tita_bal_ign')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    world_path = PathJoinSubstitution([
        FindPackageShare('tita_bal_ign'),
        'world',
        'tita_world.sdf'
    ])

    gz_args = LaunchConfiguration('gz_args', default=['-v 1 ', world_path])
    urdf_path = os.path.join(pkg_share, 'model/urdf/tita2.urdf')

    try:
        with open(urdf_path, 'r') as file:
            robot_description_content = file.read()
        print("URDF content loaded successfully.")
    except Exception as e:
        print(f"Failed to read URDF file: {e}")
        robot_description_content = ""

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'tita', '-allow_renaming', 'false',
                   '-x', '0',
                   '-y', '0',
                   #'-z', '0.3',
                   '-z', '0.45'
                   ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Joy
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        # parameters=[{'deadzone': 0.05,
        #              'autorepeat_rate': 0}],
        output='screen',

    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    bridge_state = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_info_bridge',
        arguments=[
            '/world/empty/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V'
        ],
        output='screen'
    )
    control_node = Node(
        package='tita_bal_ign',
        executable='tita_ctr',
        name='tita_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu', '/imu_sensor_broadcaster/imu'),
        ]
    )

    Load_tita_wheel_Controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'tita_wheel_controller'],
        output='screen'
    )

    load_imu = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'imu_sensor_broadcaster'],
        output='screen'
    )

    Load_Joint_State_Controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # init_leg_cmd = ExecuteProcess(
    # cmd=[
    #     'ros2', 'topic', 'pub', '--once',
    #     '/tita_leg_controller/commands',
    #     'std_msgs/msg/Float64MultiArray',
    #     '{data: [0.0, 1.31, -2.67, 0.0, 1.31, -2.67]}'
    # ],
    # output='screen'
    # )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', gz_args)]),
            # launch_arguments=[('gz_args', [gz_args, ' -v 1 /home/mok/Music/tita_ws/src/tita_bal/world/tita_world.sdf'])]),
            # launch_arguments=[('gz_args', [gz_args, ' -r -v 1 grid.sdf'])]),
            # launch_arguments=[('gz_args', ['-r -v 1 /usr/share/ignition/ignition-gazebo6/worlds/mod_empty.sdf'])]),
            # launch_arguments=[('gz_args', ['-r -v 3 /usr/share/ignition/ignition-gazebo6/worlds/mod_empty.sdf'])]),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[tita_wheel_controller_spawner,
        #                  tita_leg_controller_spawner,
        #                  imu_sensor_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=tita_wheel_controller_spawner,
        #         on_exit=[control_node],
        #     )
        # ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[Load_Joint_State_Controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Load_Joint_State_Controller,
                on_exit=[Load_tita_wheel_Controller,
                         load_imu],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=Load_tita_leg_Controller,
        #         on_exit=[init_leg_cmd],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Load_tita_wheel_Controller,
                on_exit=[control_node],
            )
        ),

        bridge,
        joy,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Controllers are launched via OnProcessExit chain above.
        # Do not add the same ExecuteProcess actions here, or launch will
        # raise "executed more than once".
        # Load_tita_wheel_Controller,
        # Load_tita_leg_Controller,
        # Load_Joint_State_Controller,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
