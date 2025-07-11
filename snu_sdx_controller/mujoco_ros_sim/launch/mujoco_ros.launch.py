import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    robot = 'fr3'
    if(robot == 'fr3'):
        # 런치 아규먼트들
        robot_name_arg = DeclareLaunchArgument(
            'robot_name',
            default_value='franka_fr3_torque',
            description='Name of the robot model to be used in MuJoCo'
        )

        controller_class_arg = DeclareLaunchArgument(
            'controller_class',
            default_value='dyros_robot_controller.fr3_controller.controller.FR3Controller',
            description='Full Python path of the controller class to load (e.g. my_pkg.my_controller.MyController)'
        )

        robot_data_class_arg = DeclareLaunchArgument(
            'robot_data_class',
            default_value='dyros_robot_controller.fr3_controller.robot_data.FR3RobotData',
            description='Full Python path of the RobotData class to load (e.g. my_pkg.my_robot_data.MyRobotData)'
        )
    elif(robot == 'husky'):

        robot_name_arg = DeclareLaunchArgument(
            'robot_name',
            default_value='husky',
            description='Name of the robot model to be used in MuJoCo'
        )

        controller_class_arg = DeclareLaunchArgument(
            'controller_class',
            default_value='dyros_robot_controller.husky_controller.controller.HuskyController',
            description='Full Python path of the controller class to load (e.g. my_pkg.my_controller.MyController)'
        )

        robot_data_class_arg = DeclareLaunchArgument(
            'robot_data_class',
            default_value='dyros_robot_controller.husky_controller.robot_data.HuskyRobotData',
            description='Full Python path of the RobotData class to load (e.g. my_pkg.my_robot_data.MyRobotData)'
        )
    elif(robot == 'fr3_husky'):

        robot_name_arg = DeclareLaunchArgument(
            'robot_name',
            default_value='fr3_husky',
            description='Name of the robot model to be used in MuJoCo'
        )

        controller_class_arg = DeclareLaunchArgument(
            'controller_class',
            default_value='dyros_robot_controller.fr3_husky_controller.controller.FR3HuskyController',
            description='Full Python path of the controller class to load (e.g. my_pkg.my_controller.MyController)'
        )

        robot_data_class_arg = DeclareLaunchArgument(
            'robot_data_class',
            default_value='dyros_robot_controller.fr3_controller.robot_data.FR3HuskyRobotData',
            description='Full Python path of the RobotData class to load (e.g. my_pkg.my_robot_data.MyRobotData)'
        )
    
    # LaunchConfiguration 객체
    robot_name = LaunchConfiguration('robot_name')
    controller_class = LaunchConfiguration('controller_class')
    robot_data_class = LaunchConfiguration('robot_data_class')

    urdf = os.path.join(get_package_share_directory('dyros_robot_controller'), robot + '_controller/robot', robot + '.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    rviz_config_file = os.path.join(get_package_share_directory('dyros_robot_controller'), robot + '_controller/config', robot + '.rviz')


    # 실제 노드 실행 설정
    sim_node = Node(
        package='mujoco_ros_sim',      # 패키지 이름 (setup.py의 project_name 혹은 package.xml의 <name>)
        executable='mujoco_ros_sim',   # setup.py 혹은 entry_point에서 지정된 실행 가능 이름
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'controller_class': controller_class,
            'robot_data_class': robot_data_class
        }]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    gui = ExecuteProcess(
      cmd=[
        'ros2','run','rqt_gui','rqt_gui',
        '--standalone','rqt_dyros_gui','--force-discover'
      ],
      output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        controller_class_arg,
        robot_data_class_arg,
        sim_node,
        robot_state_publisher,
        rviz_node,
        gui
    ])