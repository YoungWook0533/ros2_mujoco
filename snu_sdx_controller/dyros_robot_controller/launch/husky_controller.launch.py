from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    husky_controller.launch.py:
    - mujoco_ros_sim 패키지의 'mujoco_ros_sim' 노드를 실행
    - 파라미터로 'controller_class', 'robot_data_class'를
      각각 'dyros_controller.husky_controller.controller.HuskyController',
           'dyros_controller.husky_controller.robot_data.HuskyRobotData'
      로 설정
    - 필요한 경우 'robot_name'도 launch argument로 받아서 세팅
    """

    # (옵션) 로봇 이름 파라미터
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='franka_fr3_torque',
        description='Name of the robot model used in MuJoCo'
    )

    # 커스텀 컨트롤러/로봇데이터 클래스 경로
    controller_class_arg = DeclareLaunchArgument(
        'controller_class',
        default_value='husky_controller.controller.HuskyController',
        description='Python path for HuskyController'
    )
    # robot_data_class_arg = DeclareLaunchArgument(
    #     'robot_data_class',
    #     default_value='dyros_controller.husky_controller.robot_data.HuskyRobotData',
    #     description='Python path for HuskyRobotData'
    # )

    # 런치에서 참조할 LaunchConfiguration
    robot_name = LaunchConfiguration('robot_name')
    controller_class = LaunchConfiguration('controller_class')
    # robot_data_class = LaunchConfiguration('robot_data_class')

    # 실제 시뮬레이션 노드 (mujoco_ros_sim) 실행
    sim_node = Node(
        package='mujoco_ros_sim',          # 패키지 이름 (setup.py의 name, package.xml의 <name>)
        executable='mujoco_ros_sim',       # entry_point: mujoco_ros_sim.py의 main
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'controller_class': controller_class},
            # {'robot_data_class': robot_data_class}
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_class_arg,
        # robot_data_class_arg,
        sim_node,
        # ExecuteProcess(
        #     cmd=[
        #         'gdb', '--args',
        #         '/home/yoonjunheon/ros2_ws/install/mujoco_ros_sim/lib/mujoco_ros_sim/mujoco_ros_sim',
        #         '--ros-args', '-r', '__node:=mujoco_sim_node',
        #         '--params-file', '/tmp/launch_params_9c7kxie3',
        #         '--params-file', '/tmp/launch_params_l_j3krii'
        #     ],
            # output='screen'
        # )
    ])
