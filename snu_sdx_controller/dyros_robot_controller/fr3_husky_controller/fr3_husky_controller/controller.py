import numpy as np
from .utils import tfmatrix_to_array,ControlledThread
from mujoco_ros_sim import ControllerInterface
from .robot_data import FR3HuskyRobotData
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from fr3_husky_controller_wrapper_cpp import Controller as Controllercpp
from fr3_husky_controller_wrapper_cpp import RobotData as RobotDatacpp
from geometry_msgs.msg import Twist

class FR3HuskyController(ControllerInterface):
    """
    Controller implementation for the FR3Husky robot.
    
    This controller uses FR3HuskyRobotData to update the robot's state and subscribes to key inputs
    to change the control mode. It computes desired joint positions using cubic spline interpolation
    in an asynchronous thread. The controller provides control commands that are applied to the robot's actuators.
    
    Attributes:
        robot_data (FR3HuskyRobotData): Instance for managing the FR3Husky robot's data.
        is_mode_changed (bool): Flag indicating if the control mode has changed.
        key_subscriber: ROS2 subscription for key input messages.
        q_desired (np.ndarray): Desired joint positions computed by the controller.
        mode (str): Current control mode (e.g., 'home').
        current_time (float): Latest simulation time (updated in updateState).
        q_init (np.ndarray): Initial joint positions recorded at the start of a control sequence.
        control_start_time (float): Simulation time when the current control sequence started.
    """
    
    def __init__(self, node: Node, dt: float, mj_joint_names: list):
        """
        Initializes the FR3HuskyController.
        
        Parameters:
            node (Node): A ROS2 node instance used for logging, subscriptions, and communication.
            dt (float): Simulation time step in seconds.
            mj_joint_names (list): List of joint names from the MuJoCo simulation.
            
        Returns:
            None
        """
        super().__init__(node, dt, mj_joint_names)

        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'robot',
            'fr3_husky.urdf'
        )

        yaml_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'config',
            'mobile_config.yaml'
        )

        # self.robot_data = FR3HuskyRobotData(self.node, mj_joint_names)
        self.robot_data = RobotDatacpp(urdf_path)
        self.controller = Controllercpp(0.001, self.robot_data, yaml_path)
        self.desired_wheel_vel = np.zeros(4)
        self.target_base_vel = np.zeros(3)
        self.mobile_kv = 50

        self.subscription = self.node.create_subscription(Twist, '/cmd_vel', self.target_velocity_callback, 1)

    def starting(self) -> None:
        """
        Called once when the controller starts.
        Captures the initial joint positions from the robot data and records the control start time.
    
        Returns:
            None
        """
        self.control_start_time = self.current_time

        
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau: np.ndarray, current_time: float) -> None:
        """
        Updates the controller's state with the latest simulation data.
        This method updates the internal simulation time, refreshes the robot state via FR3HuskyRobotData,
        and logs the current joint positions for debugging.
        
        Parameters:
            pos (np.ndarray): Current joint positions from the MuJoCo simulation.
            vel (np.ndarray): Current joint velocities from the MuJoCo simulation.
            tau (np.ndarray): Current joint torques from the MuJoCo simulation.
            current_time (float): The current simulation time.
            
        Returns:
            None
        """
        # Update the current simulation time.
        self.current_time = current_time

        # Skip floating joints
        wheel_pose = pos[6:]
        wheel_vel = vel[6:]
        wheel_tau = tau[6:]
        
        # Update the robot's state using the FR3HuskyRobotData interface.
        self.robot_data.updateState(wheel_pose, wheel_vel, wheel_tau)
        
        self.wheel_pose = self.robot_data.getWheelPose()
        self.wheel_vel = self.robot_data.getWheelVel()
        # self.wheel_torque = self.robot_data.getWheelPose()
        self.current_base_vel = self.controller.FK(self.wheel_vel)

    def compute(self) -> None:
        """
        Computes the control commands for the FR3Husky robot.
        Launches an asynchronous thread to compute the desired joint positions and waits for the
        computation to complete within the allocated simulation time step. If the computation exceeds
        the allowed time, it attempts to forcibly terminate the thread.
        
        Returns:
            None
        """
        # Start an asynchronous thread to execute the control calculation.
        t = ControlledThread(target=self.asyncCalculationProc)
        t.start()

        # Wait for the thread to finish, up to a maximum of dt seconds.
        t.join(self.dt)

        # If the thread is still running after dt seconds, attempt to kill it.
        if t.is_alive():
            print("[FR3Huskycontroller] 제한 시간 초과, 스레드 종료 시도")
            t.kill()
            
    def getCtrlInput(self) -> tuple[np.ndarray, list]:
        """
        Retrieves the computed control commands to be applied to the robot.
        
        Returns:
            tuple:
                - np.ndarray: Array containing the desired joint positions (control commands).
                - list: List of robot joint names corresponding to the control commands.
        """
        return self.desired_wheel_vel, self.robot_data.getWheelNames()
    
    # =============================================================================================
    def target_velocity_callback(self, msg):
        self.target_base_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def asyncCalculationProc(self):
        """
        Performs asynchronous control computations in a separate thread.
        """
        # print(self.current_base_vel)
        desired_wheel_vel = self.controller.VelocityCommand(self.target_base_vel, self.current_base_vel)
        self.desired_wheel_vel = (desired_wheel_vel - self.wheel_vel) * self.mobile_kv
        print("-----------------------------------------------------")
        print(self.target_base_vel)
        print(self.current_base_vel)
        print(desired_wheel_vel)
        print(self.wheel_vel)
        print("-----------------------------------------------------")

    # *** Python functions ***

    def FK(self, wheel_vel:np.ndarray) -> np.ndarray:
        """
        PD control law for joint position and velocity tracking.
        
        Parameters:
            q_desired (np.ndarray): Desired joint positions.
            qdot_desired (np.ndarray): Desired joint velocities.
            kp (float): Proportional gain for position control.
            kd (float): Derivative gain for velocity control.
            
        Returns:
            np.ndarray: Torque commands computed by the PD controller.
        """
        return self.controller.FK(wheel_vel)
    
    def VelocityCommand(self, desired_base_vel:np.ndarray, current_base_vel:np.ndarray) -> np.ndarray:
        """
        PD control law for joint position and velocity tracking.
        
        Parameters:
            q_desired (np.ndarray): Desired joint positions.
            qdot_desired (np.ndarray): Desired joint velocities.
            kp (float): Proportional gain for position control.
            kd (float): Derivative gain for velocity control.
            
        Returns:
            np.ndarray: Torque commands computed by the PD controller.
        """
        return self.controller.VelocityCommand(desired_base_vel, current_base_vel)
    