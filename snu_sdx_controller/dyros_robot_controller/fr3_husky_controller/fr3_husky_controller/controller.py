import numpy as np
from .utils import tfmatrix_to_array,ControlledThread
from mujoco_ros_sim import ControllerInterface
# from .robot_data import FR3HuskyRobotData
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from fr3_husky_controller_wrapper_cpp import Controller as Controllercpp
from fr3_husky_controller_wrapper_cpp import RobotData as RobotDatacpp
from fr3_husky_controller_wrapper_cpp import cubic, rotationCubic, Euler2rot, rot2Euler
from std_msgs.msg import Int32, Int8MultiArray
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, WrenchStamped

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

        # urdf_path = os.path.join(
        #     get_package_share_directory('dyros_robot_controller'),
        #     'fr3_husky_controller',
        #     'robot',
        #     'fr3_husky.urdf'
        # )

        arm_urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'robot',
            'fr3.urdf'
        )

        base_urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'robot',
            'husky.urdf'
        )

        gains_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'config',
            'gains.yaml'
        )

        mobile_config_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_husky_controller',
            'config',
            'mobile_config.yaml'
        )

        # self.robot_data = FR3HuskyRobotData(self.node, mj_joint_names)
        self.robot_data = RobotDatacpp(base_urdf_path, arm_urdf_path, gains_path)
        self.controller = Controllercpp(0.001, self.robot_data, mobile_config_path)
        self.ee_name = "fr3_link8"

        self.desired_wheel_vel = np.zeros(4)
        self.target_base_vel = np.zeros(3)
        self.mobile_kv = 50

        self.control_input = np.zeros(11)

        self.key_subscriber = self.node.create_subscription(Int32, '/key_input', self.keyCallback, 10)
        self.subscription = self.node.create_subscription(Twist, '/cmd_vel', self.target_velocity_callback, 1)

        self.joint_state_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        # Wrench publisher
        self.wrench_pub = self.node.create_publisher(WrenchStamped, '/wrench', 10)

    def starting(self) -> None:
        """
        Called once when the controller starts.
        Captures the initial joint positions from the robot data and records the control start time.
    
        Returns:
            None
        """
        self.is_mode_changed = False
        self.init_keyboard = False
        self.mode = 'init'
        self.control_start_time = self.current_time
        
        self.q_init = self.q
        self.x_init_mat = self.robot_data.getPose(self.ee_name)
        self.x_init = tfmatrix_to_array(self.x_init_mat)
        self.q_desired = self.q_init
        self.pos_desired = self.x_init[:3]
        self.rot_desired = self.x_init[3:]
        self.torque_desired = self.tau

        
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
        real_pose = pos[6:]
        real_vel = vel[6:]
        real_tau = tau[6:]
        
        # Update the robot's state using the FR3HuskyRobotData interface.
        self.robot_data.updateState(real_pose, real_vel, real_tau)

        self.q = self.robot_data.getq()
        self.x = tfmatrix_to_array(self.robot_data.getPose(self.ee_name))
        self.J = self.robot_data.getJacobian(self.ee_name)
        self.qdot = self.robot_data.getqdot()
        self.tau = self.robot_data.gettau()
        self.wrench = self.robot_data.getWrench(self.ee_name)
        
        self.wheel_pose = self.robot_data.getWheelPose()
        self.wheel_vel = self.robot_data.getWheelVel()
        self.wheel_torque = np.zeros(4)
        self.current_base_vel = self.controller.FK(self.wheel_vel)

        # Publish jointstate
        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name     = self.robot_data.getWheelNames() + self.robot_data.getJointNames()
        js.position = list(self.wheel_pose) + list(self.q)
        js.velocity = list(self.wheel_vel) + list(self.qdot)
        js.effort   = list(self.wheel_torque) + list(self.tau)
        self.joint_state_pub.publish(js)

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
        return self.control_input, self.robot_data.getWheelNames()+self.robot_data.getJointNames()
    
    # =============================================================================================
    def target_velocity_callback(self, msg):
        self.target_base_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def keyCallback(self, msg: Int32):
        """
        Callback function for key input messages.
        If a message with data value 1 is received, the control mode is set to 'home'.
        
        Parameters:
            msg (Int32): The received key input message containing an integer.
            
        Returns:
            None
        """
        self.node.get_logger().info(f"[FR3Controller] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('init')
        elif msg.data == 2:
            self.setMode('home')
                    
    def setMode(self, mode: str):
        """
        Sets the current control mode and flags that a mode change has occurred.
        Also logs the mode change.
        
        Parameters:
            mode (str): The new control mode (e.g., 'home').
            
        Returns:
            None
        """
        self.is_mode_changed = True
        self.node.get_logger().info(f"[FR3Controller] Mode changed: {mode}")
        self.mode = mode
        
    def keyboard_callback(self, msg: Twist):
        # if not self.button_pressed:
        #     self.cmd_vel = np.zeros(6)
    
        # else: 
        #     self.cmd_vel = np.array([5 * msg.linear.x, 5 * msg.linear.y, 5 * msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])

    def ee_callback(self, msg: JointTrajectory):
        point = msg.points[0]
        self.target_x     = np.array(point.positions)
        self.target_xdot = np.array(point.velocities)
        self.duration     = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

        # reset interpolation
        self.is_mode_changed     = True

    def asyncCalculationProc(self):
        """
        Performs asynchronous control computations in a separate thread.
        """
        # print(self.current_base_vel)
        desired_wheel_vel = self.controller.VelocityCommand(self.target_base_vel, self.current_base_vel)
        self.desired_wheel_vel = (desired_wheel_vel - self.wheel_vel) * self.mobile_kv
        # print("-----------------------------------------------------")
        # print(self.target_base_vel)
        # print(self.current_base_vel)
        # print(desired_wheel_vel)
        # print(self.wheel_vel)
        # print("-----------------------------------------------------")

        if self.is_mode_changed:
            self.q_init = self.q
            self.x_init_mat = self.robot_data.getPose(self.ee_name)
            self.x_init = tfmatrix_to_array(self.x_init_mat)
            self.control_start_time = self.current_time
            self.init_keyboard = True
            self.is_mode_changed = False
        
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'init':
            target_q = np.array([0, 0, 0, 0, 0, 0, np.pi/4])
            for i in range(7):
                self.q_desired[i] = cubic(
                    self.current_time,
                    self.control_start_time,
                    self.control_start_time + 2.0,
                    self.q_init[i],
                    target_q[i],
                    0.0,  # initial velocity (assumed zero)
                    0.0   # final velocity (assumed zero)
                )
            self.torque_desired = self.PDJointControl(self.q_desired, np.zeros(7))
        elif self.mode == 'home':
            target_q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
            for i in range(7):
                self.q_desired[i] = cubic(
                    self.current_time,
                    self.control_start_time,
                    self.control_start_time + 2.0,
                    self.q_init[i],
                    target_q[i],
                    0.0,  # initial velocity (assumed zero)
                    0.0   # final velocity (assumed zero)
                )
            self.torque_desired = self.PDJointControl(self.q_desired, np.zeros(7))

        # self.torque_desired = [0,1,2,3,4,5,6]
        self.control_input = np.hstack((self.desired_wheel_vel, self.torque_desired))

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
    
    def PDJointControl(self, q_desired:np.ndarray, qdot_desired:np.ndarray) -> np.ndarray:
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
        try:
            return self.controller.PDJointControl(q_desired, qdot_desired)
        except Exception as e:
            print(f"Error in torque calculation: {e}")
    