import numpy as np
from .utils import cubic_spline, rotation_cubic, tfmatrix_to_array, calc_task_error, ControlledThread
from mujoco_ros_sim import ControllerInterface
from .robot_data import FR3RobotData
from rclpy.node import Node
from fr3_controller_wrapper_cpp import Controller as Controllercpp
from std_msgs.msg import Int32

class FR3Controller(ControllerInterface):
    """
    Controller implementation for the FR3 robot.
    
    This controller uses FR3RobotData to update the robot's state and subscribes to key inputs
    to change the control mode. It computes desired joint positions using cubic spline interpolation
    in an asynchronous thread. The controller provides control commands that are applied to the robot's actuators.
    
    Attributes:
        robot_data (FR3RobotData): Instance for managing the FR3 robot's data.
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
        Initializes the FR3Controller.
        
        Parameters:
            node (Node): A ROS2 node instance used for logging, subscriptions, and communication.
            dt (float): Simulation time step in seconds.
            mj_joint_names (list): List of joint names from the MuJoCo simulation.
            
        Returns:
            None
        """
        super().__init__(node, dt, mj_joint_names)
        
        self.key_subscriber = self.node.create_subscription(Int32, '/key_input', self.keyCallback, 10)
        self.robot_data = FR3RobotData(self.node, mj_joint_names)
        self.ee_name = "fr3_link8"
            
    def starting(self) -> None:
        """
        Called once when the controller starts.
        Captures the initial joint positions from the robot data and records the control start time.
    
        Returns:
            None
        """
        self.is_mode_changed = False
        self.mode = 'init'
        self.control_start_time = self.current_time
        
        self.q_init = self.q
        self.x_init = tfmatrix_to_array(self.robot_data.getPose(self.ee_name))
        self.q_desired = self.q_init
        self.torque_desired = self.tau
        
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau: np.ndarray, current_time: float) -> None:
        """
        Updates the controller's state with the latest simulation data.
        This method updates the internal simulation time, refreshes the robot state via FR3RobotData,
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
        
        # Update the robot's state using the FR3RobotData interface.
        self.robot_data.updateState(pos, vel, tau)
        
        self.q = self.robot_data.q
        self.x = tfmatrix_to_array(self.robot_data.getPose(self.ee_name))
        self.J = self.robot_data.getJacobian(self.ee_name)
        self.qdot = self.robot_data.qdot
        self.tau = self.robot_data.tau
        
    def compute(self) -> None:
        """
        Computes the control commands for the FR3 robot.
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
            print("[FR3controller] 제한 시간 초과, 스레드 종료 시도")
            t.kill()
            
    def getCtrlInput(self) -> tuple[np.ndarray, list]:
        """
        Retrieves the computed control commands to be applied to the robot.
        
        Returns:
            tuple:
                - np.ndarray: Array containing the desired joint positions (control commands).
                - list: List of robot joint names corresponding to the control commands.
        """
        return self.torque_desired, self.robot_data.rb_joint_names
    
    # =============================================================================================
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
        elif msg.data == 3:
            self.setMode('teleop')
                    
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
        
    def asyncCalculationProc(self):
        """
        Performs asynchronous control computations in a separate thread.
        """
        if self.is_mode_changed:
            self.q_init = self.robot_data.q
            self.x_init = self.x
            self.control_start_time = self.current_time
            self.is_mode_changed = False
        
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'init':
            target_q = np.array([0, 0, 0, 0, 0, 0, np.pi/4])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(7),                    # Initial velocity (assumed zero)
                np.zeros(7)                     # Final velocity (assumed zero)
            )
            self.torque_desired = self.PDJointControl(self.q_desired, np.zeros(7), 1000, 100)
        elif self.mode == 'home':
            target_q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(7),                    # Initial velocity (assumed zero)
                np.zeros(7)                     # Final velocity (assumed zero)
            )
            self.torque_desired = self.PDJointControl(self.q_desired, np.zeros(7), 1000, 100)

        elif self.mode == 'teleop':
            target_x = np.array([0.3, 0.0, 0.5, -3.14, 0.0, 0.0])
            self.pos_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 5.0,  # End time of interpolation (2 seconds later)
                self.x_init[:3],                # Initial EE pose
                target_x[:3],                   # Target EE pose
                np.zeros(3),                    # Initial EE velocity (assumed zero)
                np.zeros(3)                     # Final EE velocity (assumed zero)
            )

            self.rot_desired = rotation_cubic(
                self.current_time, 
                self.control_start_time, 
                self.control_start_time + 5.0,
                self.x_init[3:],
                target_x[3:]
            )

            # You can use R_interp directly (as a 3x3 matrix) or convert it to a rotation vector/Euler angles.
            self.x_desired = np.hstack((self.pos_desired, self.rot_desired))

            self.torque_desired = self.PDTaskControl(self.x_desired, np.zeros(6), 500, 100)

    def PDJointControl(self, q_desired:np.ndarray, qdot_desired:np.ndarray, kp:float, kd:float) -> np.ndarray:
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
        q_error = q_desired - self.q
        qdot_error = qdot_desired - self.qdot
        tau = self.robot_data.getMassMatrix() @ (kp * q_error + kd * qdot_error) + self.robot_data.getNonlinearEffects()
        return tau
    
    def PDTaskControl(self, x_desired:np.ndarray, xdot_desired:np.ndarray, kp:float, kd:float) -> np.ndarray:
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
        x_error = calc_task_error(self.robot_data.getPose(self.ee_name), x_desired)
        xdot_error = xdot_desired - self.J @ self.qdot

        N = np.identity(7) - self.J.T @ self.J @ np.linalg.inv(self.J.T @ self.J + 0.09 * np.identity(7)) 

        tau_task = N @ (-self.qdot)

        tau = self.J.T @ (self.robot_data.getTaskMassMatrix(self.ee_name) @ (kp * x_error + kd * xdot_error) + self.robot_data.getTaskNonlinearEffects(self.ee_name)) + tau_task
        return tau