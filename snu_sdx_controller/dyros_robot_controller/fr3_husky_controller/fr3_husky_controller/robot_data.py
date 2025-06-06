import numpy as np
from mujoco_ros_sim import RobotDataInterface
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from husky_controller_wrapper_cpp import RobotData as RobotDatacpp

class HuskyRobotData(RobotDataInterface):
    """
    Concrete implementation of RobotDataInterface for the Husky robot.
    
    This class loads robot-specific data from a URDF file and uses a C++ wrapper
    (RobotDatacpp) to handle the robot's state. It provides an interface to update
    the robot's state with simulation data (positions, velocities, torques) from a MuJoCo model.
    
    Attributes:
        node (Node): ROS2 node instance used for logging and communication.
        mj_joint_names (list): List of joint names from the MuJoCo simulation.
        robot_data (RobotDatacpp): Instance of the C++ robot data wrapper.
        rb_joint_names (list): List of joint names as defined in the robot_data wrapper.
        q (np.ndarray): Array for storing joint positions.
        qdot (np.ndarray): Array for storing joint velocities.
        tau (np.ndarray): Array for storing joint torques.
    """

    def __init__(self, node: Node, mj_joint_names: list):
        """
        Initializes the HuskyRobotData object.
        
        This method sets up the robot data by loading the Husky URDF file from the 
        'dyros_robot_controller' package share directory, initializes the C++ robot
        data wrapper, and creates arrays to store the robot's state.
        
        Parameters:
            node (Node): A ROS2 node instance used for logging and communication.
            mj_joint_names (list): A list of joint names (strings) from the MuJoCo simulation.
            
        Returns:
            None
        """
        # Initialize the base RobotDataInterface with the ROS node and joint names from MuJoCo.
        super().__init__(node, mj_joint_names)
        
        # Construct the path to the Husky URDF file using the package share directory.
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'husky_controller',
            'robot',
            'husky.urdf'
        )
        # Create the C++ robot data instance using the URDF file.
        self.robot_data = RobotDatacpp(urdf_path)
        
        # Retrieve the robot joint names from the C++ robot data wrapper.
        self.wheel_joint_names = self.robot_data.getWheelNames()
        
        # Initialize numpy arrays for joint positions (q), velocities (qdot), and torques (tau)
        # with a size corresponding to the number of robot joints.
        self.wheel_pose = np.zeros(len(self.wheel_joint_names))
        self.wheel_vel = np.zeros(len(self.wheel_joint_names))
        self.wheel_torque = np.zeros(len(self.wheel_joint_names))
    
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau: np.ndarray):
        """
        Updates the internal state of the Husky robot data using simulation values.
        
        This method maps simulation state values from the MuJoCo model to the robot-specific
        joint ordering and then calls the C++ robot data wrapper to update its internal state.
        
        Parameters:
            pos (np.ndarray): Array containing current joint positions from the MuJoCo simulation.
            vel (np.ndarray): Array containing current joint velocities from the MuJoCo simulation.
            tau (np.ndarray): Array containing current joint torques from the MuJoCo simulation.
            
        Returns:
            None
        """
        # For each joint in the robot's joint list (as defined in the C++ wrapper)
        for wheel_joint_name in self.wheel_joint_names:
            # Find the index of the current robot joint name in the robot data array.
            wheel_index = self.wheel_joint_names.index(wheel_joint_name)
            # Find the corresponding index in the MuJoCo joint names list.
            mj_index = self.mj_joint_names.index(wheel_joint_name)
            
            # Map the simulation joint position to the robot data array.
            self.wheel_pose[wheel_index] = pos[mj_index]
            # Map the simulation joint velocity to the robot data array.
            self.wheel_vel[wheel_index] = vel[mj_index]
            # Map the simulation joint torque to the robot data array.
            self.wheel_torque[wheel_index] = tau[mj_index]
        
        # Update the robot_data instance (C++ wrapper) with the new state arrays.
        if(not self.robot_data.updateState(self.wheel_pose, self.wheel_vel, self.wheel_torque)):
            self.node.get_logger().error("[HuskyRobotData] Failed to update robot state.")

# ==================================================================================================
    def getWheelPose(self) -> np.ndarray:
        """
        Retrieves wheel positions from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing wheel positions.
        """
        return self.robot_data.getWheelPose()
    
    def getWheelVel(self) -> np.ndarray:
        """
        Retrieves wheel velocities from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing wheel velocities.
        """
        return self.robot_data.getWheelVel()
    