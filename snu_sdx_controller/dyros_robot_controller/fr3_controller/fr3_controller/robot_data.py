import numpy as np
from mujoco_ros_sim import RobotDataInterface
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from fr3_controller_wrapper_cpp import RobotData as RobotDatacpp

class FR3RobotData(RobotDataInterface):
    """
    Concrete implementation of RobotDataInterface for the FR3 robot.
    
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
        Initializes the FR3RobotData object.
        
        This method sets up the robot data by loading the FR3 URDF file from the 
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
        
        # Construct the path to the FR3 URDF file using the package share directory.
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_controller',
            'robot',
            'fr3.urdf'
        )
        yaml_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'fr3_controller',
            'config',
            'gains.yaml'
        )
        # Create the C++ robot data instance using the URDF file.
        self.robot_data = RobotDatacpp(urdf_path, yaml_path)
        
        # Retrieve the robot joint names from the C++ robot data wrapper.
        self.rb_joint_names = self.robot_data.getJointNames()
        
        # Initialize numpy arrays for joint positions (q), velocities (qdot), and torques (tau)
        # with a size corresponding to the number of robot joints.
        self.q = np.zeros(len(self.rb_joint_names))
        self.qdot = np.zeros(len(self.rb_joint_names))
        self.tau = np.zeros(len(self.rb_joint_names))
    
    def updateState(self, pos: np.ndarray, vel: np.ndarray, tau: np.ndarray):
        """
        Updates the internal state of the FR3 robot data using simulation values.
        
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
        for rb_joint_name in self.rb_joint_names:
            # Find the index of the current robot joint name in the robot data array.
            rb_index = self.rb_joint_names.index(rb_joint_name)
            # Find the corresponding index in the MuJoCo joint names list.
            mj_index = self.mj_joint_names.index(rb_joint_name)
            
            # Map the simulation joint position to the robot data array.
            self.q[rb_index] = pos[mj_index]
            # Map the simulation joint velocity to the robot data array.
            self.qdot[rb_index] = vel[mj_index]
            # Map the simulation joint torque to the robot data array.
            self.tau[rb_index] = tau[mj_index]
        
        # Update the robot_data instance (C++ wrapper) with the new state arrays.
        if(not self.robot_data.updateState(self.q, self.qdot, self.tau)):
            self.node.get_logger().error("[FR3RobotData] Failed to update robot state.")

# ==================================================================================================
    def getq(self) -> np.ndarray:
        """
        Retrieves joint positions from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing joint positions.
        """
        return self.robot_data.getq()
    
    def getqdot(self) -> np.ndarray:
        """
        Retrieves joint velocities from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing joint velocities.
        """
        return self.robot_data.getqdot()
    
    def gettau(self) -> np.ndarray:
        """
        Retrieves joint torques from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing joint torques.
        """
        return self.robot_data.gettau()

    def getMassMatrix(self) -> np.ndarray:
        """
        Retrieves the mass matrix from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the mass matrix.
        """
        return self.robot_data.getMassMatrix()
    
    def getCoriolis(self) -> np.ndarray:
        """
        Retrieves the Coriolis vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the Coriolis vector.
        """
        return self.robot_data.getCoriolis()

    def getGravity(self) -> np.ndarray:
        """
        Retrieves the gravity vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the gravity vector.
        """
        return self.robot_data.getGravity()
    
    def getNonlinearEffects(self) -> np.ndarray:
        """
        Retrieves the nonlinear effects vector from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the nonlinear effects vector.
        """
        return self.robot_data.getNonlinearEffects()
    
    def getPose(self, link_name:str=None) -> np.ndarray:
        """
        Retrieves the end effector pose vector of from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the end effector pose vector.
        """
        if link_name is None:
            x = self.robot_data.getPose()
        else:
            x = self.robot_data.getPose(link_name)
        return np.asarray(x)
    
    def getJacobian(self, link_name:str=None) -> np.ndarray:
        """
        Retrieves the jacobian from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the jacobian.
        """
        if link_name is None:
            J = self.robot_data.getJacobian()
        else:
            J = self.robot_data.getJacobian(link_name)
        return np.asarray(J)
    
    def getTaskMassMatrix(self, link_name:str=None) -> np.ndarray:
        """
        Retrieves the task space mass matrix from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the task space mass matrix.
        """
        return self.robot_data.getTaskMassMatrix(link_name)
    
    def getTaskNonlinearEffects(self, link_name:str=None) -> np.ndarray:
        """
        Retrieves the task space nonlinear effects from the robot data wrapper.
        
        Returns:
            np.ndarray: Array containing the task space nonlinear effects.
        """
        return self.robot_data.getTaskNonlinearEffects(link_name)