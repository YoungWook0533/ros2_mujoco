import time
import numpy as np

import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer

# Import local utility functions:
# - load_mj_model: loads a MuJoCo model given a robot name.
# - precise_sleep: a high-precision sleep function to maintain time steps.
# - load_class dynamically loads a class (e.g., a controller) from a string identifier.
from geometry_msgs.msg import WrenchStamped
from .utils import load_mj_model, precise_sleep, load_class


class MujocoSimNode(Node):
    """
    A ROS2 Node that integrates MuJoCo simulation with a controller interface.
    
    This class initializes a MuJoCo simulation using a robot model and a dynamically loaded controller.
    It retrieves simulation parameters, loads the model, extracts joint and actuator names, and then
    enters a loop to update the simulation state, compute control commands, and update the GUI.

    Attributes:
        mj_model: The MuJoCo model loaded based on the provided robot name.
        mj_data: The simulation data structure associated with mj_model.
        dt (float): The simulation time step extracted from the model options.
        viewer_fps (float): The target frames per second for the MuJoCo viewer.
        joint_names (list): A list of joint names extracted from the model.
        actuator_names (list): A list of actuator names extracted from the model.
        controller: An instance of a dynamically loaded controller class.
        is_starting (bool): A flag to indicate if the simulation has just started.
    """

    def __init__(self):
        """
        Initializes the MujocoSimNode.
        
        The function:
          - Initializes the ROS2 node with the name 'mujoco_sim_node'.
          - Declares and retrieves ROS2 parameters 'robot_name' and 'controller_class'.
          - Loads the MuJoCo model and associated simulation data.
          - Extracts joint and actuator names from the model.
          - Dynamically loads the specified controller class and initializes it.
          
        Arguments:
            None (parameters are retrieved from the ROS2 parameter server).
            
        Returns:
            None
        """
        super().__init__('mujoco_sim_node')

        # 1) Retrieve parameters from the ROS2 parameter server
        self.declare_parameter('robot_name')
        # robot_name: string indicating the name of the robot model to load.
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.declare_parameter('controller_class')
        # controller_class_str: string identifier for the controller class to be dynamically loaded.
        controller_class_str = self.get_parameter('controller_class').get_parameter_value().string_value

        # The robot_data_class parameter is commented out but can be declared similarly if needed.
        # self.declare_parameter('robot_data_class')
        # robot_data_class_str = self.get_parameter('robot_data_class').get_parameter_value().string_value

        # 2) Load the MuJoCo model and create the simulation data object.
        self.mj_model = load_mj_model(robot_name)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.dt = self.mj_model.opt.timestep  # Simulation time step (in seconds)
        self.viewer_fps = 60.0  # Target frames per second for GUI updates
        self.wrench_sensor = np.zeros(6)
        
        # 3) Extract joint names from the model.
        self.joint_names = []
        for i in range(self.mj_model.njnt):
            # name_adr: starting address for the i-th joint name in the names array.
            name_adr = self.mj_model.name_jntadr[i]
            # Decode the joint name from a byte string.
            jname = self.mj_model.names[name_adr:].split(b'\x00', 1)[0].decode('utf-8')
            self.joint_names.append(jname)

        # Extract actuator names from the model.
        self.actuator_names = []
        for i in range(self.mj_model.nu):
            # name_adr: starting address for the i-th actuator name in the names array.
            name_adr = self.mj_model.name_actuatoradr[i]
            # Decode the actuator name.
            aname = self.mj_model.names[name_adr:].split(b'\x00', 1)[0].decode('utf-8')
            self.actuator_names.append(aname)
        
        # 5) Dynamically load and initialize the controller.
        Controller = load_class(controller_class_str)
        if Controller is not None:
            # Instantiate the controller with the following arguments:
            #   - self: reference to the current node (could be used for logging or ROS integration)
            #   - self.dt: simulation time step (float)
            #   - self.joint_names: list of joint names (list of strings)
            self.controller = Controller(self, self.dt, self.joint_names)
        else:
            self.controller = None
        self.wrench_pub = self.create_publisher(WrenchStamped, '/wrench_sensor', 1)
        self.get_logger().info(f"Sim node with controller={controller_class_str}")

        self.sensor_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "fr3_sensor")
        self.base_body_id   = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, "base")

    def publish_wrench_sensor(self):
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = "fr3_link8"

        # wrench_data = self.controller.getWrench("hand_tcp")
        wrench_data = self.wrench_sensor

        wrench_msg.wrench.force.x = wrench_data[0]
        wrench_msg.wrench.force.y = wrench_data[1]
        wrench_msg.wrench.force.z = wrench_data[2]
        wrench_msg.wrench.torque.x = wrench_data[3]
        wrench_msg.wrench.torque.y = wrench_data[4]
        wrench_msg.wrench.torque.z = wrench_data[5]

        self.wrench_pub.publish(wrench_msg)

    def _sensor_wrench_to_base(self, wrench_s):
        """Convert [Fx,Fy,Fz,Mx,My,Mz] from sensor frame to base frame."""

        F_s, tau_s = wrench_s[:3], wrench_s[3:]

        # ---- 1. Rotation part ----------------------------------
        R_ws = self.mj_data.xmat[self.sensor_body_id].reshape(3, 3)  # sensor → world
        R_wb = self.mj_data.xmat[self.base_body_id].reshape(3, 3)    # base   → world
        R_bs = R_wb.T @ R_ws                                         # sensor → base
        F_b   = R_bs @ F_s
        tau_b = R_bs @ tau_s

        # ---- 2. Moment shift (r × F) ----------------------------
        p_ws = self.mj_data.xpos[self.sensor_body_id]                # world coords
        p_wb = self.mj_data.xpos[self.base_body_id]                  # world coords
        r_bs_world = p_ws - p_wb
        r_bs = R_wb.T @ r_bs_world                                   # expressed in base
        tau_b += np.cross(r_bs, F_b)

        return np.concatenate([F_b, tau_b])

                
    def run(self):
        """
        Runs the MuJoCo simulation loop along with controller updates and GUI synchronization.
        
        This function:
          - Launches a passive MuJoCo viewer for the simulation.
          - Enters a while-loop that continues while the viewer is running.
          - Advances the simulation by one time step.
          - Retrieves the current simulation state (positions, velocities, forces).
          - Updates the controller with the latest state.
          - Retrieves control commands from the controller and applies them to the simulation.
          - Updates the viewer at a fixed frame rate.
          - Sleeps for any remaining time to maintain the simulation's real-time rate.
          
        Arguments:
            None
            
        Returns:
            None
        """
        
        self.is_starting = True  # Flag to indicate the first iteration of the simulation loop
        
        # Launch the MuJoCo viewer in passive mode (without UI panels)
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data,
                                          show_left_ui=False, show_right_ui=False) as viewer:

            last_view_time = 0.0  # Timestamp of the last viewer synchronization

            # Main simulation loop: runs as long as the viewer is active.
            while viewer.is_running():
                step_start = time.perf_counter()  # Record the start time of the simulation step
                rclpy.spin_once(self, timeout_sec=0.0001)  # Process any pending ROS2 messages

                # Advance the simulation by one time step.
                mujoco.mj_step(self.mj_model, self.mj_data)
                
                # Retrieve the current simulation state:
                # pos: array of generalized positions (qpos)
                # vel: array of generalized velocities (qvel)
                # tau: array of applied forces (qfrc_applied, including any external forces)
                pos = np.copy(self.mj_data.qpos)
                vel = np.copy(self.mj_data.qvel)
                acc = np.copy(self.mj_data.qacc)
                tau = np.copy(self.mj_data.qfrc_actuator + self.mj_data.qfrc_applied + self.mj_data.qfrc_constraint)
                # self.wrench_sensor = self.mj_data.sensordata[0:6]
                # self.publish_wrench_sensor()

                wrench_raw = self.mj_data.sensordata[0:6]
                self.wrench_sensor = self._sensor_wrench_to_base(wrench_raw)
                self.publish_wrench_sensor()

                
                # Update the controller with the current state.
                if self.controller is not None:
                    # updateState updates the controller's internal state.
                    # Parameters:
                    #   - pos: positions (numpy array)
                    #   - vel: velocities (numpy array)
                    #   - tau: forces (numpy array)
                    #   - current simulation time (float)
                    self.controller.updateState(pos, vel, acc, tau, self.mj_data.time)
                    
                    # If this is the first iteration, run the controller's starting routine.
                    if self.is_starting:
                        self.controller.starting()
                        self.is_starting = False   
                    # Compute new control commands based on the updated state.
                    self.controller.compute()
                
                # Retrieve and apply the control commands from the controller.
                if self.controller is not None:
                    # getCtrlInput should return:
                    #   - ctrl_cmd: a list/array of control commands (e.g., torques or positions)
                    #   - actuator_names: corresponding list of actuator names (strings)
                    ctrl_cmd, actuator_names = self.controller.getCtrlInput()
                    
                    # For each actuator command, find the corresponding actuator in the model and apply the command.
                    for given_actuator_name, given_ctrl_cmd in zip(actuator_names, ctrl_cmd):
                        if given_actuator_name in self.actuator_names:
                            actuator_id = self.actuator_names.index(given_actuator_name)
                            self.mj_data.ctrl[actuator_id] = given_ctrl_cmd
                            
                # Update the GUI if enough simulation time has elapsed based on the viewer FPS.
                sim_time = self.mj_data.time
                if (sim_time - last_view_time) >= 1.0 / self.viewer_fps:
                    viewer.sync()  # Sync the viewer with the current simulation state.
                    last_view_time = sim_time
                
                # Calculate remaining time to sleep to maintain the simulation time step.
                leftover = self.dt - (time.perf_counter() - step_start)
                if leftover > 0:
                    precise_sleep(leftover)


def main(args=None):
    """
    The main entry point for starting the simulation node.
    
    This function initializes the ROS2 environment, creates an instance of MujocoSimNode,
    and enters the simulation loop. It also handles graceful shutdown on a KeyboardInterrupt.
    
    Arguments:
        args (list, optional): Command-line arguments to pass to the ROS2 initialization. Defaults to None.
        
    Returns:
        None
    """
    rclpy.init(args=args)  # Initialize the ROS2 communications.
    node = MujocoSimNode()  # Create an instance of the simulation node.
    try:
        node.run()  # Run the simulation loop.
    except KeyboardInterrupt:
        # Exit cleanly on Ctrl+C.
        pass
    finally:
        node.destroy_node()  # Clean up the node.
        rclpy.shutdown()  # Shutdown the ROS2 communications.
