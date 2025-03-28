import numpy as np
import threading
import ctypes
from scipy.spatial.transform import Rotation
from scipy.linalg import expm

# ---------------------- Cubic Spline ----------------------
def cubic_spline(
    time: float,
    time_0: float,
    time_f: float,
    x_0: np.ndarray,
    x_f: np.ndarray,
    x_dot_0: np.ndarray,
    x_dot_f: np.ndarray
) -> np.ndarray:
    """
    Computes a cubic spline interpolation between x_0 and x_f over [time_0, time_f].
    For scalar or vector inputs.
    """
    if time < time_0:
        return x_0
    elif time > time_f:
        return x_f
    else:
        T = time_f - time_0
        t = time - time_0
        a0 = x_0
        a1 = x_dot_0
        a2 = (3*(x_f - x_0) - T*(2*x_dot_0 + x_dot_f)) / (T**2)
        a3 = (-2*(x_f - x_0) + T*(x_dot_0 + x_dot_f)) / (T**3)
        return a0 + a1*t + a2*(t**2) + a3*(t**3)

# ---------------------- Matrix Exponential/Logarithm Helpers ----------------------
def skew(v: np.ndarray) -> np.ndarray:
    """Returns the skew symmetric matrix of a 3D vector v."""
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                     [-v[1],  v[0],    0]])

def exp_rot(v: np.ndarray) -> np.ndarray:
    """
    Computes the matrix exponential of a 3D rotation vector v using Rodrigues’ formula.
    Returns the corresponding 3×3 rotation matrix.
    """
    theta = np.linalg.norm(v)
    if theta < 1e-8:
        return np.eye(3)
    u = v / theta
    K = skew(u)
    return np.eye(3) + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)

def log_rot(R: np.ndarray) -> np.ndarray:
    """
    Computes the matrix logarithm of a rotation matrix R.
    Returns the corresponding 3×3 skew-symmetric matrix S such that R = exp(S).
    """
    # Ensure R is 3x3.
    if R.ndim != 2 or R.shape != (3,3):
        raise ValueError("Input to log_rot must be a 3x3 matrix.")
    cos_theta = (np.trace(R) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if np.abs(theta) < 1e-8:
        return np.zeros((3,3))
    return (theta / (2 * np.sin(theta))) * (R - R.T)

# ---------------------- Rotation Cubic Interpolation ----------------------
def _to_rotation_matrix(rot_input: np.ndarray) -> np.ndarray:
    """
    Converts the input to a 3x3 rotation matrix.
    If the input is 1D (assumed Euler angles in 'xyz' order), convert it.
    If it's already a 3x3 matrix, return it.
    """
    if rot_input.ndim == 1:
        # Convert Euler angles (in radians) to a 3x3 rotation matrix.
        return Rotation.from_euler('xyz', rot_input, degrees=False).as_matrix()
    elif rot_input.ndim == 2 and rot_input.shape == (3,3):
        return rot_input
    else:
        raise ValueError("Rotation input must be either a 3-element vector or a 3x3 matrix.")

def rotation_cubic_full(time: float, time_0: float, time_f: float,
                        w_0: np.ndarray, a_0: np.ndarray,
                        rotation_0: np.ndarray, rotation_f: np.ndarray) -> np.ndarray:
    """
    Full version of cubic rotation interpolation.
    
    Parameters:
      - time, time_0, time_f: current, start, and final times.
      - w_0: initial angular velocity (3D vector)
      - a_0: initial angular acceleration (3D vector)
      - rotation_0, rotation_f: initial and final rotations (either as Euler angles [3] or 3×3 matrices).
      
    Computes:
      r_skew = log( rotation_0ᵀ * rotation_f )    [a 3×3 skew matrix],
      then extracts r (the rotation vector) as:
          r = [ r_skew(2,1), r_skew(0,2), r_skew(1,0) ]
      with c = w_0, b = a_0/2, and a = r - b - c.
      The interpolated rotation is then:
          rotation_interp = rotation_0 * exp( skew(a*tau³ + b*tau² + c*tau) )
    """
    if time < time_0:
        return _to_rotation_matrix(rotation_0)
    if time >= time_f:
        return _to_rotation_matrix(rotation_f)
    tau = (time - time_0) / (time_f - time_0)
    
    R0 = _to_rotation_matrix(rotation_0)
    Rf = _to_rotation_matrix(rotation_f)
    # Compute relative rotation log.
    S = log_rot(R0.T @ Rf)  # 3x3 skew matrix.
    # Extract rotation vector r from S.
    r = np.array([S[2,1], S[0,2], S[1,0]])
    c = w_0
    b = a_0 / 2
    a = r - b - c
    poly = a * (tau**3) + b * (tau**2) + c * tau
    R_interp = R0 @ exp_rot(poly)
    return R_interp

def rotation_cubic(time, time_0, time_f, rotvec_0, rotvec_f):
    """
    Interpolate between two rotation vectors using exponential map.
    Inputs:
        - rotvec_0: rotation vector (axis-angle) at t0
        - rotvec_f: rotation vector (axis-angle) at tf
    """
    R0 = Rotation.from_rotvec(rotvec_0).as_matrix()
    Rf = Rotation.from_rotvec(rotvec_f).as_matrix()

    if time <= time_0:
        return rotvec_0
    if time >= time_f:
        return rotvec_f

    tau = cubic_spline(time, time_0, time_f, 0.0, 1.0, 0.0, 0.0)  # scalar between 0 and 1
    R_rel = R0.T @ Rf  # Relative rotation
    R_rel_log = Rotation.from_matrix(R_rel).as_euler('xyz', degrees=False)
    R_interp = Rotation.from_matrix(R0 @ Rotation.from_rotvec(tau * R_rel_log).as_matrix())
    return R_interp.as_euler('xyz', degrees=False)

# ---------------------- Pose and Error Functions ----------------------
def tfmatrix_to_array(x: np.ndarray) -> np.ndarray:
    """
    Converts a 4x4 homogeneous transformation matrix into a 6D vector.
    The output is [x, y, z, rx, ry, rz] where the rotation part is represented as a rotation vector.
    """
    pos = x[:3, 3]
    R = x[:3, :3]
    # We use scipy's Rotation here for simplicity; you could also use a manual implementation.
    rotvec = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    return np.hstack((pos, rotvec))

def calc_task_error(x: np.ndarray, x_d: np.ndarray) -> np.ndarray:
    """
    Computes the error between the current end-effector pose and the desired pose,
    following the C++ approach.
    
    Parameters:
      x: Current 4x4 transformation matrix.
      x_d: Desired 6D pose vector, where:
           - x_d[0:3] are the desired position.
           - x_d[3] is desired roll,
             x_d[4] is desired pitch,
             x_d[5] is desired yaw.
    
    In C++ the desired orientation is constructed as:
      q_des = AngleAxis(desired_x[5], Z) * AngleAxis(desired_x[4], Y) * AngleAxis(desired_x[3], X)
    which we replicate here using:
      desired_orientation = Rotation.from_euler('zyx', [x_d[5], x_d[4], x_d[3]], degrees=False)
    
    The error is computed as:
      position_error = current_position - desired_position,
      orientation_error = axis * angle, where
        orientation_error = current_orientation * (desired_orientation)⁻¹.
    
    Returns:
      A 6D error vector: [position_error, orientation_error].
    """
    # Extract current position and orientation.
    pos_current = x[:3, 3]
    current_orientation = Rotation.from_matrix(x[:3, :3])
    
    # Desired position.
    pos_des = x_d[:3]
    # Convert desired Euler angles (roll, pitch, yaw) to desired orientation.
    # Note: To mimic C++ (Z * Y * X), we use the 'zyx' convention with input [yaw, pitch, roll].
    desired_orientation = Rotation.from_euler('zyx', [x_d[5], x_d[4], x_d[3]], degrees=False)
    
    # Compute position error (current minus desired).
    pos_error = pos_des - pos_current 

    # Convert current and desired orientations to quaternions.
    q_current = current_orientation.as_quat()  # [x, y, z, w]
    q_des = desired_orientation.as_quat()
    # If dot product is negative, flip current quaternion to ensure shortest path.
    if np.dot(q_current, q_des) < 0:
        q_current = -q_current
    current_orientation = Rotation.from_quat(q_current)
    
    # Compute orientation error as: q_error = current_orientation * (desired_orientation)⁻¹.
    orientation_error = desired_orientation* current_orientation.inv()
    # Convert the orientation error to a rotation vector (axis * angle).
    ori_error = orientation_error.as_euler('xyz', degrees=False)
    
    return np.hstack((pos_error, ori_error))


class ControlledThread(threading.Thread):
    """
    A custom thread class that allows controlled execution with the ability to forcibly terminate
    the thread if it exceeds a specified time limit.

    This class wraps a target function and executes it in a separate thread while holding a lock.
    The kill() method uses Python's C API to asynchronously raise a SystemExit exception in the thread,
    attempting to terminate its execution.
    """

    def __init__(self, target, *args, **kwargs):
        """
        Initializes the ControlledThread.

        Parameters:
            target (callable): The function to be executed in the thread.
            *args: Variable length argument list for the target function.
            **kwargs: Arbitrary keyword arguments for the target function.

        Returns:
            None
        """
        super().__init__()
        # Store the target function and its arguments.
        self._target = target
        self._args = args
        self._kwargs = kwargs
        
        # Lock to control access during the thread execution.
        self.calculation_lock = threading.Lock()

    def run(self):
        """
        Runs the target function within a controlled (locked) context.

        This method overrides threading.Thread.run() and acquires a lock before
        executing the target function to protect shared resources.

        Returns:
            None
        """
        # Acquire the lock to ensure the target function runs in a protected context.
        with self.calculation_lock:
            self._target(*self._args, **self._kwargs)

    def kill(self):
        """
        Attempts to terminate the thread by asynchronously raising a SystemExit exception.

        This method uses ctypes to access Python's C API and inject an exception into the thread.
        If more than one thread is affected by the exception injection, it will revert the injection.

        Returns:
            None
        """
        # Get the thread identifier.
        tid = self.ident
        if not tid:
            return
        # Attempt to asynchronously raise SystemExit in the thread.
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(tid),
            ctypes.py_object(SystemExit)
        )
        # If more than one thread was affected, revert the exception injection.
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), None)