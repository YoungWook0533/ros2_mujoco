import numpy as np
import threading
import ctypes
from scipy.spatial.transform import Rotation

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
    Computes a cubic spline interpolation for a given time within an interval.

    This function performs cubic spline interpolation between an initial state x_0 at time_0 and
    a final state x_f at time_f, given the corresponding initial and final derivatives (velocities)
    x_dot_0 and x_dot_f. If the specified time is outside the interval [time_0, time_f], it returns
    the respective boundary values.

    Parameters:
        time (float): The current time at which to evaluate the spline.
        time_0 (float): The starting time of the interpolation interval.
        time_f (float): The ending time of the interpolation interval.
        x_0 (np.ndarray): The starting value (state) at time_0.
        x_f (np.ndarray): The ending value (state) at time_f.
        x_dot_0 (np.ndarray): The derivative (velocity) at time_0.
        x_dot_f (np.ndarray): The derivative (velocity) at time_f.

    Returns:
        np.ndarray: The interpolated state at the given time.
    """
    # If current time is before the start time, return the initial state.
    if time < time_0:
        return x_0
    # If current time is after the final time, return the final state.
    elif time > time_f:
        return x_f
    else:
        # Total duration of the interpolation interval.
        T = time_f - time_0
        # Elapsed time from the start.
        t = time - time_0
        # Coefficients for the cubic polynomial:
        # a0: initial state
        a0 = x_0
        # a1: initial derivative
        a1 = x_dot_0
        # a2: second coefficient computed from boundary conditions
        a2 = ( 3*(x_f - x_0) - T*(2*x_dot_0 + x_dot_f) ) / (T**2)
        # a3: third coefficient computed from boundary conditions
        a3 = ( -2*(x_f - x_0) + T*(x_dot_0 + x_dot_f) ) / (T**3)
        # Return the value of the cubic polynomial at time t.
        return a0 + a1*t + a2*(t**2) + a3*(t**3)
    
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