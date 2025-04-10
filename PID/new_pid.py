"""
Author: Victor Cruz de Oliveira
Date: 19-03-2025
Description: PID controller implementation
"""

from typing import Callable, Any, NoReturn

class PID:
    """A simple PID controller implementation"""
    
    def __init__(
        self,
        Kp: float = 1,
        Ki: float = 0,
        Kd: float = 0,
        *,
        setpoint: float = 0,
        sample_time: float = 0,
        output_limits: tuple[float] = (None, None),
        time_fn: Callable[[Any], float] = None,
        starting_output: float = 0
    ):
        """
        @brief Initialize a new PID controller instance.

        @param Kp: Proportional gain. Default is 1.
        @param Ki: Integral gain. Default is 0.
        @param Kd: Derivative gain. Default is 0.
        @param setpoint: The desired value that the PID controller will try to achieve. Default is 0.
        @param sample_time: Time in seconds between each update of the PID controller. Default is 0.
        @param output_limits: A tuple specifying the minimum and maximum output limits. Default is (None, None).
        @param time_fn: Function to get the current time. If None, time.time() will be used. Default is None.
        @param starting_output: The initial output value of the PID controller. Default is 0.
        """
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.starting_output = starting_output

        self._last_time = None
        self._last_error = None
        
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        
        if time_fn is not None:
            self.time_fn = time_fn
        else:
            import time
            
            self.time_fn = time.monotonic
            
        self._last_time = self.time_fn()
        self._last_output = None
        self._last_error = None
        self._last_input = None
            
        self._integral = self.clamp(starting_output, output_limits)
        
    @staticmethod
    def clamp(value: float, limits: tuple[float, float]) -> float:
        """
        @brief Clamp a value between specified lower and upper limits.

        This method checks if the given value exceeds the upper limit or falls below the lower limit,
        and returns the corresponding limit if that's the case. If the value is within the limits,
        it returns the value unchanged. If the value is None, it returns None.

        @param value The value to be clamped.
        @param limits A tuple containing the lower and upper limits (lower, upper).
        @return The clamped value or None if the input value is None.
        """
        
        if value is None:
            return None

        lower, upper = limits
        match limits:
            case (None, None):
                return value
            case (None, upper):
                return upper if value > upper else value
            case (lower, None):
                return lower if value < lower else value
            case (lower, upper):
                if value < lower:
                    return lower
                if value > upper:
                    return upper
                return value
            case _:
                raise ValueError("Limits must be a tuple of size 2")
            
    def update(self, feedback: float, dt=None) -> float:
        
        time_now = self.time_fn()
        if dt is None:
            dt = time_now - self._last_time 
        elif dt <= 0:
            raise ValueError(f'dt has negative value {dt}, it must be positive')
            
        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - feedback
        d_input = feedback - (self._last_input if (self._last_input is not None) else feedback)
        d_error = error - (self._last_error if (self._last_error is not None) else error)

        self._proportional = self.Kp * error

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = self.clamp(self._integral, self.output_limits)  # Avoid integral windup

        self._derivative = self.Kd * d_error / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = self.clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = feedback
        self._last_error = error
        self._last_time = time_now

        return output
            
    def reset(self) -> None:
        """
        @brief Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = self.clamp(self._integral, self.output_limits)

        self._last_time = self.time_fn()
        self._last_output = None
        self._last_input = None
        self._last_error = None
        
    @property
    def components(self) -> tuple[float, float, float]:
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative
    
    @property
    def tunings(self) -> tuple[float, float, float]:
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd
    
    @tunings.setter
    def tunings(self, tunings) -> None:
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings
        
    @property
    def output_limits(self) -> tuple[float, float]:
        """
        The current output limits as a 2-tuple: (lower, upper).

        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return (None, None)
    
    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')
   

import time       
import matplotlib.pyplot as plt          
  
# For testing
if __name__ == "__main__":
    controller = PID(0.1, 2, 1.3, setpoint=10)
    x_arr = []
    t_arr = []
    x = 1
    
    time_last = time_0 = time.monotonic()
    for i in range(100):
        time_now = time.monotonic()
        dt = time_now - time_last
        x = controller.update(x, dt)
        time_last = time_now
        x_arr.append(x)
        t_arr.append(time_now - time_0)
        
        
    plt.figure(figsize=(8, 5))
    plt.plot(t_arr, x_arr, label="System Output")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.title("Simple PID Controller")
    plt.legend()
    plt.grid(True)
    plt.show()