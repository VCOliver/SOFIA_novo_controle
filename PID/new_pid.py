"""
Author: Victor Cruz de Oliveira
Date: 19-03-2025
Description: PID controller implementation
"""

from typing import Callable, Any

class PID:
    """A simple PID controller implementation"""
    
    def __init__(
        self,
        Kp: float = 1,
        Ki: float = 0,
        Kd: float = 0,
        setpoint: float = 0,
        sample_time: float = 0,
        output_limits: tuple[float] = (None, None),
        time_fn: Callable[[Any], float] = None,
        starting_output: float = 0
    ):
        """
        Initialize a new PID controller instance.

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
        self.time_fn = time_fn
        self.starting_output = starting_output

        self._last_time = None
        self._last_error = None
        
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        
        self._output = starting_output