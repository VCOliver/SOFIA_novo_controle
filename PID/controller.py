# out = (in - out*Kh) * Kg
# out = in*Kg - out*Kh*Kg
# out + out*Kh*Kg = in*Kg
# out(1 + Kh*Kg) = in*Kg

import numpy as np
import matplotlib.pyplot as plt

class Controller:
    def __init__(self, Kh=1, Kg=1) -> None:
        self._Kh = Kh
        self._Kg = Kg

    def __call__(self, error: float) -> float:
        """Compute control signal based on current error"""
        num = error * self._Kg
        out = num / (1 + self._Kh * self._Kg)
        return out

    def __str__(self):
        return f'{self._Kh=}\n{self._Kg=}'
    
    @property
    def Kg(self) -> float:
        return self._Kg
    
    @Kg.setter
    def Kg(self, value: float) -> None:
        self._Kg = value
        
    @property
    def Kh(self) -> float:
        return self._Kh
    
    @Kh.setter
    def Kh(self, value: float) -> None:
        self._Kh = value

if __name__ == "__main__":
    controller = Controller(Kh=0.5, Kg=2.0)
    print(controller)

    setpoint = 10
    in_ = 0             
    dt = 0.1              
    T_end = 60            

    t_values = [0]
    x_values = [in_]

    steps = int(T_end // dt)
    for step in range(steps):
        time = step * dt

        error = setpoint - in_       # feedback error
        u = controller(error)        
        in_ += u * dt                # Plant simulation

        # Save values for plotting
        t_values.append(time)
        x_values.append(in_)

    # Plot results
    plt.figure(figsize=(10, 5))
    plt.plot(t_values, x_values, label="Current value")
    plt.plot(t_values, [setpoint] * len(t_values), 'r--', label="Setpoint")
    plt.title("Control System Simulation")
    plt.xlabel("Time (s)")
    plt.ylabel("Input / Control")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
