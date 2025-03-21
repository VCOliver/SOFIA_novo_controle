# out = (in + out*Kh) * Kg
# out = in*Kg + out*Kh*Kg
# out - out*Kh*Kg = in*Kg
# out(1 - Kh*Kg) = in*Kg

from typing import NoReturn
import matplotlib.pyplot as plt

class Controller:
    def __init__(self, Kh=1, Kg=1) -> NoReturn:
        self._Kh = Kh
        self._Kg = Kg
        
    def __call__(self, feedback: float) -> float:
        in_ = feedback*self._Kg
        out = in_/(1 - self._Kh*self._Kg)
        return out
    
    @property
    def Kg(self) -> float:
        return self._Kg
    
    @Kg.setter
    def Kg(self, value: float) -> NoReturn:
        self._Kg = value
        
    @property
    def Kh(self) -> float:
        return self._Kh
    
    @Kg.setter
    def Kh(self, value: float) -> NoReturn:
        self._Kh = value
        
if __name__ == "__main__":
    control = Controller(-0.5, 2.5)
    x = []
    t = []
    feedback = 10
    
    for i in range(100):
        x_temp = control(feedback)
        x.append(x_temp)
        t.append(i)
        feedback = x_temp
        
    plt.figure(figsize=(8, 5))
    plt.plot(t, x, label="System Output")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.title("Simple Controller")
    plt.legend()
    plt.grid(True)
    plt.show()