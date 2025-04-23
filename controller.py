import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sgnl

class MyComplex(complex):
    def __new__(cls, real=0.0, imag=0.0):
        return super().__new__(cls, real, imag)

    def __str__(self):
        sign = '+' if self.imag >= 0 else '-'
        return f"({self.real} {sign} j{abs(self.imag)})"

    def __repr__(self):
        return self.__str__()

class TransferFunction:
    def __init__(self, b: list[float], a: list[float] =[1]):
        self._b: list[float] = b[:] # Numerator
        self._a: list[float] = a[:] # Denominator
        self.normalize()
        self._zeros = list(np.roots(self._b))
        self._poles = list(np.roots(self._a))
        self.reset()

    def normalize(self):
        if not self._a:
            raise ValueError("Denominator cannot be empty.")
        if self._a[0] == 0:
            raise ValueError("Leading denominator coefficient cannot be zero.")
        factor = self._a[0]
        if factor != 1.0:
            self._a = [ai / factor for ai in self._a]
            self._b = [bi / factor for bi in self._b]

    def reset(self):
        self.u_history = [0.0] * len(self._b)
        self.y_history = [0.0] * (len(self._a) - 1)

    def __call__(self, u):
        self.u_history = [u] + self.u_history[:-1]
        y = sum(bi * ui for bi, ui in zip(self._b, self.u_history))
        y -= sum(ai * yi for ai, yi in zip(self._a[1:], self.y_history))
        y_out = y
        self.y_history = [y_out] + self.y_history[:-1]
        return y_out

    def __add__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        a1, b1 = self._a, self._b
        a2, b2 = other._a, other._b
        a = self._convolve(a1, a2)
        b = self._add_lists(self._convolve(b1, a2), self._convolve(b2, a1))
        return TransferFunction(b, a)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        a1, b1 = self._a, self._b
        a2, b2 = other._a, other._b
        a = self._convolve(a1, a2)
        b = self._add_lists(self._convolve(b1, a2), self._convolve([-bi for bi in b2], a1))
        return TransferFunction(b, a)

    def __rsub__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        return other.__sub__(self)

    def __mul__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        b = self._convolve(self._b, other._b)
        a = self._convolve(self._a, other._a)
        return TransferFunction(b, a)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        b = self._convolve(self._b, other._a)
        a = self._convolve(self._a, other._b)
        return TransferFunction(b, a)

    def __rtruediv__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        return other.__truediv__(self)
    
    def __str__(self):
        def poly_to_str(coeffs):
            terms = []
            degree = len(coeffs) - 1
            for i, coef in enumerate(coeffs):
                power = degree - i
                if coef == 0:
                    continue
                if power == 0:
                    term = f"{coef:g}"
                elif power == 1:
                    term = f"{coef:g}·s"
                else:
                    term = f"{coef:g}·s^{power}" if coef != 1 else f's^{power}'
                terms.append(term)
            return " + ".join(terms) if terms else "0"

        num_str = poly_to_str(self._b)
        den_str = poly_to_str(self._a)

        bar = "―" * max(len(num_str), len(den_str))
        bar = '\t' + bar
        num_str = '\t' + num_str.center(len(bar))
        den_str = '\t' + den_str.center(len(bar))
        return f"{num_str}\n{bar}\n{den_str}"

    def __repr__(self):
        return f"<TransferFunction {self.__str__()}>"

    @staticmethod
    def _convolve(x, y):
        res = [0.0] * (len(x) + len(y) - 1)
        for i in range(len(x)):
            for j in range(len(y)):
                res[i + j] += x[i] * y[j]
        return res

    @staticmethod
    def _add_lists(x, y):
        size = max(len(x), len(y))
        x = x + [0.0] * (size - len(x))
        y = y + [0.0] * (size - len(y))
        return [xi + yi for xi, yi in zip(x, y)]
    
    def to_dict(self):
        return {'num': self._b, 'den': self._a}
    
    @property
    def num(self):
        return TransferFunction(self._b)
    
    @property
    def den(self):
        return TransferFunction(self._a)
    
    @property
    def zeros(self):
        return [float(zero) for zero in self._zeros]
    
    @property
    def poles(self):
        return [float(pole) for pole in self._poles]


class Controller:
    def __init__(self, Gs: TransferFunction, Hs: TransferFunction, Kc=1.0):
        self.Gs = Gs
        self.Hs = Hs
        self.Kc = Kc
        self._update_loop()

    def _update_loop(self):
        closed_loop = (self.Kc * self.Gs) / (1 + self.Kc * self.Gs * self.Hs)
        self._loop_tf = closed_loop

    def reset(self):
        self._loop_tf.reset()

    def __call__(self, reference: float) -> float:
        return self._loop_tf(reference)

    def print_poles_zeros(self, filename=None):
        zeros = np.roots(self._loop_tf._b[::-1])
        poles = np.roots(self._loop_tf._a[::-1])
        zeros_rounded = [MyComplex(round(z.real, 2), round(z.imag, 2)) for z in zeros]
        poles_rounded = [MyComplex(round(p.real, 2), round(p.imag, 2)) for p in poles]
        print("Zeros:", zeros_rounded)
        print("Poles:", poles_rounded)

        if filename:
            with open(filename, 'w') as f:
                f.write("Zeros: " + str(zeros_rounded) + "\n")
                f.write("Poles: " + str(poles_rounded) + "\n")
                
    @property
    def tf(self):
        return self._loop_tf

TF = TransferFunction
if __name__ == "__main__":
    G = TF([0.3], [1.0, 1.2, 0.36])
    
    print('G(s) = ')
    print(G)
    print()
    print(G.poles, end='\n\n')
    
    wn = 2
    H = (TF([1, 2*wn, wn**2]) - G.den)/G.num
    print('H(s) = ')
    print(H, end='\n\n')

    controller = Controller(G, H, Kc=2.0)
    tf = controller.tf.to_dict()
    #system = sgnl.TransferFunction(tf['num'], tf['den'])
    print(controller.tf)
    
    # t, y = sgnl.step(system)

    controller.print_poles_zeros("poles_zeros.txt")

    setpoint = 1.0
    y = 0.0

    t_values = [0]
    y_values = [y]

    for t in np.linspace(0.04, 12, 299):
        y = controller(setpoint)
        t_values.append(t)
        y_values.append(y)

    print(f'Error from setpoint to end result: {setpoint-y}')
    plt.figure(figsize=(10, 5))
    plt.plot(t_values, y_values, label="Output")
    # plt.plot(t, y, label="Step Response")
    plt.axhline(setpoint, color='r', linestyle='--', label="Setpoint")
    plt.title("Closed-Loop Step Response")
    plt.xlabel("Time [s]")
    plt.ylabel("Output")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("step_response.png")
    plt.show()

