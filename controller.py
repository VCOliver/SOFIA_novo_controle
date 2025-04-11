import matplotlib.pyplot as plt
import numpy as np

class MyComplex(complex):
    def __new__(cls, real=0.0, imag=0.0):
        return super().__new__(cls, real, imag)

    def __str__(self):
        sign = '+' if self.imag >= 0 else '-'
        return f"({self.real} {sign} j{abs(self.imag)})"

    def __repr__(self):
        return self.__str__()

class TransferFunction:
    def __init__(self, b, a):
        self.b = b[:]
        self.a = a[:]
        self.normalize()
        self.reset()

    def normalize(self):
        if not self.a:
            raise ValueError("Denominator cannot be empty.")
        if self.a[0] == 0:
            raise ValueError("Leading denominator coefficient cannot be zero.")
        factor = self.a[0]
        if factor != 1.0:
            self.a = [ai / factor for ai in self.a]
            self.b = [bi / factor for bi in self.b]

    def reset(self):
        self.u_history = [0.0] * len(self.b)
        self.y_history = [0.0] * (len(self.a) - 1)

    def __call__(self, u):
        self.u_history = [u] + self.u_history[:-1]
        y = sum(bi * ui for bi, ui in zip(self.b, self.u_history))
        y -= sum(ai * yi for ai, yi in zip(self.a[1:], self.y_history))
        y_out = y
        self.y_history = [y_out] + self.y_history[:-1]
        return y_out

    def __add__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        a1, b1 = self.a, self.b
        a2, b2 = other.a, other.b
        a = self._convolve(a1, a2)
        b = self._add_lists(self._convolve(b1, a2), self._convolve(b2, a1))
        return TransferFunction(b, a)

    def __radd__(self, other):
        return self.__add__(other)

    def __mul__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        b = self._convolve(self.b, other.b)
        a = self._convolve(self.a, other.a)
        return TransferFunction(b, a)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        b = self._convolve(self.b, other.a)
        a = self._convolve(self.a, other.b)
        return TransferFunction(b, a)

    def __rtruediv__(self, other):
        if not isinstance(other, TransferFunction):
            other = TransferFunction([other], [1.0])
        return other.__truediv__(self)

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


class Controller:
    def __init__(self, Gs: TransferFunction, Hs: TransferFunction, Kc=1.0):
        self.Gs = Gs
        self.Hs = Hs
        self.Kc = Kc
        self._update_loop()

    def _update_loop(self):
        closed_loop = (self.Kc * self.Gs) / (1 + self.Kc * self.Gs * self.Hs)
        self.loop_tf = closed_loop

    def reset(self):
        self.loop_tf.reset()

    def __call__(self, reference: float) -> float:
        return self.loop_tf(reference)

    def print_poles_zeros(self, filename=None):
        zeros = np.roots(self.loop_tf.b[::-1])
        poles = np.roots(self.loop_tf.a[::-1])
        zeros_rounded = [MyComplex(round(z.real, 2), round(z.imag, 2)) for z in zeros]
        poles_rounded = [MyComplex(round(p.real, 2), round(p.imag, 2)) for p in poles]
        print("Zeros:", zeros_rounded)
        print("Poles:", poles_rounded)

        if filename:
            with open(filename, 'w') as f:
                f.write("Zeros: " + str(zeros_rounded) + "\n")
                f.write("Poles: " + str(poles_rounded) + "\n")


if __name__ == "__main__":
    G = TransferFunction([0.3], [1.0, -1.2, 0.36])
    H = TransferFunction([-0.4], [1.03, 0.15])

    controller = Controller(G, H, Kc=0.4)

    controller.print_poles_zeros("poles_zeros.txt")

    setpoint = 1.0
    y = 0.0

    t_values = [y]
    y_values = [0]

    for t in range(1, 300):
        y = controller(setpoint)
        t_values.append(t)
        y_values.append(y)

    print(f'Error from setpoint to end result: {setpoint-y}')
    plt.figure(figsize=(10, 5))
    plt.plot(t_values, y_values, label="Output")
    plt.axhline(setpoint, color='r', linestyle='--', label="Setpoint")
    plt.title("Closed-Loop Step Response")
    plt.xlabel("Time Steps")
    plt.ylabel("Output")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("step_response.png")
    plt.show()

