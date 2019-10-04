import numpy as np
from math import atan2


class Point:

    def __init__(self, x: float = None, y: float = None, z: float = None, vector: np.ndarray = None):
        self._x = x
        self._y = y
        self._z = z
        self.vector = vector

        if None not in (self.x, self.y, self.z):
            self.vector = np.asarray([self._x, self._y, self._z, 1])
        elif self.vector is not None:
            self._x, self._y, self._z, _ = self.vector
        else:
            self._x = self._y = self._z = 0
            self.vector = np.asarray([self._x, self._y, self._z, 1])

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x):
        self._x = x
        self.vector[0] = self._x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, y):
        self._y = y
        self.vector[1] = self._y

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, z):
        self._z = z
        self.vector[2] = self._z

    def __mul__(self, other):
        return Point(vector=other.matrix.dot(self.vector))


class TMatrix:

    def __init__(self, alpha: float = None, a: float = None, d: float = None, theta: float = None,
                 modified: bool = False, radians: bool = True, matrix: np.ndarray = None):
        self._alpha = alpha
        self._a = a
        self._d = d
        self._theta = theta
        self._modified = modified
        self._radians = radians
        self.matrix = matrix

        if None not in (self._alpha, self._a, self._d, self._theta):
            self.create_matrix()
        elif self.matrix is not None:
            self._alpha, self._a, self._d, self._theta = self.get_params()
        else:
            raise Exception("ParameterError")

    def create_matrix(self):
        if self._radians:
            ct = np.cos(self._theta)
            st = np.sin(self._theta)
            ca = np.cos(self._alpha)
            sa = np.sin(self._alpha)
        elif not self._radians:
            ct = np.cos(np.deg2rad(self._theta))
            st = np.sin(np.deg2rad(self._theta))
            ca = np.cos(np.deg2rad(self._alpha))
            sa = np.sin(np.deg2rad(self._alpha))
        else:
            raise Exception("RadianError")

        if not self._modified:
            self.matrix = np.asarray([[ct, -st * ca, st * sa, self._a * ct],
                                      [st, ct * ca, -ct * sa, self._a * st],
                                      [0, sa, ca, self._d],
                                      [0, 0, 0, 1]])
        elif self._modified:
            self.matrix = np.asarray([[ct, -st, 0, self._a],
                                      [st*ca, ct*ca, -sa, -self._d*sa],
                                      [st*sa, ct*sa, ca, self._d*ca],
                                      [0, 0, 0, 1]])
        else:
            raise Exception("ModeError")

    def get_params(self):
        if not self._modified:
            d = self.matrix[2][3]
            theta = atan2(self.matrix[1][0], self.matrix[0][0])
            alpha = atan2(self.matrix[2][1], self.matrix[2][2])
            a = self.matrix[0][3] / np.cos(theta)
            if self._radians:
                return alpha, a, d, theta
            elif not self._radians:
                return np.rad2deg(alpha), np.rad2deg(a), np.rad2deg(d), np.rad2deg(theta)
            else:
                raise Exception("RadianError")

        elif self._modified:
            a = self.matrix[0][3]
            theta = atan2(-self.matrix[0][1], self.matrix[0][0])
            alpha = atan2(-self.matrix[1][2], self.matrix[2][2])
            d = self.matrix[2][3] / np.cos(alpha)
            if self._radians:
                return alpha, a, d, theta
            elif not self._radians:
                return np.rad2deg(alpha), np.rad2deg(a), np.rad2deg(d), np.rad2deg(theta)
            else:
                raise Exception("RadianError")
        else:
            raise Exception("ModeError")

    @property
    def a(self):
        return self._a

    @a.setter
    def a(self, value):
        self._a = value
        self.create_matrix()

    @property
    def alpha(self):
        return self._alpha

    @alpha.setter
    def alpha(self, value):
        self._alpha = value
        self.create_matrix()

    @property
    def d(self):
        return self._d

    @d.setter
    def d(self, value):
        self._d = value
        self.create_matrix()

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value
        self.create_matrix()

    @property
    def modified(self):
        return self._modified

    @modified.setter
    def modified(self, value):
        self._modified = value
        self.create_matrix()

    @property
    def radians(self):
        return self._radians

    @radians.setter
    def radians(self, value):
        self._radians = value
        self.create_matrix()

    def __mul__(self, other):
        if self._radians != other.radians:
            raise Exception("DivergentRadiansArgument")
        return TMatrix(matrix=self.matrix.dot(other.matrix), radians=False)

    def __eq__(self, other):
        if isinstance(other, TMatrix):
            return True if False not in (self.matrix == other.matrix)[:] else False
        else:
            return False


if __name__ == '__main__':
    t1 = TMatrix(0, 0, 0, 0, radians=False)
    t2 = TMatrix(-90, 1, 0, 45, radians=False)
    t3 = TMatrix(0, 1, 0, 0, radians=False)
    t4 = t1 * t2 * t3
