from __future__ import annotations
import numpy as np

""" TODO: maybe rewrite to pre-calculate and store things at construction time """


class Curve:
    """ points is a list of points that define the curve,
        the first element should be the start, and last element the destination """

    def __init__(self, points: np.ndarray[np.ndarray, ...]):
        self.points = points
        self.order = len(points) - 1

        if not 1 <= self.order <= 3:
            raise ValueError

        self.length = self.__generate_length()
        self.scale = self.length if self.length != 0 else 0.000001
        return

    def get_point_on_curve(self, t: float) -> np.ndarray:
        """ t is the parametric curve parameter, between 0.0 and 1.0 """
        if self.order == 1:
            return (((1. - t) * self.points[0])
                    + (t * self.points[1]))
        elif self.order == 2:
            return ((((1. - t) ** 2.) * self.points[0])
                    + (2 * (1. - t) * t * self.points[1])
                    + ((t ** 2.) * self.points[2]))
        elif self.order == 3:
            return ((((1. - t) ** 3.) * self.points[0])
                    + (3 * ((1. - t) ** 2.) * t * self.points[1])
                    + (3 * (1. - t) * (t ** 2.) * self.points[2])
                    + ((t ** 3.) * self.points[3]))
        else:
            raise ValueError

    def get_first_derivative(self, t: float) -> np.ndarray:
        """ t is the parametric curve parameter, between 0.0 and 1.0 """
        if self.order == 1:
            return self.points[1] - self.points[0]
        elif self.order == 2:
            return ((2 * (1 - t) * (self.points[1] - self.points[0]))
                    + (2 * t * (self.points[2] - self.points[1])))
        elif self.order == 3:
            return ((3 * ((1 - t) ** 2) * (self.points[1] - self.points[0]))
                    + (6 * (1 - t) * t * (self.points[2] - self.points[1]))
                    + (3 * (t ** 2) * (self.points[3] - self.points[2])))
        else:
            raise ValueError

    def get_second_derivative(self, t: float) -> np.ndarray:
        """ t is the parametric curve parameter, between 0.0 and 1.0 """
        if self.order == 1:
            return np.array([0, 0, 0])
        elif self.order == 2:
            return 2 * (self.points[2] - (2 * self.points[1]) + self.points[0])
        elif self.order == 3:
            return ((6 * (1 - t) * (self.points[2] - (2 * self.points[1]) + self.points[0]))
                    + (6 * t * (self.points[3] - (2 * self.points[2]) + self.points[1])))
        else:
            raise ValueError

    def __generate_length(self) -> float:
        """ Calculate the length of the curve """
        CURVE_SAMPLES = 1000

        if self.order == 1:
            return np.linalg.norm(self.points[1] - self.points[0])
        elif self.order == 2 or self.order == 3:
            samples = np.linspace(start=0., stop=1., num=CURVE_SAMPLES+1)
            points = np.array([self.get_point_on_curve(i) for i in samples])
            chord_lengths = np.array([np.linalg.norm(points[i + 1] - points[i]) for i in range(CURVE_SAMPLES)])
            return np.sum(chord_lengths)
        else:
            raise ValueError
