from __future__ import annotations

import numpy as np

DECODE_DEBUG = False


def get_basis_vectors(angle: float) -> np.ndarray:
    # Returns an array of orthogonal basis vectors according to the angle
    forward_axis = np.array([np.cos(angle), 0, np.sin(angle)])
    sideways_axis = np.array([-np.sin(angle), 0, np.cos(angle)])
    vertical_axis = np.array([0, 1, 0])
    return np.array([forward_axis, vertical_axis, sideways_axis])


def get_realigned_components(angle: float, vector: np.ndarray) -> np.ndarray:
    # Returns the components of motion according to a new coordinate
    # NOTE: this system uses x, y, z with y as the vertical axis, but
    #       the olympe api uses z as a vertical axis (pointing down):
    #       the y and z components need to be swapped before the result of this
    #       is used and the z component needs to be negated as well.
    basis_triple = get_basis_vectors(angle)
    displacements = np.array([np.dot(vector, axis) for axis in basis_triple])
    return displacements


def get_basis_vectors_from_axis(forward_axis: np.ndarray) -> np.ndarray:
    # Returns an array of orthogonal basis vectors according the forward axis
    vertical_axis = np.array([0, 1, 0])
    sideways_axis = np.cross(forward_axis, vertical_axis)
    return np.array([forward_axis, vertical_axis, sideways_axis])


def get_realigned_components_from_axis(axis: np.ndarray, vector: np.ndarray) -> np.ndarray:
    # Returns the components of motion according to a new coordinate
    # NOTE: this system uses x, y, z with y as the vertical axis, but
    #       the olympe api uses z as a vertical axis (pointing down):
    #       the y and z components need to be swapped before the result of this
    #       is used and the z component needs to be negated as well.
    basis_triple = get_basis_vectors_from_axis(axis)
    displacements = np.array([np.dot(vector, axis) for axis in basis_triple])
    return displacements


def clamp(value, minimum, maximum):
    # Clamps a value within a range [minimum, maximum]
    return min(max(value, minimum), maximum)


def get_compensation_vector(*, speeds: np.ndarray, target_speeds: np.ndarray = np.zeros(3)) -> np.ndarray:
    # Calculates the compensation vector for movement controller
    # NOTE: c is the number of times this function has been called
    #       already with the same movement instruction
    output = np.zeros(3)
    target_speeds[1] = 0.

    if not np.allclose(target_speeds, np.zeros(3)):
        # Compensation for acceleration
        for i in (0, 2):
            if not np.isclose(target_speeds[i], 0.):
                percent_speed = 100 * (speeds[i] / target_speeds[i])

                if target_speeds[i] >= 0:
                    accel_compensation = 100 - percent_speed
                else:
                    accel_compensation = percent_speed - 100

                output[i] = clamp(accel_compensation, -200., 200.)

            elif not np.isclose(speeds[i], 0., rtol=1e-3, atol=1e-2):
                accel_compensation = - speeds[i] * 100
                output[i] = clamp(accel_compensation, -200., 200.)

    # Compensation for stopping
    else:
        for i in (0, 2):
            if not np.isclose(speeds[i], 0., rtol=1e-3, atol=1e-2):
                accel_compensation = - speeds[i] * 100
                output[i] = clamp(accel_compensation, -200., 200.)

    return output


def get_horizontal_speed_estimate(speed: float) -> int:
    # Calculates the estimated tilt multiplier for the movement controller
    return -81.01381 + 85.35033 * np.power(np.e, 0.08714212 * speed)


def get_displacement(start: np.ndarray, end: np.ndarray, *, from_gps) -> np.ndarray:
    # Gets the displacement between two points, which may be specified as gps coords
    if from_gps:
        d_lat = ((end[0] - start[0]) / 360.) * np.pi * 6.371e6 * 2.
        ave_lat = (start[0] + end[0]) / 2.
        d_long = ((end[2] - start[2]) / 360.) * np.pi * (np.cos(ave_lat) * 6.371e6 * 2.)
        return np.array([d_lat, 0, d_long])
    else:
        return end - start


def dbgprint(*args, **kwargs):
    if DECODE_DEBUG:
        print(*args, **kwargs)
