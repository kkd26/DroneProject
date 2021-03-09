from __future__ import annotations

import numpy as np


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


def clamp(value, minimum, maximum):
    # Clamps a value within a range [minimum, maximum]
    return min(max(value, minimum), maximum)