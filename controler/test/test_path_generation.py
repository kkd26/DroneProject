import pytest
from pytest import fixture

from controler.generate_path import PathGenerator, Curve


class MockModel:
    def __init__(self):
        self.drone = [0, 1, 0]
        self.target = [1, 0, 1]


@fixture
def path():
    return [Curve([0, 1, 2], [0, 1, 0]), Curve([2, 3, 4], [0, -1, 0])]


def test_if_target_follows_path_perfectly_the_drone_also_does(path):
    out_path = PathGenerator(MockModel(), path)

    for curve in path:
        # todo move target along path

        assert curve == next(out_path)


def test_if_target_does_not_move_the_drone_also_doesnt(path):
    """is that something i can test? (does it also require decode/timing) todo ask peter"""
    assert 0


def test_drone_tracks_target_1():
    """
    the target moves in a straight line,
    but the path says the drone should oscillate (eg sin wave)
    the drone should stay closer to the target than the default path would
    (eg lower average distance from target)
    """
    assert 0


def test_drone_tracks_target_2():
    """
    the target oscillates (eg sin wave),
    but the path says the drone should move in a straight line
    the drone should stay closer to the target than the default path would
    (eg lower average distance from target)
    """
    assert 0


def main():
    pass


if __name__ == '__main__':
    main()
