from __future__ import annotations

import asyncio
from asyncio import get_event_loop

import bezier.curve

from .squish import interpolate


class Curve:
    """
    Do something like

    for c in path_generator:
        move_drone_along_path(c)
        # while that's happening, wait until the curve either completes or is invalidated
        try:
            c.sleep(1)
        except CancelledError:
            pass
    """
    def __init__(self, xs: [float], ys: [float]):
        self.points = bezier.curve.Curve([xs, ys], degree=2)
        self.future = get_event_loop().create_future()

    def __eq__(self, other: Curve) -> bool:
        # two curves are equal if they are defined by the same points
        # the weird notation is because eq is overloaded by numpy
        return (self.points.nodes == other.points.nodes).all()

    def cancel_sleep(self):
        self.future.cancel()
        self.future = get_event_loop().create_future()

    async def sleep(self, delay):
        """
        sleeps for `delay` seconds, unless the curve updates,
        in which case we raise a CancelledError
        """
        async def return_after(fut, delay_):
            await asyncio.sleep(delay_)
            fut.done()

        get_event_loop().create_task(return_after(self.future, delay))


class PathGenerator:
    def __init__(self, model, ground_route: [Curve]):
        self.model = model
        self.ground_route = ground_route
        self.proposed_route = ground_route.copy()
        self.index = 0

    def __iter__(self):
        return self

    def __next__(self) -> Curve:
        try:
            self.index += 1
            return self.proposed_route[self.index - 1]
        except IndexError:
            raise StopIteration

    async def model_change_callback(self):
        """would be nice to have the info as to what changed, eg
          - which entity(s)?
          - which attribute(s)?
          - by how much?
        """

        # we moved, and so did the target
        # question 1: where are we on the curve now (assuming we followed it correctly?)
        # currently unsure, will solve soon. Dummy value of "at the start" used for now
        t = 0

        # part 2: warp the curve to better follow the target
        c = self.proposed_route[self.index]
        new_c = interpolate(c, t, self.model.target)  # this takes ~0.1s, could be run in executor if needed

        # part 3: assuming the target moved off the path, invalidate the old curve and push the new one
        self.proposed_route[self.index] = new_c
        self.index -= 1
        c.cancel_sleep()
