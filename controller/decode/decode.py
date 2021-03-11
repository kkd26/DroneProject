from __future__ import annotations

import asyncio
import sys
import queue
import threading
import time

import numpy as np
import olympe

from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.move import extended_move_by, extended_move_to

from curve import Curve
from common import *
from movement_controller import *


class SingleMove:
    REAL_TIME_FACTOR = 0.4
    RTF_TIMEOUT = 50

    def __init__(self, command, blocking: bool = True, duration: float = -1, **kwargs):
        self.__func = command
        self.__kwargs = kwargs
        self.__time = duration
        self.__blocking = blocking

        if "_timeout" in self.__kwargs:
            self.__kwargs["_timeout"] *= int(1 / SingleMove.REAL_TIME_FACTOR)
        else:
            self.__kwargs["_timeout"] = SingleMove.RTF_TIMEOUT
        return

    def __call__(self, drone: olympe.Drone):
        dbgprint(f"Executing Instruction Started: {self.__func}, {self.__kwargs}\n", end="")

        if not self.__blocking:
            drone(self.__func(**self.__kwargs))
            if self.__time > 0:
                time.sleep(self.__time * (1 / SingleMove.REAL_TIME_FACTOR))
        else:
            dbgprint(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")

            if self.__time > 0:
                assert drone(self.__func(**self.__kwargs)).wait().success()
                time.sleep(self.__time * (1 / SingleMove.REAL_TIME_FACTOR))

            else:
                dbgprint("\tCommencing move\n", end="")
                assert drone(self.__func(**self.__kwargs)).wait().success()

                dbgprint("\tWaiting for move to end:\n", end="")

                elapsed = 1
                while not drone(FlyingStateChanged(state="hovering")).wait().success():
                    dbgprint(f"waiting; elapsed={elapsed}\tflying state: {drone.get_state(FlyingStateChanged)['state']}\n",
                          end="")
                    elapsed += 1

                dbgprint("\tDone\n", end="")

        dbgprint(f"Executing Instruction Ended: {self.__func}, {self.__kwargs}\n", end="")
        return


class Decoder:
    def __init__(self, drone, *, use_offset: bool = False):
        self.__drone = drone
        self.__use_offset = use_offset
        self.__interrupt = threading.Event()
        self.__flush = threading.Event()
        self.__queue = queue.Queue()
        self.__terminate = threading.Event()
        self.__sample_rate = 4.
        self.__speed = 2.

        self.__interrupt.clear()
        self.__flush.clear()
        self.__terminate.clear()
        return

    def interrupt_and_flush(self):
        self.__interrupt.set()
        self.__flush.set()
        return

    def interrupt(self):
        self.__interrupt.set()
        return

    def enqueue(self, curve: Curve):
        self.__queue.put(curve)
        return

    def __move_to_start(self, *, position=None, offset=None):
        if position is not None and offset is not None:
            raise ValueError("__move_to_start: only one of position and offset may be not None")

        elif position is not None:
            move = SingleMove(command=extended_move_to,
                              latitude=position[0],
                              longitude=position[1],
                              altitude=position[2],
                              orientation_mode="to_target",
                              heading=0,
                              max_horizontal_speed=8,
                              max_vertical_speed=4,
                              max_yaw_rotation_speed=15)
            move(self.__drone)

        elif offset is not None:
            move = SingleMove(command=extended_move_by,
                              d_x=offset[0],
                              d_y=-offset[1],
                              d_z=offset[2],
                              d_psi=0,
                              max_horizontal_speed=8,
                              max_vertical_speed=4,
                              max_yaw_rotation_speed=15)
            move(self.__drone)

        else:
            raise ValueError("__move_to_start: both position and offset are None")

        return

    def __fetch_curve(self):
        try:
            return self.__queue.get_nowait()
        except queue.Empty:
            self.__terminate.set()
            return None

    def main(self):
        mc = MovementController(self.__drone)

        curve = self.__fetch_curve()
        curve_time = 0
        curve_time_increment = self.__speed / (curve.scale * self.__sample_rate)
        orientation = 0.

        if self.__use_offset:
            self.__move_to_start(offset=curve.get_point_on_curve(0.))
        else:
            self.__move_to_start(position=curve.get_point_on_curve(0.))
        next_pos = curve.get_point_on_curve(0)

        curve_fetch = False
        mc.start()

        while not self.__terminate.is_set():
            # Update current position, etc
            curve_time += curve_time_increment
            current_pos = next_pos

            if self.__interrupt.isSet():
                if self.__flush.is_set():
                    mc.flush()

                # TODO: flush anything else
                curve_fetch = True
                self.__interrupt.clear()

            if np.isclose(curve_time, 1., rtol=1e-7, atol=1e-9):
                curve_time = 1.
            elif curve_time > 1.:
                curve_fetch = True

            if curve_fetch:
                curve = self.__fetch_curve()

                if self.__terminate.is_set():
                    break

                curve_time = 0.
                curve_time_increment = self.__speed / (curve.scale * self.__sample_rate)
                curve_time += curve_time_increment
                curve_fetch = False

            if self.__flush.is_set():
                mc.suspend()
                if self.__use_offset:
                    self.__move_to_start(offset=curve.get_point_on_curve(0.))
                else:
                    self.__move_to_start(position=curve.get_point_on_curve(0.))
                self.__flush.clear()
                mc.resume()

            # main functionality

            # get next points and segments on curve
            next_pos = curve.get_point_on_curve(curve_time)

            displacement = get_displacement(current_pos, next_pos, from_gps=self.__use_offset)
            dbgprint(f"disp:{displacement}, start:{current_pos}, end:{next_pos}\n", end="")

            distance = np.linalg.norm(displacement)
            heading = (lambda x: x / np.linalg.norm(x))(displacement)

            rotation = 0.

            relative_heading = get_realigned_components(orientation + (rotation / 2.), heading)

            mc.enqueue_move(MovementControllerInstr(relative_tilts=relative_heading,
                                                    rotation=rotation,
                                                    velocity=heading * self.__speed,
                                                    absolute_direction=heading,
                                                    distance=distance * 3))

        mc.join()
        return

