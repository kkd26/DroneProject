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

""" TEST CODE: """
DRONE_IP = "192.168.56.101"
# DRONE_IP = "192.168.122.107"
REAL_TIME_FACTOR = 0.4  # TODO: change back to 1 for production
RTF_COMPENSATED_TIMEOUT = 50  # TODO: change back to 10 for production
""" END TEST CODE: """

def get_displacement_from_gps(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    # TODO: calculate the displacement between start and end

    return end - start  # TODO: remove this later


def get_angle_from_gps(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    # TODO: calculate the displacement between start and end

    return np.zeros(3)  # TODO: remove this later


class SingleMove:
    def __init__(self, command, blocking: bool = True, duration: float = -1, **kwargs):
        self.__func = command
        self.__kwargs = kwargs
        self.__time = duration
        self.__blocking = blocking

        if "_timeout" in self.__kwargs:
            self.__kwargs["_timeout"] *= int(1 / REAL_TIME_FACTOR)
        else:
            self.__kwargs["_timeout"] = RTF_COMPENSATED_TIMEOUT
        return

    def __call__(self, drone: olympe.Drone):
        print(f"Executing Instruction Started: {self.__func}, {self.__kwargs}\n", end="")

        if not self.__blocking:
            drone(self.__func(**self.__kwargs))
            if self.__time > 0:
                time.sleep(self.__time * (1 / REAL_TIME_FACTOR))
        else:
            print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")

            if self.__time > 0:
                assert drone(self.__func(**self.__kwargs)).wait().success()
                time.sleep(self.__time * (1 / REAL_TIME_FACTOR))

            else:
                print("\tCommencing move\n", end="")
                assert drone(self.__func(**self.__kwargs)).wait().success()

                print("\tWaiting for move to end:\n", end="")

                elapsed = 1
                while not drone(FlyingStateChanged(state="hovering")).wait().success():
                    print(f"waiting; elapsed={elapsed}\tflying state: {drone.get_state(FlyingStateChanged)['state']}\n",
                          end="")
                    elapsed += 1

                print("\tDone\n", end="")

        print(f"Executing Instruction Ended: {self.__func}, {self.__kwargs}\n", end="")
        return


class Decoder:
    def __init__(self, drone):
        self.__drone = drone
        self.__interrupt = threading.Event()
        self.__flush = threading.Event()
        self.__sample_rate = 4.
        self.__speed = 5.

        self.__interrupt.clear()
        self.__flush.clear()
        return

    def interrupt_and_flush(self):
        self.__interrupt.set()
        self.__flush.set()
        return

    def interrupt(self):
        self.__interrupt.set()
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
                              d_x=10,
                              d_y=0,
                              d_z=0,
                              d_psi=0,
                              max_horizontal_speed=8,
                              max_vertical_speed=4,
                              max_yaw_rotation_speed=15)
            move(self.__drone)

        else:
            raise ValueError("__move_to_start: both position and offset are None")

        return

    def __fetch_curve(self):
        # TODO: fetch the curve from alex's code
        # (maybe add a curve source as a member variable for Decode)

        # The below code is temporary:
        return Curve(np.array([np.array([10, 2, 0]), np.array([10, 2, 10]), np.array([0, 2, 10])]))

    def main(self):
        mc = MovementController(self.__drone)

        curve = self.__fetch_curve()
        curve_time = 0
        curve_time_increment = self.__speed / (curve.scale * self.__sample_rate)
        orientation = 0.

        # TEMP:
        # self.__move_to_start(offset=np.array([0, 10, 0]))

        # self.__move_to_start(position=curve.get_point_on_curve(0))
        next_pos = curve.get_point_on_curve(0)

        loop = True
        curve_fetch = False
        preserve_time = False
        mc.start()

        while loop:
            # Update current position, etc
            curve_time += curve_time_increment
            current_pos = next_pos

            if self.__interrupt.isSet():
                if self.__flush.is_set():
                    mc.flush()

                # TODO: flush anything else
                curve_fetch = True
                self.__interrupt.clear()

            if math.isclose(curve_time, 1., rel_tol=1e-7, abs_tol=1e-9):
                curve_time = 1.
            elif curve_time > 1.:
                curve_fetch = True
                preserve_time = True

            if curve_fetch:
                break  # Temp

                curve = self.__fetch_curve()
                curve_time = 0.
                curve_time_increment = self.__speed / (curve.scale * self.__sample_rate)

                if preserve_time:
                    curve_time += curve_time_increment
                    preserve_time = False

                curve_fetch = False

            if self.__flush.is_set():
                mc.suspend()
                self.__move_to_start(position=curve.get_point_on_curve(0.))
                self.__flush.clear()
                mc.resume()

            # main functionality

            # get next points and segments on curve
            next_pos = curve.get_point_on_curve(curve_time)

            displacement = get_displacement_from_gps(current_pos, next_pos)
            print(f"disp:{displacement}, start:{current_pos}, end:{next_pos}\n", end="")

            distance = np.linalg.norm(displacement)
            heading = (lambda x: x / np.linalg.norm(x))(displacement)

            rotation = 0.

            relative_heading = get_realigned_components(orientation + (rotation / 2.), heading)

            mc.enqueue_move(MovementControllerInstr(relative_tilts=relative_heading,
                                                    rotation=rotation,  # TEMP
                                                    velocity=heading * self.__speed,
                                                    absolute_direction=heading,
                                                    distance=distance,
                                                    endpoint=next_pos))

        mc.join()
        return


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    if not drone(TakeOff(_timeout=RTF_COMPENSATED_TIMEOUT)
                 >> FlyingStateChanged(state="hovering", _timeout=3 * RTF_COMPENSATED_TIMEOUT)
                 ).wait().success():
        print("\nERROR: Could not take off\n", file=sys.stderr)
        drone.disconnect()
        return

    print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")
    time.sleep(5)

    """ DO STUFF HERE """

    d = Decoder(drone)
    d.main()

    """ END DO STUFF """

    time.sleep(5)
    print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")

    assert drone(Landing(_timeout=RTF_COMPENSATED_TIMEOUT)).wait().success()
    drone.disconnect()
    return


if __name__ == "__main__":
    main()
