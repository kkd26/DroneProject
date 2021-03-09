from __future__ import annotations

import math
import queue
import threading
import time

import numpy as np
import olympe

from olympe.messages.ardrone3.Piloting import PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged, PositionChanged

from common import *


HIGH_COMPENSATION_THRESHOLD = True  # Change to False for production


class MovementControllerInstr:
    def __init__(self, relative_tilts: np.ndarray, rotation: float, velocity: np.ndarray,
                 absolute_direction: np.ndarray, distance: float, endpoint: np.ndarray):
        self.drone_tilts = relative_tilts  # (unit vector)
        self.planar_rotation = rotation
        self.target_velocity = velocity  # (relative to the normal axes)
        self.abs_speed_direction = absolute_direction  # (unit vector or zero)
        self.distance = distance
        self.target_location = endpoint
        return


def generate_compensation_vector(speeds: np.ndarray, target: np.ndarray = np.array([0., 0., 0.])) -> np.ndarray:
    output = np.array([0., 0., 0.])

    for i in (0, 2):
        if math.isclose(target[i], 0.):
            if abs(speeds[i]) < 0.01:
                output[i] = 0.
            else:
                if speeds[i] > 0.:
                    output[i] = clamp(-(speeds[i] * 50), -200., 200.)
                else:
                    output[i] = clamp(speeds[i] * 50, -200., 200.)
        else:
            percent_speed = 100 * (speeds[i] / target[i])
            output[i] = clamp(100 - percent_speed, -200., 200.)

    output /= 10
    output_helper = np.array([abs(e) for e in output])

    return output * output_helper


class MovementController(threading.Thread):
    def __init__(self, drone, *, name=None, daemon=None):
        super().__init__(target=self.run, name=name, daemon=daemon)
        self.__drone = drone
        self.__moves = queue.Queue()

        # Terminate & Awake:
        # term = 0, awake = 0: pause normal execution and block until awake is set
        # term = 0, awake = 1: resume normal execution
        # term = 1, awake = 0: [avoid this state]
        # term = 1, awake = 1: unblock if necessary and terminate thread after completing current instruction
        # self.flush
        self.__terminate = threading.Event()
        self.__awake = threading.Event()

        self.__idle = threading.Event()
        self.__cancel_instruction = threading.Event()

        self.__terminate.clear()
        return

    SLEEP_TIME = 0.05  # TODO: change back to 0.05 for production usage

    def run(self):
        self.__awake.set()
        running = True
        horizontal_tilt_multiplier = 50
        vertical_tilt_multiplier = 100
        current_orientation = 0  # (facing positive x)
        iteration_start = time.time_ns()
        idle_count = 0

        while running:
            while self.__awake.is_set() and not self.__terminate.is_set():
                # Main loop body

                if self.__cancel_instruction.is_set() or self.__moves.empty():
                    idle_count += 1
                    if idle_count > 100:
                        self.__idle.set()

                    # update velocity and location
                    velocity = self.__drone.get_state(SpeedChanged)
                    velocity = np.array([velocity['speedX'], -velocity['speedZ'], velocity['speedY']])
                    gps_loc = self.__drone.get_state(PositionChanged)

                    # realign components
                    actual_velocity = get_realigned_components(current_orientation, velocity)

                    # adjust compensation tilt
                    compensation = generate_compensation_vector(speeds=actual_velocity)

                    # send pcmd command
                    print(f"idle... , speed:{velocity}, compensation:{compensation}, "
                          f"pos:{[gps_loc['latitude'], gps_loc['longitude'], gps_loc['altitude']]}\n", end="")

                    self.__drone(PCMD(flag=1,
                                      roll=clamp(int(compensation[2]), -100, 100),
                                      pitch=clamp(int(compensation[0]), -100, 100),
                                      yaw=0,
                                      gaz=-clamp(int(compensation[1]), -100, 100),
                                      timestampAndSeqNum=0))

                    # Timing control
                    diff = float(time.time_ns() - iteration_start) / 1e9
                    time.sleep(max(MovementController.SLEEP_TIME - diff, 0))
                    iteration_start = time.time_ns()

                else:
                    print("starting move\n", end="")
                    idle_count = 0
                    self.__idle.clear()

                    # fetch new instruction
                    current_instr = self.__moves.get_nowait()
                    target_velocity = current_instr.target_velocity
                    relative_tilts = current_instr.drone_tilts
                    reached_target = False
                    distance_travelled = 0.

                    # send PCMD messages until move has been completed
                    while not reached_target:
                    
                        # exit inner loop if instruction is cancelled
                        if self.__cancel_instruction.is_set():
                            break
                        
                        # update velocity and location
                        velocity = self.__drone.get_state(SpeedChanged)
                        velocity = np.array([velocity['speedX'], -velocity['speedZ'], velocity['speedY']])
                        gps_loc = self.__drone.get_state(PositionChanged)
                        distance_travelled += np.linalg.norm(np.dot(velocity, current_instr.abs_speed_direction)) / 20.

                        # heading = get heading somehow
                        heading = 0
                        average_rot = 0

                        # realign components
                        actual_velocity = get_realigned_components(current_orientation, velocity)

                        # compensate
                        compensation = generate_compensation_vector(speeds=actual_velocity, target=target_velocity)

                        # send pcmd command
                        print(f"distance:{distance_travelled}, speed:{actual_velocity}, target:{target_velocity}, " 
                              f"compensation:{compensation},  "
                              f"pos:{[gps_loc['latitude'], gps_loc['longitude'], gps_loc['altitude']]}\n", end="")

                        self.__drone(PCMD(
                            flag=1,
                            roll=clamp(int((horizontal_tilt_multiplier * relative_tilts[2]) + compensation[2]), -100, 100),
                            pitch=clamp(int((horizontal_tilt_multiplier * relative_tilts[0]) + compensation[0]), -100, 100),
                            yaw=0,
                            gaz=-clamp(int((horizontal_tilt_multiplier * relative_tilts[1]) + compensation[1]), -100, 100),
                            timestampAndSeqNum=0)
                        )

                        # Timing control
                        diff = float(time.time_ns() - iteration_start) / 1e9
                        time.sleep(max(MovementController.SLEEP_TIME - diff, 0))
                        iteration_start = time.time_ns()

                        # Check if destination reached
                        if math.isclose(distance_travelled, current_instr.distance) \
                                or distance_travelled > current_instr.distance:
                            print(f"distance")
                            reached_target = True
                            distance_travelled = 0.

                    self.__moves.task_done()
                    print("move done\n", end="")

            self.__idle.set()

            if self.__terminate.is_set():
                running = False
            else:
                self.__awake.wait()
        return

    def suspend(self):
        self.__awake.clear()
        return

    def resume(self):
        self.__awake.set()
        return

    def flush(self):
        self.__cancel_instruction.set()
        while not self.__moves.empty():
            self.__moves.get_nowait()
        self.__cancel_instruction.clear()
        return

    def enqueue_move(self, move: MovementControllerInstr):
        self.__moves.put(move)

    def join(self, timeout=None):
        self.__idle.wait()
        self.__awake.set()
        self.__terminate.set()
        super().join(timeout)
