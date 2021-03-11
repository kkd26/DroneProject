from __future__ import annotations

import queue
import threading
import time

import numpy as np
import olympe

from olympe.messages.ardrone3.Piloting import PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged, PositionChanged

from common import *


class MovementControllerInstr:
    def __init__(self, relative_tilts: np.ndarray, rotation: float, velocity: np.ndarray,
                 absolute_direction: np.ndarray, distance: float):
        self.drone_tilts = relative_tilts  # (unit vector)
        self.planar_rotation = rotation
        self.target_velocity = velocity  # (relative to the normal axes)
        self.abs_speed_direction = absolute_direction  # (unit vector or zero)
        self.distance = distance
        return


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
        self.__terminate = threading.Event()
        self.__awake = threading.Event()

        self.__idle = threading.Event()
        self.__cancel_instruction = threading.Event()

        self.__terminate.clear()
        return

    SLEEP_TIME = 0.05

    def run(self):
        self.__awake.set()
        running = True
        current_orientation = 0  # (facing positive x)
        iteration_start = time.time_ns()
        idle_count = 0
        relative_tilts = np.zeros(3)

        while running:
            while self.__awake.is_set() and not self.__terminate.is_set():
                # Main loop body

                if self.__cancel_instruction.is_set() or self.__moves.empty():
                    idle_count += 1
                    if idle_count == 50:
                        relative_tilts = np.zeros(3)
                    if idle_count == 100:
                        self.__idle.set()

                    # update velocity and location
                    velocity = self.__drone.get_state(SpeedChanged)
                    velocity = np.array([velocity['speedY'], -velocity['speedZ'], -velocity['speedX']])

                    # adjust compensation tilt
                    compensation = get_compensation_vector(speeds=velocity)

                    # send pcmd command
                    dbgprint(f"idle... , speed:{velocity}, compensation:{compensation}\n", end="")

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
                    dbgprint("starting move\n", end="")
                    idle_count = 0
                    self.__idle.clear()

                    previous_tilt = relative_tilts

                    # fetch new instruction
                    current_instr = self.__moves.get_nowait()
                    target_velocity = get_realigned_components(current_orientation, current_instr.target_velocity)
                    target_speed = np.linalg.norm(target_velocity)
                    relative_tilts = current_instr.drone_tilts

                    horizontal_tilt_multiplier = get_horizontal_speed_estimate(target_speed)
                    reached_target = False
                    distance_travelled = 0.
                    count = 0

                    # compensate for previous move
                    relative_tilts_actual = relative_tilts - previous_tilt

                    # send PCMD messages until move has been completed
                    while not reached_target:
                    
                        # exit inner loop if instruction is cancelled
                        if self.__cancel_instruction.is_set():
                            break
                        
                        # update velocity and location
                        velocity = self.__drone.get_state(SpeedChanged)
                        velocity = np.array([velocity['speedY'], -velocity['speedZ'], -velocity['speedX']])

                        # update distance travelled
                        relative_velocities = get_realigned_components_from_axis(relative_tilts, velocity)
                        distance_travelled += relative_velocities[0] * MovementController.SLEEP_TIME

                        # revert relative_tilts_actual
                        if count == 20:
                            dbgprint("reverted...\n", end="")
                            relative_tilts_actual = relative_tilts

                        # heading = get heading somehow
                        heading = 0
                        average_rot = 0

                        # compensate
                        compensation = get_compensation_vector(speeds=velocity, target_speeds=target_velocity)

                        # send pcmd command
                        dbgprint(f"distance:{distance_travelled}, speed:{velocity}, target:{target_velocity}, " 
                                 f"compensation:{compensation}\n", end="")

                        self.__drone(PCMD(
                            flag=1,
                            roll=clamp(int((horizontal_tilt_multiplier * relative_tilts_actual[2]) + compensation[2]), -100, 100),
                            pitch=clamp(int((horizontal_tilt_multiplier * relative_tilts_actual[0]) + compensation[0]), -100, 100),
                            yaw=0,
                            gaz=-clamp(int((horizontal_tilt_multiplier * relative_tilts_actual[1]) + compensation[1]), -100, 100),
                            timestampAndSeqNum=0)
                        )

                        # Timing control
                        diff = float(time.time_ns() - iteration_start) / 1e9
                        time.sleep(max(MovementController.SLEEP_TIME - diff, 0))
                        iteration_start = time.time_ns()

                        # Check if destination reached
                        if np.isclose(distance_travelled, current_instr.distance) \
                                or distance_travelled > current_instr.distance:
                            dbgprint(f"distance")
                            reached_target = True
                            distance_travelled = 0.

                        count += 1

                    self.__moves.task_done()
                    dbgprint("move done\n", end="")

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
