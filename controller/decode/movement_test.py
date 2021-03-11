from __future__ import annotations

import sys
import time

import olympe

from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

from movement_controller import *


def to_unit(x: np.ndarray):
    return x / np.linalg.norm(x)


def main():
    DRONE_IP = "192.168.56.101"
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    if not drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=30)
                 ).wait().success():
        print("\nERROR: Could not take off\n", file=sys.stderr)
        drone.disconnect()
        return

    print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")
    time.sleep(5)
    """ BEGIN TEST CODE HERE """

    mc = MovementController(drone)

    mc.start()
    print("move queueing started\n", end="")

    mc.enqueue_move(MovementControllerInstr(relative_tilts=np.array([1., 0., 0.]),
                                            rotation=0.,
                                            velocity=np.array([2., 0., 0.]),
                                            absolute_direction=np.array([1, 0., 0.]),
                                            distance=10.))

    mc.enqueue_move(MovementControllerInstr(relative_tilts=np.array([0., 0., 1.]),
                                            rotation=0.,
                                            velocity=np.array([0., 0., 2.]),
                                            absolute_direction=np.array([0., 0., 1.]),
                                            distance=10.))

    mc.enqueue_move(MovementControllerInstr(relative_tilts=to_unit(np.array([1., 0., 1.])),
                                            rotation=0.,
                                            velocity=np.array([2., 0., 2.]),
                                            absolute_direction=to_unit(np.array([1, 0., 1.])),
                                            distance=10.))

    mc.enqueue_move(MovementControllerInstr(relative_tilts=to_unit(np.array([1., 0., -1.])),
                                            rotation=0.,
                                            velocity=np.array([2., 0., -2.]),
                                            absolute_direction=to_unit(np.array([1., 0., -1.])),
                                            distance=10.))

    print("move queueing ended\n", end="")
    mc.join()
    print("move waiting ended\n", end="")

    """ END TEST CODE HERE """
    # time.sleep(50)
    print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")

    assert drone(Landing()).wait().success()
    drone.disconnect()
    return


if __name__ == "__main__":
    main()
