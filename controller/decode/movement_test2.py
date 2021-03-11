from __future__ import annotations

import sys
import queue
import time

import olympe

from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged


from curve import Curve
from decode import *


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

    """ DO STUFF HERE """

    a = lambda x, y: np.array([x, 2, y])

    d = Decoder(drone, use_offset=True)

    d.enqueue(Curve(np.array([a(-10, 0), a(-10, -5), a(-5, -5), a(-5, 0)])))
    d.enqueue(Curve(np.array([a(-5, 0), a(-5, 5), a(0, 5)])))
    d.enqueue(Curve(np.array([a(0, 5), a(5, 5)])))
    d.enqueue(Curve(np.array([a(5, 5), a(8, 8), a(4, -4), a(0, -8)])))
    d.enqueue(Curve(np.array([a(0, -8), a(-2, -10), a(-6, -4), a(-6, 0)])))

    d.main()

    """ END DO STUFF """

    time.sleep(5)
    print(f"flying state: {drone.get_state(FlyingStateChanged)['state']}\n", end="")

    assert drone(Landing(_timeout=RTF_COMPENSATED_TIMEOUT)).wait().success()
    drone.disconnect()
    return


if __name__ == "__main__":
    main()
