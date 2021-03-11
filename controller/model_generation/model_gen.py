import math
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    FlyingStateChanged,
    SpeedChanged,
)
from olympe.messages.ardrone3.GPSSettings import SendControllerGPS


DRONE_IP = "10.202.0.1"


class Entity:
    """gps_loc = (latitude, longitude, altitude)
       velocity is a vector (lat_vel, long_vel, alt_vel) of velocity in those three directions"""
    # maybe should also include orientation?
    def __init__(self, gps_loc, velocity):
        self.gps_loc = gps_loc
        self.velocity = velocity


class Model:
    """ drone and user are both instances of the Entity class """
    def __init__(self, drone, user):
        self.drone = drone
        self.user = user


class FlightListener(olympe.EventListener):

    # model is updated every time there is a change so most recent stats used
    model = None

    def __init__(self, drone, model):
        self.model = model
        super().__init__(self, drone)
    
    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        model.drone.gps_loc = (event.latitude, event.longitude, event.altitude)
    

    @olympe.listen_event(SpeedChanged())
    def onSpeedChanged(self, event, scheduler):
        model.drone.velocity = (event.speedX, event.speedY, event.speedZ)

    # user position gps and speed data from app - done in ros integration

    def predictFutureUserLoc(time):
        lat_pos = model.user.gps_loc[0] + time * model.user.velocity[0]
        long_pos = model.user.gps_loc[1] + time * model.user.velocity[1]
        alt_pos = model.user.gps_loc[2] - time * model.user.velocity[2]  # positive speed downwards
        return (lat_pos, long_pos, alt_pos)

    def predictFutureDroneLoc(time):
        lat_pos = model.drone.gps_loc[0] + time * model.drone.velocity[0]
        long_pos = model.drone.gps_loc[1] + time * model.drone.velocity[1]
        alt_pos = model.drone.gps_loc[2] - time * model.drone.velocity[2]  # positive speed downwards
        return (lat_pos, long_pos, alt_pos)
    


if __name__ == '__main__':
    drone = olympe.Drone(DRONE_IP)
    # model starts initialised to 0 here, updated as soon as gps data of drone available
    ent_drone = Entity((0,0,0), (0,0,0))
    ent_user = Entity((0,0,0), (0,0,0))
    model = Model(ent_drone, ent_user)
    # possible implementation for testing purposes
    with FlightListener(drone, model) as flight_listener:
        # flight_listener.model will give access to the latest coordinates and speed of user and drone
        drone.connect()
        drone(
            FlyingStateChanged(state="hovering")
            | (TakeOff() & FlyingStateChanged(state="hovering"))
        ).wait()
        drone(moveBy(10,0,0,0)).wait()
        drone(Landing()).wait()
        drone(FlyingStateChanged(state="landed")).wait()
        drone.disconnect()
        drone(FlyingStateChanged(state="landed")).wait()
        drone.disconnect()

