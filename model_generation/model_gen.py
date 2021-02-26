import math
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    FlyingStateChanged,
    SpeedChanged,
)


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


class GPSListener(olympe.EventListener):

    models = []
    user_loc = None
    user_speed = None

    # currently waiting for drone to send event messages, results in more velocity than location
    # -> find method of querying location and position periodically to generate models
    
    
    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        f = open("gps.txt", "a")
        f.write("latitude = {latitude} longitude = {longitude}\n".format(**event.args))
        f.close()
    # query and update user location here too - controller_gps sends command message to drone   
    
    @olympe.listen_event(SpeedChanged())
    def onSpeedChanged(self, event, scheduler):
        f = open("velocity.txt", "a")
        f.write("latN: {speedX}  longE: {speedY}  altD: {speedZ}\n".format(**event.args))
        f.close()
    # query and update user speed here too - controller_gps sends command message to drone

    def predictFutureUserLoc(time):
        lat_pos = user_loc[0] + time * user_speed[0]
        long_pos = user_loc[1] + time * user_speed[1]
        alt_pos = user_loc[2] - time * user_speed[2]  # positive speed downwards
        return (lat_pos, long_pos, alt_pos)
    


drone = olympe.Drone(DRONE_IP)
with GPSListener(drone) as gps_listener:
    drone.connect()
    drone(
        FlyingStateChanged(state="hovering")
        | (TakeOff() & FlyingStateChanged(state="hovering"))
    ).wait()
    drone(moveBy(10,0,0,0)).wait()
    drone(Landing()).wait()
    drone(FlyingStateChanged(state="landed")).wait()
    drone.disconnect()

