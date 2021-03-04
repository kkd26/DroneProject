from typing import Optional

import olympe
from sanic import Sanic
from sanic import exceptions
from sanic.response import text

import generate_path
from model_generation.model_gen import GPSListener


class GodObject:
    def __init__(self, drone_ip, path):
        self.drone = olympe.Drone(drone_ip)

        self.model_gen = GPSListener(self.drone)
        self.model_gen.subscribe()

        self.generator = generate_path.PathGenerator(model=self.model_gen.model, ground_route=path)

    def handle_gps_data(self, data):
        # pass this to the model generation, when they have a stable api
        pass


app = Sanic(name='Drone Controller')
controller: Optional[GodObject] = None


@app.post("/api/start")
async def start(request):
    # connect to drone
    # init other modules and start their main loops
    # and save them in global scope i guess
    global controller
    if controller is not None:
        raise exceptions.InvalidUsage("Drone is already in flight!")
    controller = GodObject(drone_ip='10.202.0.1', path=request.json['path'])
    return text('OK')


@app.post("/api/gps")
async def gps(request):
    if controller is None:
        # just discard gps data until the drone starts flying
        return
    controller.handle_gps_data(request.json)


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8000)
