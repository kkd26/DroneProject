# DroneProject

## Prerequisites

* Parrot-Sphinx
* Docker (for ROS modules)
* **Recursive** clone of this repository

To quickly install Docker, run

```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

## Build

### Building the controller

```
docker build --rm -f "Dockerfile" -t mbyzhang/alpha:latest "."
```

The controller contains a copy of prebuilt web-app (in `alpha_webgui/www`). To build the web app, see `web-app/README.md`.

## Run

### Step 1: Run the simulator

```
./sim/launch_sim.sh
```

Wait for it to spin up (until you see "All drones instantiated" in the console.)

### Step 2: Run the controller

```
docker run -it --rm --network host mbyzhang/alpha
```

Then visit http://localhost:8000 to control the simulated drone.

