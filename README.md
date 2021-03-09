# DroneProject

## Prerequisites

* Parrot-Sphinx
* Docker (for ROS modules)
* Node.js and NPM (for the web app)
* **Recursive** clone of this repository

To install Docker, run

```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

## Build

```
docker build --rm -f "Dockerfile" -t mbyzhang/alpha:latest "."
```

## Run

Start the simulator first, then run

```
docker run -it --rm --network host mbyzhang/alpha
```

To start the development server for the web app, run

```
cd web-app
npm run build:dev
```

Then visit http://localhost:8080 to control the drone.
