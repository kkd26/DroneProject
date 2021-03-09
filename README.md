# DroneProject

## Build

```
docker build --rm -f "Dockerfile" -t mbyzhang/alpha:latest "."
```

## Run

Start the simulator first, then run

```
docker run -it --rm --network host mbyzhang/alpha
```

To start the development server for the WebGUI, run

```
cd web-app
npm run build:dev
```

Then visit http://localhost:8080 to control the drone.
