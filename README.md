# skyengine

Skyengine provides high-level avionics abstractions for piloting the Skynet drones.

## Usage

### DroneController

#### Basics

A `DroneController` allows you to control a drone using high-level commands.
Creating one is trivial:

```
from skyengine import DroneController
drone = DroneController("udp:127.0.0.1:6001") # Or wherever the FC is...
```

If the flight controller address is unspecified, the DroneController will default to using SITL.

`DroneControllers` are threadsafe and singleton.

#### Takeoff / Landing API

* `drone.take_off` 

* `drone.land`

#### Movement API

* `drone.goto`

