"""
This module defines two classes, FlightController and DroneController,
which are responsible for encapsulating stateful flight controller behaviors
and drone flight behaviors.

DroneController inherits from FlightController - if really fine-grained operations
on the FC are required for a very "unusual" mission, then a FlightController can be
directly instantiated in place of a DroneController. However, for any missions we've
described, a DroneController is probably preferable.
"""
from functools import wraps
from multiprocessing import Event, RLock
from threading import Thread

import dronekit_sitl
from dronekit import LocationGlobal, VehicleMode, connect, mavutil
from redis import StrictRedis

from skyengine.batterydata import BatteryData
from skyengine.concurrent import blocking_poll, sharing_attrs, synchronized
from skyengine.exceptions import AvionicsException, RicePublicRelationsException
from skyengine.flightstatus import FlightStatus
from skyengine.geometry import airborne_haversine_distance, offset2latlon
from skyengine.gpsdata import GPSData
from skyengine.rangefinderdata import RangefinderData
from skyengine.singleton import Singleton

# 1/2 mile -- if a drone wanted to suddenly travel farther than this, something's amiss.
_MINIMUM_UNREASONABLE_DISTANCE_METERS = 800

# Anything below 3 meters is potentially dangerous.
_MAXIMUM_UNREASONABLE_ALTITUDE = 3.0

# Communication rate with the flight controller
_FC_UPDATE_RATE_HZ = 20


@sharing_attrs("_status")
class FlightController(object):
    """
    Singleton class defining useful abstractions for interacting with the flight controller.
    """

    __metaclass__ = Singleton
    _locks = {
        "arm": RLock(),
        "connect": RLock(),
        "status": RLock(),
        }

    def __init__(self, fc_address=None, arm_timeout_sec=60, loc_start=None):
        """
        Connect to the flight controller and create a new Drone.

        :param fc_address: Address of the flight controller.
        :param arm_timeout_sec: Maximum time it should take to arm the drone
          before we give up.
        :param loc_start: A (lat,lon) pair used to initialize the GPS location of the drone when
          using SITL.
        """
        self._status = FlightStatus.NOT_READY
        self.fc_address = fc_address
        self._arm_timeout = arm_timeout_sec
        self._sitl = None
        self._vehicle = None
        self._loc_start = loc_start if not fc_address else None

        # Connect to the flight controller.
        self.connect()
        # The drone should be on the ground.
        self.status = FlightStatus.LANDED

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, new_status):
        """
        Once the status is PANICKED, prohibit setting other statuses.
        """
        if self._status is not FlightStatus.PANICKED:
            self._status = new_status

    @property
    def vehicle(self):
        """
        Getter for the 'vehicle', DroneKit's interface to the physical drone.
        If no connection is established, then this function guarantees that one
        will be attempted.
        """
        if self._vehicle is None:
            self.connect()
        return self._vehicle

    @synchronized(_locks["arm"])
    def ensure_armed(self):
        """
        Attempt to arm the vehicle if it is not already.
        Do NOT perform this step until a flight mode has been selected.

        :param timeout: The time after which we give up on arming the drone.
        """
        # If the vehicle is already armed, we're done.
        if self.vehicle.armed is True:
            return

        # Make sure the drone is ready to be armed.
        armable = blocking_poll(lambda: self.vehicle.is_armable, 0.5, self._arm_timeout)
        if not armable:
            raise AvionicsException("Drone is not armable after {}".format(self._arm_timeout))

        self._vehicle.armed = True
        # Arm the drone and wait for it to acknowledge being armed.
        arm_success = blocking_poll(lambda: self._vehicle.armed,
                                    0.25,
                                    timeout_sec=self._arm_timeout)
        if not arm_success:
            raise AvionicsException(
                "Asked drone to arm, but it refused. "
                "Could not arm within {}".format(self._arm_timeout)
                )

    @synchronized(_locks["connect"], _locks["status"])
    def connect(self):
        """
        Establish a connection with the vehicle's flight controller.
        If no connection string was specified, assume we're using SITL.

        This function also "downloads commands" from the flight controller.
        """
        # If a simultaneous connect attempt follows a successful one, ignore it.
        if self._vehicle is not None:
            return

        # Set up SITL if we're using it.
        if self.fc_address is None:
            if self._loc_start:
                lat, lon = self._loc_start
                self._sitl = dronekit_sitl.start_default(lat=lat, lon=lon)
            else:
                self._sitl = dronekit_sitl.start_default()
            self.fc_address = self._sitl.connection_string()

        # Connect to the flight controller.
        self._vehicle = connect(self.fc_address,
                                rate=_FC_UPDATE_RATE_HZ,
                                wait_ready=True)

        # "Download commands" from the flight controller.
        # The fact that this is separate from the dronekit connect() function is puzzling.
        # Many common tasks (eg, reading the starting altitude of the drone) will fail until
        # we do so.

        # Repeatedly download commands until the vehicle acquires a home location.
        blocking_poll(
            self.poll_home_location,
            2,
            10
        )

    def configure_rangefinder(self, rangefinder_type, max_cm):
        """
        Configure the flight controller's connected range finder.

        TODO: At the moment, this only supports configuration of the settings needed for the range
        finder hardware we use. This will need to be updated in order to fully support different
        range finding sensors.

        :param rangefinder_type: Integer indicating the type of range finder hardware. To disable
        the range finder set this parameter to 0. For a list of compatable hardwares and their type
        numbers, see the link below.
        http://ardupilot.org/copter/docs/common-rangefinder-landingpage.html

        :param max_cm: The maximum distance for which the range finder should be considered a source
        of truth for altitude readings.
        """
        self.vehicle.parameters['RNGFND_TYPE'] = rangefinder_type
        self.vehicle.parameters['RNGFND_MAX_CM'] = max_cm

    def read_rangefinder(self):
        """
        Get the current range finder info.

        :return: A RangefinderData object describing the current distance and voltage reported by
        DroneKit.
        """
        data = self.vehicle.rangefinder
        return RangefinderData(data.distance, data.voltage)

    def read_gps(self):
        """
        Fetch the current location.
        :return: a GPSData object describing the current coordinates reported by
        DroneKit.
        """
        location = self.vehicle.location.global_frame
        location_rel = self.vehicle.location.global_relative_frame
        return GPSData(location.lat, location.lon, location_rel.alt, location.alt)

    def read_battery_status(self):
        """
        Fetch the battery status.
        :return: A BatteryData object containing current battery status.
        """
        battery = self.vehicle.battery
        # There should be a more pythonic way of doing this?
        if battery.level is not None:
            # Cast battery level to a float to be consistent with other typing.
            return BatteryData(battery.voltage, battery.current, float(battery.level))
        else:
            return BatteryData(battery.voltage, battery.current, battery.level)

    def poll_home_location(self):
        """
        Downloads the vehicle commands and returns the home location if it has been set.

        :return: The home_location of the vehicle. May be None.
        """
        cmds = self._vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return self._vehicle.home_location

    def cleanup(self):
        """
        Stop SITL (if started) and close the connection to the vehicle (if opened).
        """
        if self._sitl is not None:
            self.sitl.stop()
        if self._vehicle is not None:
            self._vehicle.close()


# XXX: Ideally, this would be a staticmethod of the Drone class.
#      Unfortunately, trying to use a staticmethod on a class inside that class
#      as a decorator results in a TypeError in Python 2. So, it's a standalone
#      function now.
def _only_when(*args):
    """
    A decorator which will cause certain flight operations to be ignored
    unless the vehicle is in a particular mode. For example,
    ```
    @_only_when(FlightStatus.FOO, FlightStatus.BAR)
    def some_flight_method(self):
        ...
    ```
    would guarantee that Drone.some_flight_method() will be ignored unless
    the drone has status FlightStatus.FOO or FlightStatus.BAR.

    This function can be used to prevent invalid transitions on the Drone
    state machine.

    :param args: A list of FlightStatuses in which the command makes sense
    te perform.
    """
    permissible_statuses = frozenset(args)

    # Actual decorator created by _only_when.
    def _decorator(func):
        @wraps(func)
        def _wrapped_instance_method(*args, **kwargs):
            # Needs a "self" to check status
            if len(args) is 0:
                return
            if not isinstance(args[0], DroneController):
                return
            # Status must be one of the listed statuses.
            if args[0].status not in permissible_statuses:
                return
            return func(*args, **kwargs)
        return _wrapped_instance_method
    return _decorator


class DroneController(FlightController):
    """
    High-level drone-control abstractions, building on top of those for just the
    flight controller.
    """
    __metaclass__ = Singleton

    def __init__(self, *args, **kwargs):
        # Triggered when the drone is emergency panicked to end all in-progress blocking polls
        self.panic_event = Event()
        # Triggered when the drone is killed during landing to end all in-progress blocking polls
        self.kill_event = Event()

        # Connect to local Redis and subscribe to the skyengine_panic channel.
        self._redis = StrictRedis()
        self._redis_pubsub = self._redis.pubsub()
        self._redis_pubsub.subscribe("skyengine_panic")
        self._redis_pubsub_thread = Thread(target=self._redis_pubsub_listener)
        self._redis_pubsub_thread.daemon = True
        self._redis_pubsub_thread.start()

        super(DroneController, self).__init__(*args, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.LANDED)
    def take_off(self, height_meters):
        """
        Ascend to a given height, in meters. Blocks until complete.

        :param height_meters: Height, in meters, to take off to.
        """
        self.vehicle.mode = VehicleMode("GUIDED")
        self.status = FlightStatus.TAKING_OFF
        self.ensure_armed()
        self.vehicle.simple_takeoff(height_meters)
        # Wait until we've reached the correct height.
        blocking_poll(
            lambda: self.vehicle.location.global_relative_frame.alt >= 0.95 * height_meters,
            0.25,
            abort_event=self.panic_event)
        self.status = FlightStatus.FLYING

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING, FlightStatus.PANICKED)
    def land(self):
        """
        Land the drone. Blocks until complete.
        """
        self.vehicle.mode = VehicleMode("LAND")
        self.status = FlightStatus.LANDING
        # Make sure the drone is *actually* landed before we declare that it's
        # actually landed.
        blocking_poll(
            lambda: self.vehicle.location.global_relative_frame.alt <= 0.1,
            0.25,
            timeout_sec=120,
            abort_event=self.kill_event,
        )
        self.status = FlightStatus.LANDED

    @_only_when(FlightStatus.LANDING, FlightStatus.LANDED)
    def kill(self, i_know_what_im_doing=False):
        """
        Disarm the drone while landing. This is equivalent to manually flipping a kill switch.

        :param i_know_what_im_doing: Specify this as True if you know what you're about to do and
                                     have already ensured in your application logic that this is
                                     safe to execute.
        """
        if not i_know_what_im_doing:
            raise AvionicsException(
                'You must specify the i_know_what_im_doing flag to kill the drone.'
            )

        self.kill_event.set()

        # Setting the `armed` property on the vehicle will only disarm the drone if it is already
        # landed. This is a (very sane) safety mechanism built into ArduPilot.
        # ArduPilot exposes the ability to forcefully disarm the motors while in any state by
        # passing a deliberately magic number to the MAVLink MAV_CMD_COMPONENT_ARM_DISARM command.
        # There is no abstraction in the vehicle to provide this functionality, so this can only be
        # accomplished by manually constructing and sending that MAV message.
        disarm_msg = self.vehicle.message_factory.command_long_encode(
            target_system=0,
            target_component=0,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0,  # 0 for disarm; 1 for arm.
            param2=21196,  # Magic number to allow emergency disarming during flight.
            # Remaining parameters are unused.
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )
        self.vehicle.send_mavlink(disarm_msg)

        self.status = FlightStatus.LANDED

    def distance_to(self, abs_position):
        """
        For a (lat, lon, alt) triple, find an approximate distance between that point and the
        drone's current location.

        :param abs_position: The (lat, lon, alt) triple to find the distance to. This is an
          absolute position, with altitude measured from sea level.
        :return: the distance to that abs_position
        """
        frame = self.vehicle.location.global_frame
        return airborne_haversine_distance((frame.lat, frame.lon, frame.alt), abs_position)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def goto(self, coords, altitude, airspeed=1, radius=2.5, timeout_sec=None):
        """
        Blocking function that tells the drone to go somewhere.
        The drone must be flying and asked to go somewhere sane.

        :param coords: A (lat, lon) double; both specified in an absolute reference frame.
        :param altitude: An altitude specified relative to the height the drone acquired its GPS
          fix at.
        :param airspeed: The speed, in m/s, that the drone should travel through these
          air.
        :param radius: The maximum distance (in meters) the drone is permitted to be
          from the position specified before considering itself arrived.
        :param timeout_sec: The length of time the drone has to get there.
        """
        # Make sure we're off to someplace "sane."
        lat, lon = coords
        home_altitude = self.vehicle.home_location.alt
        position = (lat, lon, altitude + home_altitude)
        self._validate_position(position)

        # Tell the drone to go there.
        self.vehicle.simple_goto(LocationGlobal(*position), airspeed=airspeed)

        # Wait for the drone to arrive.
        arrived = blocking_poll(
            lambda: self.distance_to(position) <= radius,
            0.25,
            timeout_sec=timeout_sec,
            abort_event=self.panic_event
            )
        if not arrived:
            raise AvionicsException("Drone could not make it to {}.".format(position))

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def goto_relative(self, coords, altitude, **kwargs):
        """
        Blocking function that tells the drone to go somewhere, relative to its
        home position (ie, where it first acquired the GPS lock).
        All keyword arguments are identical to the goto() function.

        :param coords: A (delta_lat, delta_lon) double, both specified relative
          to the drone's starting location.
        :param altitude: An altitude relative to the drone's starting location to
          go to.
        """
        delta_lat, delta_lon = coords
        home = self.vehicle.home_location
        self.goto((home.lat + delta_lat, home.lon + delta_lon), altitude, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def move_by(self, offset, altitude, **kwargs):
        """
        Blocking function that moves the drone from its current position by
        an offset specified in meters, and to an altitude specified relative to
        the drone's home location.

        :param offset: A (meters_north, meters_east) pair specifying where the
          drone should move relative to its current location.
        :param altitude: The desired altitude for the drone, relative to the
          elevation at the drone's home location.
        """
        m_north, m_east = offset
        here = self.read_gps()
        d_lat, d_lon = offset2latlon(m_north, m_east, here.lat)
        self.goto((here.lat + d_lat, here.lon + d_lon), altitude, **kwargs)

    @synchronized(FlightController._locks["status"])
    @_only_when(FlightStatus.FLYING)
    def elevate(self, altitude, **kwargs):
        """
        Blocking funciton that moves the drone to an altitude, specified relative
        to the drone's home location. The drone should move only vertically.

        :param altitude: An altitude, in meters, that the drone should fly to.
        :param **kwargs: Keyword arguments forwarded to `DroneController.goto`.
        """
        self.move_by((0.0, 0.0), altitude, **kwargs)

    def panic(self):
        """
        Panic a drone. This aborts the flight immediately, causes the drone to
        land, and makes it unresponsive to any further movement-related commands
        issued through the DroneController.

        This should be used only in circumstances where Something Very Bad (TM)
        occurs, as requires that the drone be stopped immediately.

        The panic event will be sent to all DroneController instances through
        Redis.
        """
        # Publishing the 'panic' message to the Redis 'skyengine_panic' channel
        # will trigger _panic_handler, which will perform the actual panic actions.
        self._redis.publish("skyengine_panic", "panic")

        # Land the drone.
        self.land()

    def _redis_pubsub_listener(self):
        """
        Listener action for "panic" messages communicated over Redis.
        This function sets the panic event and cancels any movement commands.
        """
        for msg in self._redis_pubsub.listen():
            # We may receive 'subscribe'-type messages to indicate that someone
            # has subscribed. Make sure that we only react to 'message' type messages
            # so that we don't trigger the panic event when other processes merely subscribe!
            if msg['type'] == 'message':
                # Abort all polling events by triggering the panic event.
                self.panic_event.set()
                # Alter the status.
                self.status = FlightStatus.PANICKED

    def _validate_position(self, position):
        """
        Guarantee that moving the drone to a new position is "sane." Raise a
        RicePublicRelationsException if the position provided is suspect.
        :param position: a (lat, lon, altitude) 3-tuple containing the target position
        of the drone.
        """

        # Having the drone run away is not a university-sanctioned recreational activity.
        if self.distance_to(position) >= _MINIMUM_UNREASONABLE_DISTANCE_METERS:
            raise RicePublicRelationsException(
                "Drone tried to run away to {} (presumably the location of a Peer "
                "Institution).".format(position)
                )

        # Make sure our flying machine isn't trying to become a digging machine.
        _, _, requested_alt = position
        if (requested_alt - self.vehicle.home_location.alt) < _MAXIMUM_UNREASONABLE_ALTITUDE:
            raise RicePublicRelationsException(
                "Flying a drone underground is a poor use of university resources."
                )
