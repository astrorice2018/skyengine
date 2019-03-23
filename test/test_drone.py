import mock
from mock import patch
from dronekit import VehicleMode

from unittest import TestCase

from skyengine.drone import DroneController, FlightController
from skyengine.exceptions import AvionicsException, RicePublicRelationsException
from skyengine.singleton import Singleton
from skyengine import FlightStatus


class TestFlightController(TestCase):

    def tearDown(self):
        # "Forget" all singleton classes, so we can instantiate more than 1.
        # This should only be used for testing - do not use this in the real
        # world.
        Singleton._really_unsafe_reset_singletons(FlightController)

    @patch("skyengine.drone.connect")
    def test_connect(self, mock_connect):
        fc = FlightController(fc_address="udp:127.0.0.1:6100")
        mock_connect.assert_called_once_with("udp:127.0.0.1:6100",
                                             rate=20,
                                             wait_ready=True)
        self.assertEquals(fc.status, FlightStatus.LANDED)
        self.assertNotEquals(fc.vehicle(), None)
        # Make sure we can run another test without re-using the last singleton instance.

    @patch("skyengine.drone.connect")
    def test_connects_once(self, mock_connect):
        fc = FlightController(fc_address="...")
        mock_connect.assert_called_once_with("...",
                                             rate=20,
                                             wait_ready=True)
        # Since there's already a vehicle, connect shouldn't be called again.
        fc.connect()
        mock_connect.assert_called_once()

    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.dronekit_sitl")
    def test_sitl_default(self, mock_sitl, mock_connect):
        fc = FlightController()  # noqa: F841
        mock_connect.assert_called_once()
        mock_sitl.start_default.assert_called_once()

    @patch("skyengine.drone.connect")
    def test_read_gps(self, mock_connect):
        # Mock out a vehicle and give it a position
        vehicle = mock.MagicMock()
        vehicle.location.global_frame.lat = 10.0
        vehicle.location.global_frame.lon = -5.0
        vehicle.location.global_frame.alt = 30.0
        vehicle.location.global_relative_frame.alt = 20.0
        mock_connect.side_effect = [vehicle]

        # Connect to the vehicle and confirm that the position provided
        # is what we expect it to be
        fc = FlightController()
        location = fc.read_gps()
        self.assertEquals(location.lat, 10.0)
        self.assertEquals(location.lon, -5.0)
        self.assertEquals(location.alt, 20.0)
        self.assertEquals(location.alt_abs, 30.0)

    @patch("skyengine.drone.connect")
    def test_read_rangefinder(self, mock_connect):
        # Mock a vehicle and some sensor readings
        vehicle = mock.MagicMock()
        vehicle.rangefinder.distance = 100.0
        vehicle.rangefinder.voltage = 1.2
        mock_connect.side_effect = [vehicle]

        # Connect to the vehicle and confirm that the rangefinder readings are
        # what we expect
        fc = FlightController()
        rangefinder_data = fc.read_rangefinder()
        self.assertEquals(rangefinder_data.distance, 100.0)
        self.assertEquals(rangefinder_data.voltage, 1.2)

    @patch("skyengine.drone.connect")
    def test_read_battery(self, mock_connect):
        # Mock out a vehicle and give it battery values
        vehicle = mock.MagicMock()
        vehicle.battery.level = 90
        vehicle.battery.current = 90.5
        vehicle.battery.voltage = 15.0
        mock_connect.side_effect = [vehicle]

        # Connect to the vehicle and confirm that battery status
        # is what we expect it to be
        fc = FlightController()
        battery = fc.read_battery_status()
        self.assertEquals(battery.level, 90.0)
        self.assertEquals(battery.current, 90.5)
        self.assertEquals(battery.voltage, 15.0)

    @patch("skyengine.drone.connect")
    def test_read_battery_with_none(self, mock_connect):
        # Mock out a vehicle and give it battery values.
        # Use None for some values because some flight controllers only support voltage.
        vehicle = mock.MagicMock()
        vehicle.battery.level = None
        vehicle.battery.current = None
        vehicle.battery.voltage = 15.0
        mock_connect.side_effect = [vehicle]

        # Connect to the vehicle and confirm that the position provided
        # is what we expect it to be
        fc = FlightController()
        battery = fc.read_battery_status()
        self.assertEquals(battery.level, None)
        self.assertEquals(battery.current, None)
        self.assertEquals(battery.voltage, 15.0)


class TestDroneController(TestCase):
    def tearDown(self):
        # "Forget" all singleton classes, so we can instantiate more than 1.
        # This should only be used for testing - do not use this in the real
        # world.
        Singleton._really_unsafe_reset_singletons(DroneController)

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    def test_only_when_prevents_execution(self, mock_connect, mock_redis, mock_thread):
        drone = DroneController("")  # noqa: F841
        mock_connect.assert_called_once()
        drone.status = FlightStatus.PANICKED
        drone.take_off(10)
        self.assertEquals(drone.status, FlightStatus.PANICKED)

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    def test_takeoff(self, mock_connect, mock_redis, mock_thread):
        vehicle = mock.Mock()
        vehicle.location.global_relative_frame.alt = 10.0
        vehicle.is_armable = True
        vehicle.armed = False
        mock_connect.side_effect = [vehicle]

        drone = DroneController("")  # noqa: F841
        mock_connect.assert_called_once()
        drone.status = FlightStatus.LANDED
        drone.take_off(10.0)
        self.assertTrue(vehicle.armed)
        self.assertTrue(drone.status, FlightStatus.FLYING)

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_land(self, mock_poll, mock_connect, mock_redis, mock_thread):
        self.mock_vehicle_with(mock_poll, mock_connect)

        drone = DroneController("")
        mock_connect.assert_called_once()
        drone.take_off(10.0)
        drone.land()
        self.assertFalse(drone._vehicle.armed)
        self.assertEquals(drone.status, FlightStatus.LANDED)

    @patch("skyengine.drone.connect")
    def test_kill(self, mock_connect):
        drone = DroneController("")
        vehicle = mock.MagicMock()
        mock_connect.side_effect = [vehicle]

        drone.status = FlightStatus.LANDING
        self.assertRaises(
            AvionicsException,
            drone.kill,
        )
        self.assertEqual(drone.status, FlightStatus.LANDING)

        drone.kill(i_know_what_im_doing=True)
        self.assertEqual(drone.status, FlightStatus.LANDED)

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_goto(self, mock_poll, mock_connect, mock_redis, mock_thread):
        vehicle = self.mock_vehicle_with(mock_poll, mock_connect)
        set_home(vehicle, 0.0, 0.0, 5.0)

        drone = DroneController("")
        drone.take_off(30.0)
        move_vehicle_to(vehicle, 0., 0., 30.)
        drone.goto((0.0001, 0), 30.0, airspeed=3)
        drone.vehicle.simple_goto.assert_called_once()
        move_vehicle_to(vehicle, 0.0001, 0., 35.0)
        self.assertAlmostEqual(drone.distance_to((0.0001, 0., 35.0)), 0.0)

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_goto_validation(self, mock_poll, mock_connect, mock_redis, mock_thread):
        vehicle = self.mock_vehicle_with(mock_poll, mock_connect)
        move_vehicle_to(vehicle, 0.0, 0.0, 5.0)
        set_home(vehicle, 0.0, 0.0, 5.0)

        drone = DroneController("")
        drone.take_off(30.0)
        self.assertRaises(RicePublicRelationsException,
                          lambda: drone.goto((10.0, 0.), 30.0))
        self.assertRaises(RicePublicRelationsException,
                          lambda: drone.goto((0.0, 0.0), -1.0))

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_redis(self, mock_poll, mock_connect, mock_redis, mock_thread):
        mock_thread_instance = mock.Mock()
        mock_thread.return_value = mock_thread_instance
        drone = DroneController("")

        # Confirm Redis was initialized in SUBSCRIBE mode
        mock_redis.assert_called_once()
        drone._redis.pubsub.assert_called_once()
        drone._redis_pubsub.subscribe.assert_called_once()

        # Make sure we started a thread for the Redis listener.
        mock_thread.assert_called_once()
        drone._redis_pubsub_thread.start.assert_called_once()

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    def test_redis_panic_handler(self, mock_connect, mock_redis, mock_thread):
        drone = DroneController("")

        # Simulate receiving a panic message
        drone._redis_pubsub.listen.return_value = ["panic"]
        drone._redis_pubsub_listener()

        # Now, the DroneController instance should have set panicked state.
        self.assertEquals(drone.status, FlightStatus.PANICKED)
        self.assertTrue(drone.panic_event.is_set())

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_panic_lands(self, mock_poll, mock_connect, mock_redis, mock_thread):
        vehicle = self.mock_vehicle_with(mock_poll, mock_connect)
        drone = DroneController("")
        drone.take_off(30.0)
        drone.panic()

        self.assertEquals(vehicle.mode, VehicleMode("LAND"))

    @patch("skyengine.drone.Thread")
    @patch("skyengine.drone.StrictRedis")
    @patch("skyengine.drone.connect")
    @patch("skyengine.drone.blocking_poll")
    def test_panic_rejects_other_movement(self, mock_poll, mock_connect, mock_redis, mock_thread):
        vehicle = self.mock_vehicle_with(mock_poll, mock_connect)
        set_home(vehicle, 0.0, 0.0, 5.0)
        drone = DroneController("")
        drone.take_off(30.0)
        drone.panic()
        # Attempt some movement of the drone. These should be ignored.
        drone.goto((0.0001, 0.0001), 30)
        drone.goto_relative((0.0001, 0.0001), 30)
        drone.move_by((100.0, 100.0), 30)
        # Drone should not have actually ever issued any instruction to the FC
        # to move.
        vehicle.simple_goto.assert_not_called()

    @staticmethod
    def mock_vehicle_with(mock_poll, mock_connect):
        mock_poll.side_effect = lambda *args, **kwargs: True
        vehicle = mock.Mock()
        mock_connect.side_effect = [vehicle]
        vehicle.is_armable = True
        vehicle.armed = False
        return vehicle


def set_home(vehicle, lat, lon, alt):
    vehicle.home_location.lat = lat
    vehicle.home_location.lon = lon
    vehicle.home_location.alt = alt


def move_vehicle_to(mock_vehicle, lat, lon, alt):
    mock_vehicle.location.global_frame.lat = lat
    mock_vehicle.location.global_frame.lon = lon
    mock_vehicle.location.global_frame.alt = alt
    return mock_vehicle
