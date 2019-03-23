from unittest import TestCase

from skyengine.flightstatus import FlightStatus


class TestFlightStatus(TestCase):
    def test_distinct(self):
        status_set = frozenset([FlightStatus.NOT_READY,
                               FlightStatus.LANDED,
                               FlightStatus.TAKING_OFF,
                               FlightStatus.LANDING,
                               FlightStatus.FLYING,
                               FlightStatus.PANICKED, ])
        self.assertEqual(len(status_set), 6)
