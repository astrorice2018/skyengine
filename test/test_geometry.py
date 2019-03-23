from unittest import TestCase

from skyengine.geometry import haversine_distance


class TestGeometry(TestCase):
    def test_haversine(self):
        # All ground truth values established using
        # http://www.nhc.noaa.gov/gccalc.shtml
        self.check_haversine((0.0, 0.0), (0.0, 0.0), 0.0)
        self.check_haversine((1.0, 0.0), (0.0, 0.0), 111000.0)
        self.check_haversine((0.0, 1.0), (0.0, 0.0), 111000.0)
        self.check_haversine((-1.0, 0.0), (0.0, 0.0), 111000.0)
        self.check_haversine((0.0, -1.0), (0.0, 0.0), 111000.0)
        self.check_haversine((13.0, -7.0), (19.0, 2.0), 1169000.0)

    def check_haversine(self, loc1, loc2, distance):
        calc_distance = haversine_distance(loc1, loc2)
        # Because 1 degree is a huge, but easy to work with distance, and given
        # that our radius of earth isn't that accurate, give ourselves 2000m (<2%)
        # of 'wiggle room' for these tests.
        self.assertTrue(calc_distance + 2000 >= distance and
                        calc_distance - 2000 <= distance)
