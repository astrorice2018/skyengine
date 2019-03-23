from unittest import TestCase

from skyengine.gpsdata import GPSData


class TestGPSData(TestCase):
    def test_repr(self):
        self.assertEqual(
            str(GPSData(1.0, 2.0, 3.0, 4.0)),
            'GPSData(lat=1.0, lon=2.0, alt=3.0, alt_abs=4.0)',
        )
