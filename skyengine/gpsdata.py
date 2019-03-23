class GPSData(object):
    """
    Object representing data read back from the GPS sensor.
    """

    def __init__(self, lat=None, lon=None, alt=None, alt_abs=None):
        """
        Create a container describing the current drone location.

        :param lat: Latitude (float)
        :param lon: Longitude (float)
        :param alt: Relative altitude (meters)
        :param alt_abs: Absolute altitude (meters)
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.alt_abs = alt_abs

    def __repr__(self):
        """
        Generate a string representation of this object.

        :return: Stringified representation of the drone location.
        """
        return 'GPSData(lat={lat}, lon={lon}, alt={alt}, alt_abs={alt_abs})'.format(
            lat=self.lat,
            lon=self.lon,
            alt=self.alt,
            alt_abs=self.alt_abs,
        )
