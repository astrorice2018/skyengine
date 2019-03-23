class RangefinderData(object):
    """
    Object representing data read back from the range finder sensor.

    :param distance: Distance from the sensor to the nearest detectable object.
    :param voltage: Analog voltage reading from device's sensor. Correlates to distance, but the
    correlation may not be direct. Use to tune sensor if necessary.
    """
    def __init__(self, distance=None, voltage=None):
        self.distance = distance
        self.voltage = voltage
