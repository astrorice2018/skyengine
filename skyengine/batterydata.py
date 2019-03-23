class BatteryData(object):
    """
    Object representing battery status as measured by power module and reported by dronekit.
    More or less equivalent to a dronekit.battery object, so I'm not sure if this is
    even worth creating.
    """
    def __init__(self, voltage, current=None, level=None):
        """
        :param voltage: Voltage in millivolts.
        :param current: Current in 10 * mA (None if unsupported).
        :param level: Remaining battery level as a percent (100 is maximum) (None if unsupported).
        """
        self.voltage = voltage
        self.current = current
        self.level = level
