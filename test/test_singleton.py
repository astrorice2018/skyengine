from unittest import TestCase

from skyengine.singleton import Singleton


class TestSingleton(TestCase):

    def test_singleton_class(self):
        s1 = TrivialSingleton(9)
        s2 = TrivialSingleton(10)
        self.assertTrue(s1 is s2)
        self.assertEquals(s2.x, 9)
        s2.x = 10
        self.assertEquals(s1.x, 10)


class TrivialSingleton(object):
    """
    A trivial singleton class to test that the Singleton metaclass works.
    """
    __metaclass__ = Singleton

    def __init__(self, x):
        self.x = x
