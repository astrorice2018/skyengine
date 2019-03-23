import mock
from unittest import TestCase
from multiprocessing import Event

from skyengine.concurrent import synchronized, blocking_poll
from skyengine.exceptions import FlightAbortedException


class TestConcurrent(TestCase):

    def test_synchronized(self):
        lock = mock.Mock()

        @synchronized(lock)
        def plusone(x):
            return x + 1
        self.assertEquals(plusone(1), 2)
        lock.acquire.assert_called_once()
        lock.release.assert_called_once()

        self.assertEquals(plusone(2), 3)
        self.assertEquals(lock.acquire.call_count, 2)
        self.assertEquals(lock.release.call_count, 2)

    @mock.patch("skyengine.concurrent.time")
    def test_blocking_poll_timeout(self, mock_time):
        # Make a blocking poll that times out
        mock_predicate = mock.Mock()
        mock_predicate.side_effect = lambda: False
        mock_time.time.side_effect = [0, 0.3, 2.0]
        self.assertFalse(blocking_poll(mock_predicate, 0.25, timeout_sec=1.0))
        # Was the predicate actually used?
        self.assertEqual(mock_predicate.call_count, 2)
        # Did we delay like we were supposed to?
        mock_time.sleep.assert_called_once_with(0.25)
        # Did we timeout?
        self.assertEqual(mock_time.time.call_count, 3)

    @mock.patch("skyengine.concurrent.time")
    def test_blocking_poll_success(self, mock_time):
        # Make a blocking poll that succeeds.
        mock_predicate = mock.Mock()
        mock_predicate.side_effect = lambda: True
        mock_time.time.side_effect = [0, 0.3]
        # Should report success
        self.assertTrue(blocking_poll(mock_predicate, 0.25, timeout_sec=0.1))
        # Should get an initial time
        mock_time.time.assert_called_once()
        # Should actually use the predicate
        mock_predicate.assert_called_once()
        # Should skip a useless "sleep"
        mock_time.sleep.assert_not_called()

    @mock.patch("skyengine.concurrent.time")
    def test_blocking_poll_aborts(self, mock_time):
        mock_predicate = mock.Mock()
        mock_predicate.side_effect = lambda: True
        mock_time.side_effect = lambda: 0.0

        abort_event = Event()
        abort_event.set()
        with self.assertRaises(FlightAbortedException):
            blocking_poll(mock_predicate, 0.25, abort_event=abort_event)

        # Shouldn't even have run the command in question if the abort event
        # was triggered!
        mock_predicate.assert_not_called()
