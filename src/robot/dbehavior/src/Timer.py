import rospy
import time

class Timer(object):
    def __init__(self, timeTarget = 0):
        """
        @param timeTarget: target time in seconds
        @type timeTarget: float
        """
        self._start = rospy.get_time()
        self.timeTarget = timeTarget

    def restart(self):
        self._start = rospy.get_time()

    def elapsed(self):
        """
        Get elapsed time since start as secs
        @rtype: float
        """
        return rospy.get_time() - self._start

    def finished(self):
        """
        Returns if timeTarget sec has passed
        @rtype: bool
        """
        return self.elapsed() >= self.timeTarget

    def sleep(self, duration):
        time.sleep(duration)

    def leftshift(self, time):
        self._start -= time
