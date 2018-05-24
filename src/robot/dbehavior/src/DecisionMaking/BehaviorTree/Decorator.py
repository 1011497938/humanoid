from Task import Task, Status
from Timer import Timer
from Branch import parallel
from utils.mathutil import get_magnitude

def repeatSec(runs):
    """
    Executes child node N seconds, N must be positive
    """

    def wrap(child):
        class Repeat(Task):
            def __init__(self):
                super(Repeat, self).__init__()
                self.timer = Timer(runs)
                self.addChild(child)

            def tick(self):
                if self.status is not Status.RUNNING:
                    self.timer.restart()

                if len(self._children) is not 1:
                    raise Exception('Repeater should have one child, but I have {}'.format(len(self._children)))

                if not self.timer.finished():
                    self.status =  self._children[0].tick()
                    return self.status
                else:
                    self._children[0].success()
                    return self.success()

        return Repeat

    return wrap

def seeBall(child):
    class SeeBall(Task):
        def init(self):
            self.addChild(child)

        def tick(self):
            if self.bb.visionInfo.see_ball:
                self.status = self._children[0].tick()
                return self.status
            else:
                self._children[0].failure()
                return self.failure()

    return SeeBall

def farFromObstacle(child):
    class FarFromObstacle(Task):
        def init(self):
            self.addChild(child)

        def tick(self):
            if self.bb.visionInfo.see_obstacle:
                obstacles_field = self.bb.visionInfo.obstacles_field
                see_ball = self.bb.visionInfo.see_ball
                ball_field = self.bb.ball_field
                # for obstacle in obstacles_field:
                #     if get_magnitude(obstacle) < 30 and see_ball and get_magnitude(ball_field) < 15:
                if get_magnitude(obstacles_field[0]) < 30 and see_ball and get_magnitude(ball_field) < 15:
                    self._children[0].failure()
                    return self.failure()
            self.status = self._children[0].tick()
            return self.status

    return FarFromObstacle

def condition(func):
    class cond(Task):
        def init(self):
            pass

        def tick(self):
            if func(self):
                return self.success()
            else:
                return self.failure()

    return cond
def guard(func):
    def mk_guard(child):
        g_ = condition(func)
        return parallel(g_, child)

    return mk_guard
