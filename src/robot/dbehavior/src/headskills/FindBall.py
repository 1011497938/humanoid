from DecisionMaking.BehaviorTree.Task import Action
from Timer import Timer
from utils.VecPos import VecPos
from utils.mathutil import get_magnitude

GazePlats = [
    VecPos(15, -100),
    VecPos(15, 0),
    VecPos(15, 100)
]


class FindBall(Action):
    """
    Head skill: Find ball, when robot is not turning, scan field, else look down
    """

    def init(self):
        self.timer = Timer(1)
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()

    def tick(self):
        if not self.bb.visionInfo.see_ball and self.bb.ball_lost.elapsed() < 10 and get_magnitude(self.bb.last_seen_ball_field) < 50:
            return self.failure()

        if self.gazeBall():
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()
            return self.success()
        else:
            if self.timer.finished():
                self.timer.restart()
                if not self.next_plat():
                    return self.failure()

            self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            return self.running()

    def next_plat(self):
        try:
            self.curPlat = self.iter.next()
            # self.world.scanning = True
            return True
        except StopIteration:
            # GazePlats.reverse()
            # self.world.scanning = False
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()
            return False
