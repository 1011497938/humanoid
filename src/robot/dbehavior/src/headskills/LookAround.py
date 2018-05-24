from DecisionMaking.BehaviorTree.Task import Action
from Timer import Timer
from utils.VecPos import VecPos

GazePlats = [
    VecPos(0, 0),
    VecPos(0, 60),
    VecPos(0, 0),
    VecPos(0, -60)
]


class LookAround(Action):
    """
    Head skill: Find ball, when robot is not turning, scan field, else look down
    """

    def init(self):
        self.timer = Timer()
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()

    def tick(self):
        # print("*********LookAround tick")
        if self.gazeBall():
            # if ball is seen, then reinit FindBall
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()
            # print('LookAround success')
            return self.success()

        else:
            if self.timer.elapsed() > 1:
                self.timer.restart()
                self.next_plat()
            self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
        # print('LookAround running')
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
