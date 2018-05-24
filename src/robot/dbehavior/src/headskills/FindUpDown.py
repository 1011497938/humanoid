from DecisionMaking.BehaviorTree.Task import Action
from Timer import Timer
from utils.VecPos import VecPos

GazePlats = [
    VecPos(40, 0),
    VecPos(0, 0)
]

class FindUpDown(Action):
    def init(self):
        self.timer = Timer()
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()

    def tick(self):
        if self.bb.visionInfo.see_ball:
            self.gazeBall()
            return self.success()
        else:
            if self.timer.elapsed() > 1:
                self.timer.restart()
                if not self.next_plat():
                    print "failure"
                    return self.failure()

            self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            print "lookAt ({}, {})".format(self.curPlat.x, self.curPlat.y)
            print "running"
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
