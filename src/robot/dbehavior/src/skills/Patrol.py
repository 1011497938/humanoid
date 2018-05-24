from math import degrees, atan2
from DecisionMaking.BehaviorTree.Task import Action
from utils.mathutil import *
from Timer import Timer


class Patrol(Action):
    def init(self):
        self.timer = Timer()
        self.timer2 = Timer(2)
        self.straight = True

    def tick(self):
        # if not ballValid():
        #     return self.failure()
        print(self.timer2.elapsed())
        self.lookAt(0, 0)
        self.walk(3, 0, 0)

        if self.straight:
            self.walk(3, 0, 0)
        else:
            self.walk(3, 0, 3)

        if self.timer2.finished():
            if self.straight:
                self.straight = False
                self.timer2.restart()
            else:
                self.straight = True
                self.timer2.restart()



        return self.running()
