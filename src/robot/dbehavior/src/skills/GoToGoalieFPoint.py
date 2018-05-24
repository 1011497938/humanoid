from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.TrackBall import TrackBall
from utils.mathutil import *

attackRight = True
GoalPoint_Back = VecPos(-440, 0, 90)
GoalPoint_Front = VecPos(440, 0, -90)

class GoToGoalieFPoint(Action):
    def init(self):
        pass

    def tick(self):
        if attackRight:
            if not self.got_dest_tight(GoalPoint_Back):
                self.gotoGlobal(GoalPoint_Back)
                return self.running()
            else:
                return self.success()
        else:
            if not got_dest_tight(GoalPoint_Front):
                self.gotoGlobal(GoalPoint_Front)
                return self.running()
            else:
                return self.success()
