from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.TrackBall import TrackBall
from utils.mathutil import *

GoalPoint_Left = VecPos(-420, 0, 0)
GoalPoint_Right = VecPos(420, 0, 180)

class GoToGoaliePoint(Action):
    def init(self):
        pass

    def tick(self):

        if self.bb.attackRight:
            print 'Go to goalie point tick 11'
            if not self.got_dest(GoalPoint_Left):
                self.gotoGlobal(GoalPoint_Left)
                return self.running()
            else:
                return self.success()
        else:
            print 'Go to goalie point tick 22'
            if not self.got_dest(GoalPoint_Right):
                self.gotoGlobal(GoalPoint_Right)
                return self.running()
            else:
                return self.success()
