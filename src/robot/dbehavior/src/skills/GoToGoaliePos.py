from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.TrackBall import TrackBall
from utils.mathutil import *


class GoToGoaliePos(Action):
    def init(self):
        self.attackRight = True
        self.angleDiff = 0
        if self.attackRight:
            self.goaliePos = VecPos(-440, 0, 0)
        else:
            self.goaliePos = VecPos(440, 0, 180)
        pass

    def tick(self):
        print 'go to goal pos'
        self.gotoGlobal(self.goaliePos)
        if self.bb.field_angle - self.goaliePos.z > 180:
            self.angleDiff = self.bb.field_angle - (self.goaliePos.z + 360)
        elif self.bb.field_angle - self.goaliePos.z < -180:
            self.angleDiff = (self.bb.field_angle + 360) - self.goaliePos.z
        else:
            self.angleDiff = self.bb.field_angle - self.goaliePos.z

        if self.bb.robot_pos.distance(self.goaliePos) < 20:
            if abs(self.angleDiff) < 5:
                print 'success'
                return self.success()
            elif self.angleDiff > 0:
                self.turn(-5)
            else:
                self.turn(5)
        return self.running()
