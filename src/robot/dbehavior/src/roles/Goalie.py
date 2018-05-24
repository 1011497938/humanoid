from DecisionMaking.BehaviorTree.Task import Action, Task
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from skills.GoToGoalieFPoint import GoToGoalieFPoint
from demo.GoalieDemoF import GoalieDemoF
from skills.Lie import Lie
from skills.GoalieNormal import GoalieNormal
from utils.VecPos import VecPos
from utils.mathutil import get_dis
from math import fabs


KICK_ABAILITY = 200

class _Goalie(Action):
    """

    """
    def init(self):
        if self.bb.parameters.attackRight:
            # print 'fuck right'
            self.bb.reset_particle_point(VecPos(-self.bb.parameters.GoaliePointX, 0))
            # print "fuck init--------------- R"
        else:
            self.bb.reset_particle_point(VecPos(self.bb.parameters.GoaliePointX, 0))
            # print "fuck init--------------- L"
        self.target = VecPos()
        pass

    def tick(self):
        x = self.bb.robot_pos.x
        y = self.bb.robot_pos.y
        if self.bb.parameters.attackRight:
            self.target = VecPos(550, 0)
            if x > -450 + KICK_ABAILITY:
                self.bb.enable_kick = True
            else:
                self.bb.enable_kick = False

        else:
            self.target = VecPos(-550 ,0)
            if x < 450 - KICK_ABAILITY:
                self.bb.enable_kick = True
            else:
                self.bb.enable_kick = False
        self.bb.attack_target = self.target
        return self.running()


Goalie = parallel(_Goalie, GoalieNormal)

# When Penalty
# Goalie = GoalieDemoF
