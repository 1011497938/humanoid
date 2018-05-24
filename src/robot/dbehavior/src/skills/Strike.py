from DecisionMaking.BehaviorTree.Task import Action, Task
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from skills.Attack import Attack
from utils.VecPos import VecPos
from utils.mathutil import get_dis
from math import fabs
from Blackboard import attackRight

if attackRight():
    KICK_ABAILITY = 240
else:
    KICK_ABAILITY = 200

class _Strike(Task):
    """
    Set attack target
    """
    def init(self):
        pass
    def tick(self):
        x = self.bb.robot_pos.x
        y = self.bb.robot_pos.y
        magicy = 30

        if self.bb.parameters.attackRight:
            # set target
            if y > 0:
                target = VecPos(550, magicy)
            else:
                target = VecPos(550, -magicy)

            if fabs(y) > 130:
                target.x -= 100

            # set enable kick
            if  x > 450 - KICK_ABAILITY and x < 450 - 100:
                self.bb.enable_kick = True
            else:
                self.bb.enable_kick = False

        else:
            if y > 0:
                target = VecPos(-550, magicy)
            else:
                target = VecPos(-550, -magicy)

            if fabs(y) > 130:
                target.x += 100

            if x < -(450 - KICK_ABAILITY) and x > -(450 - 100):
                self.bb.enable_kick = True
            else:
                self.bb.enable_kick = False

        self.bb.attack_target = target

        return self.running()

Strike = parallel(_Strike,
                   Attack)
