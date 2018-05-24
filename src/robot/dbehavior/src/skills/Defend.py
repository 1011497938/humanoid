from DecisionMaking.BehaviorTree.Task import Action
from Blackboard import attackRight
from utils.VecPos import VecPos
from utils.CalcAttackPoint import calc_attack_point
from headskills.ScanField import ScanField
from headskills.TrackBall import trackBall

class Defend(Action):
    """
    If see ball go to defend position, and face ball,
    when team lost ball, find ball
    else go back,
    when in danger call support
    """
    def init(self):
        self.sc = ScanField()

    def tick(self):
        if self.got_dest_loose(self.calc_defend_pos()):
            self.crouch()
            trackBall.tick()
        else:
            if self.gotoGlobal(self.calc_defend_pos()):
                self.faceBall()
                self.crouch()
            else:
                trackBall.tick()


        return self.running()

    def calc_defend_pos(self):
        ball = self.bb.ball_global

        if attackRight():
            dest = VecPos(-330, 0, 0)
        else:
            dest = VecPos(330, 0, 180)

        return dest


