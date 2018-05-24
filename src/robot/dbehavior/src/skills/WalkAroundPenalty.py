from utils.VecPos import VecPos
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.TrackBall import TrackBall
from utils.mathutil import *

GazePlats = [VecPos(0, 0),
             VecPos(0, 60),
             VecPos(0, 0),
             VecPos(0, -60)]

Penalty_Back = VecPos(-240, 0, 0)
Penalty_Front = VecPos(240, 0, -180)

class WalkAroundPenalty(Action):
    def init(self):
        self.currentTarget = 0
        self.penalty_1_arrived = False
        self.penalty_2_arrived = False
        self.iter = iter(GazePlats)



    def tick(self):
        # self.currentTarget += 1
        # if self.currentTarget == 4:
        #     self.currentTarget = 0
        if self.bb.visionInfo.see_ball:
            # self.gazeBall()
            return self.success()
        else:
            if not self.penalty_1_arrived:
                # cur_x = Penalty_Back.x
                # cur_y = Penalty_Back.y
                # cur_t = 0
                if self.bb.robot_pos.distance(Penalty_Back) < 20:
                    self.penalty_1_arrived = True
                    self.penalty_2_arrived = False
                self.gotoGlobal(Penalty_Back)
            else:
                # cur_x = Penalty_Front.x
                # cur_y = Penalty_Front.y
                # cur_t = 0
                if self.bb.robot_pos.distance(Penalty_Front) < 20:
                    self.penalty_1_arrived = False
                    self.penalty_2_arrived = True
                self.gotoGlobal(Penalty_Front)
        return self.running()
