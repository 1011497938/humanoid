# from DecisionMaking.BehaviorTree.Task import Action
# from dmsgs.msg import BehaviorInfo
# from utils.VecPos import VecPos
# from headskills.ScanField_fast import scanField_fast
# from headskills.TrackBall import trackBall
# from skills.Defend import Defend
# from skills.Strike import Strike
#
# class Defender(Action):
#     def init(self):
#         self.defend = Defend()
#         self.strike = Strike()
#
#     def onEnter(self):
#         self.init_ = True
#
#     def tick(self):
#         """
#         Defender go to defend position, to be changed to striker
#         """
#         if self.bb.closest_to_ball():
#             self.strike.tick()
#         else:
#             self.defend.tick()
#
#         return self.running()
#

from DecisionMaking.BehaviorTree.Task import Action, Task
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from skills.FindBallAroundOnce import FindBallAroundOnce
from skills.Attack import Attack
from skills.Strike import Strike
from utils.VecPos import VecPos
from utils.mathutil import get_dis, degree_between, get_magnitude
from Blackboard import attackRight
from headskills.TrackBall import trackBall
from math import fabs, sqrt
from Timer import Timer
from skills.SeekBall_Fuck import SeekBall_Fuck


KICK_ABAILITY = 200
class Defender(Action):
    def init(self):
        self.strike = Strike()
        self.seekball = FindBallAroundOnce()
        self.ready = False
        self.lostBallTimer = Timer()
        self.seekballFuck = SeekBall_Fuck()

    def tick(self):
        if self.bb.see_ball:
            self.lostBallTimer.restart()

        if self.lostBallTimer.elapsed() > 15:
            self.gotoGlobal(self.bb.get_default_dest())

            if self.got_dest_loose(self.bb.get_default_dest()):
                self.lostBallTimer.restart()

        elif self.bb.closest_to_ball() or self.bb.team_lost_ball():
            self.ready= False
            self.strike.tick()

        else:
            """
            Face ball if see ball, else seek ball
            Approach close to ball
            
            Keep 1m away, don't block attack path
            """

            if self.bb.see_ball:
                bally = self.bb.ball_global.y
                ballx = self.bb.ball_global.x

                rx = self.bb.robot_pos.x
                ry = self.bb.robot_pos.y

                dx = rx - ballx
                dy = ry - bally

                dist = sqrt(dx * dx + dy * dy)

                # for attack right case
                change = False
                if attackRight():
                    if ballx > rx or abs(bally - ry) > 150 or dist > 200 or dist < 50:
                        if ry > bally:
                            y = bally + 50
                        else:
                            y = bally - 50

                        angle = degree_between(self.bb.robot_pos, self.bb.ball_global)
                        if not self.gotoGlobal(VecPos(ballx - 50, y, angle)):
                            change = True
                else:
                    if ballx < rx or abs(bally - ry) > 150 or dict > 200 or dist < 50:
                        if ry > bally:
                            y = bally + 50
                        else:
                            y = bally - 50

                        angle = degree_between(self.bb.robot_pos, self.bb.ball_global)
                        if not self.gotoGlobal(VecPos(ballx - 50, y, angle)):
                            change = True

                trackBall.tick()
                if not change:
                    self.faceBall()
            else:
                # to do, max turn cnt, else go back
                self.seekball.tick()

        return self.running()


    def wait(self):
        """
        Crouch and face ball and scan field
        """
        self.faceBall()

    def push(self):
        """
        Push forward waiting for a pass
        :return:
        """
        y = self.bb.robot_pos.y
        if self.bb.parameters.attackRight:
            dest = VecPos(200, y, 0)
        else:
            dest = VecPos(-200, y, 180)

        if not self.got_dest_loose(dest):
            self.gotoGlobalOmni(dest)
            self.ready = False
        else:
            self.ready = True

