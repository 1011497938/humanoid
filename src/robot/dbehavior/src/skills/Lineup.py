from __future__ import division
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from DecisionMaking.BehaviorTree.Decorator import seeBall
from utils.CalcAttackPoint import calc_attack_point
from Timer import Timer

@seeBall
class Lineup(Action):
    def init(self):
        self.diff_x = 0
        self.diff_y = 0

        self.sum_x = 0
        self.sum_y = 0

        self.avg_x = 0
        self.avg_y = 0

        self.line_up_cycle = 0
        self.stop_timer = Timer()
        # FIXME(MWX): on entry
        self.entry = True
        self.timer = Timer(.5)

        self.left_kick = False

    def tick(self):
        if self.entry:
            self.entry = False
            self.timer.restart()

        self.lookAt(43, 0)
        ballvision = self.bb.ball_field
        if abs(ballvision.y) > 25 or abs(ballvision.x) > 25:
            return self.failure()

        # final_dest, dest, rub = calc_attack_point()
        # if not self.got_dest_tight(final_dest):
        #     return self.failure()

        # TODO(MWX): step may be better
        if not self.timer.finished():
            self.crouch()
            # if ballvision.y > 0:
            #     self.bb.left_kick = True
            # else:
            #     self.bb.left_kick = False
            return self.running()

        # if self.bb.left_kick:
        #     dx = ballvision.x - self.bb.parameters.left_kick_point.x
        #     dy = ballvision.y - self.bb.parameters.left_kick_point.y
        # else:

        dx = ballvision.x - self.bb.parameters.left_kick_point.x
        dy = ballvision.y - self.bb.parameters.left_kick_point.y

        if not abs(dx) < 3 or not abs(dy) < 3:
            x, y, t = [0, 0, 0]
            if dx > 3:
                x = 1
            elif dx < -3:
                x = -1

            if dy > 3:
                y = 1
            elif dy < -3:
                y = -1

            self.walk(x, y, t)
            return self.running()
        else:
            self.crouch()
            return self.success()

