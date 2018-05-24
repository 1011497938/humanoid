from math import degrees, atan2, radians, cos, sin
from utils.VecPos import VecPos, calc_field_position, calc_global_position
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel, selector
from DecisionMaking.BehaviorTree.Decorator import seeBall
from headskills.TrackBall import TrackBall
from utils.CalcAttackPoint import calc_attack_point
from utils.mathutil import *

SAFE_DIST = 30
ANGLE_PER_TICK = 20
CLOCK_WISE = -1
ANTI_CLOCK_WISE = 1


@seeBall
class WalkBehindBall(Action):
    def tick(self):
        self.lookAt(45, 0)

        ball_field = self.bb.ball_field
        final_dest, _, rub = calc_attack_point()

        if self.got_dest(final_dest):
            self.step()
            return self.success()
        else:
            if rub.y > 0:
                direction = CLOCK_WISE
            else:
                direction = ANTI_CLOCK_WISE

            r = SAFE_DIST
            theta = ANGLE_PER_TICK * direction

            beta = angle_normalization(180.0 - degrees(atan2(ball_field.y, ball_field.x)))
            alpha = radians(theta - beta)
            r_vec = VecPos(r * cos(alpha), r * sin(alpha))

            des = ball_field + r_vec
            des.z = angle_normalization(alpha + 180)

            x, y, t = self.walksolver.solveField(des)

            t *= 0.5
            self.walk(x, y, t)
            return self.running()

    def got_dest(self, pos):
        dis = get_dis(pos, self.bb.robot_pos)
        diff_angle = abs_angle_diff(pos.z - self.bb.robot_pos.z)

        if dis < 30 and diff_angle < 10:
            return True
        else:
            return False


