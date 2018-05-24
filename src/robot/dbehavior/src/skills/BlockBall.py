
from math import degrees, cos, sin
from DecisionMaking.BehaviorTree.Branch import parallel
from DecisionMaking.BehaviorTree.Decorator import seeBall
from DecisionMaking.BehaviorTree.Task import Action
from headskills.TrackBall import TrackBall
from utils.CalcAttackPoint import calc_attack_point
from utils.VecPos import VecPos
from utils.mathutil import angle_between, PI_2, PI, get_dis, abs_angle_diff, get_magnitude, degree_to_radian, degree_between
from roles.Striker import _Striker

# Avoid
AVOID_DIS = 50
AVOID_POINT_DOWN = VecPos(0, -AVOID_DIS * 1.5)
AVOID_POINT_UP = VecPos(0, AVOID_DIS * 1.5)
_striker = _Striker()


@seeBall
class _BlockBall(Action):

    def tick(self):
        destination = self.calc_block_point()

        if self.bb.parameters.attackRight and self.bb.ball_global.x > AVOID_DIS:
            return self.success()
        elif not self.bb.parameters.attackRight and self.bb.ball_global.x < -AVOID_DIS:
            return self.success()

        if not self.got_dest(destination):
            # print 'avoid ball go to dest'
            dis = get_dis(self.bb.ball_global, self.bb.robot_pos)
            if dis < AVOID_DIS:
                self.gotoGlobalOmni(destination)
            else:
                self.gotoGlobal(destination)

            return self.running()
        else:
            # print 'avoid ball get dest'
            self.crouch()
            return self.success()

    def calc_block_point(self):
        ball_pos = self.bb.ball_global
        if self.bb.parameters.attackRight:
            goal = VecPos(-550, 0)
        else:
            goal = VecPos(550, 0)
        # goal.x *= -1
        theta = degree_to_radian(degree_between(goal, ball_pos))


        block_point = VecPos()
        block_point.x = ball_pos.x - AVOID_DIS * 1.5 * cos(theta)
        block_point.y = ball_pos.y - AVOID_DIS * 1.5 * sin(theta)
        block_point.z = degrees(theta)

        return block_point

        # _, _, rub = calc_attack_point()
        # if rub.y > 0:
        #     final_x = ball_pos.x + AVOID_POINT_UP.x * \
        #         cos(theta) - AVOID_POINT_UP.y * sin(theta)
        #     final_y = ball_pos.y + AVOID_POINT_UP.x * \
        #         sin(theta) + AVOID_POINT_UP.y * cos(theta)
        #
        #     return VecPos(final_x, final_y, degrees(theta - PI_2))
        # else:
        #     final_x = ball_pos.x + AVOID_POINT_DOWN.x * \
        #         cos(theta) - AVOID_POINT_DOWN.y * sin(theta)
        #     final_y = ball_pos.y + AVOID_POINT_DOWN.x * \
        #         sin(theta) + AVOID_POINT_DOWN.y * cos(theta)
        #     return VecPos(final_x, final_y, degrees(theta + PI_2))


BlockBall = parallel(_BlockBall, TrackBall)
blockBall = BlockBall()
