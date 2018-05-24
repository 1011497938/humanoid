from utils.VecPos import VecPos, calc_field_position, calc_global_position
from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel, selector
from DecisionMaking.BehaviorTree.Decorator import seeBall, farFromObstacle
from headskills.TrackBall import TrackBall
from skills.WalkBehindBall import WalkBehindBall
from utils.CalcAttackPoint import calc_attack_point
from utils.mathutil import *

@seeBall
# @farFromObstacle
class _Approach(Action):
    def init(self):
        self.get_dest_cycle = 0

    def tick(self):
        final_dest, dest, rub = calc_attack_point()

        if self.got_dest(final_dest):
            # fixme !?
            self.step()
            # print 'approachball got dest'
            return self.success()

        elif -30 < rub.x < 0 and abs(rub.y) < 30:
            return self.failure()

        else:
            if dest is not final_dest:
                dest_field = calc_field_position(final_dest, self.bb.robot_pos)
                if dest_field.x < -20:
                    dest = final_dest
                elif self.got_dest(dest):
                    dest = final_dest

            self.bb.behaviorInfo.dest = dest.toVector3()
            self.bb.behaviorInfo.final_dest = final_dest.toVector3()

            x, y, t = self.walksolver.solveGlobal(dest)

            angle = degree_between(self.bb.ball_global, self.bb.robot_pos)
            diff = angle_normalization(self.bb.field_angle - angle + 180)
            dis = self.bb.ball_field.length()

            if 80 < dis:
                if rub.x < 10:   #  and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 3
                    else:
                        t += 3

                elif rub.x > 10:  # and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 3
                    else:
                        t += 3

            self.walk(x, y, t)
            # self.crouch()
            return self.running()


    # def got_dest(self, dest):
    #     robot_pos = self.bb.robot_pos
    #     angle = self.bb.field_angle
    #     dangle = abs_angle_diff(dest.z - angle)
    #
    #     diff_x = dest.x - robot_pos.x
    #     diff_y = dest.y - robot_pos.y
    #
    #     if not self.bb.enable_kick:
    #         if self.bb.ball_field.y > 0:
    #             if abs(diff_x) < DEST_REGION and -DEST_REGION < diff_y < DEST_REGION / 3 and abs(dangle) < DEST_RE_ANGLE:
    #                 return True
    #             else:
    #                 return False
    #         else:
    #             if abs(diff_x) < DEST_REGION and -DEST_REGION / 3 < diff_y < DEST_REGION and abs(dangle) < DEST_RE_ANGLE:
    #                 return True
    #             else:
    #                 return False
    #     elif self.bb.enable_kick:
    #         if abs(diff_x) < 5 and abs(diff_y) < 5 and abs(dangle) < 5:
    #             return True
    #         else:
    #             return False
    #     else:
    #         return False

ApproachBall = parallel(selector(_Approach, WalkBehindBall), TrackBall)
