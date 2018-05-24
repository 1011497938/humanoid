from math import degrees, atan2
from DecisionMaking.BehaviorTree.Decorator import seeBall, farFromObstacle
from DecisionMaking.BehaviorTree.Task import Action
from utils.CalcAttackPoint import calc_attack_point
from utils.mathutil import *
from math import fabs
from Timer import Timer

DRIBBLE_THRES = 30
EPSO = 1e-10

DRIBBLE_SAFE = -20
DRIBBLE_X_B = -2
DRIBBLE_X_AD_B = -3  # 2
DRIBBLE_X_AD_F = 3  # 2
DRIBBLE_X_MID = 3
DRIBBLE_X_TOP = 4

DRIBBLE_Y_B = 1
DRIBBLE_Y_AD_B = 2
DRIBBLE_Y_AD_F = 2
DRIBBLE_Y_MID = 1
DRIBBLE_Y_TOP = EPSO

STEP_L = 12
STEP_R = -12
RM_L = 3.5
RM_R = -15.5
LM_L = 15.5
LM_R = -3.5

x_can = [DRIBBLE_X_B, DRIBBLE_X_B, DRIBBLE_X_AD_B, 0, DRIBBLE_X_AD_F, DRIBBLE_X_MID, DRIBBLE_X_TOP, DRIBBLE_X_TOP]
y_can = [EPSO, DRIBBLE_Y_B, DRIBBLE_Y_B, DRIBBLE_Y_AD_B, DRIBBLE_Y_AD_F, DRIBBLE_Y_MID, DRIBBLE_Y_TOP, EPSO]
rec_can = [x_can[i] * 1.0 / y_can[i] for i in range(0, len(x_can))]

DEBUG_DRIBBLE = True

#@farFromObstacle
@seeBall
class Dribble(Action):
    def init(self):
        self.sx, self.sy = 0, 0
        self.exit_cycle = 0
        self.exit_cycle2 = 0
        self.exit_cycle3 = 0
        self.exit_cycle4 = 0
        self.prev_angle = 0
        self.started = False
        self.cycle = 0

    def tick(self):
        # print 'Dribble tick'

        if self.bb.enable_kick:
            self.step()
            # print 'dribble fail'
            return self.failure()

        ball_field = self.bb.ball_field
        if ball_field.x < 15 and abs(ball_field.y) > 20:
            self.exit_cycle4+=1
        if self.exit_cycle4 > 3:
            self.step()
            return self.failure()

        final_dest, dest, rub = calc_attack_point()
        # if abs(self.bb.robot_pos.x) > 100:

        if not self.got_dest_loose(final_dest):
            self.step()
            self.exit_cycle3 += 1

        if self.exit_cycle3 > 5:
            self.step()
            return self.failure()

        if self.bb.ball_field.x > 40:
            self.step()
            return self.failure()

        self.lookAt(45, 0)
        vy = self.bb.vy

        if vy <= 0:
            current_l = STEP_L - (STEP_L - RM_L) / 1.8 * abs(vy)
            current_r = STEP_R - (STEP_R - RM_R) / 1.8 * abs(vy)
        else:
            current_l = STEP_L - (STEP_L - LM_L) / 1.8 * abs(vy)
            current_r = STEP_R - (STEP_R - LM_R) / 1.8 * abs(vy)

        eye_y = (current_l + current_r) / 2.0

        ball_y = self.bb.ball_field.y
        ball_x = self.bb.ball_field.x
        diff_y = ball_y - eye_y

        # print diff_y

        if STEP_L * 1.5 > diff_y > STEP_R * 1.5:
            pass
        else:
            self.exit_cycle2 += 1

        if self.exit_cycle2 > 0:
            self.step()
            return self.failure()
        else:
            diff_x = ball_x + 10
            theta = diff_x / abs(diff_y + 0.00001)

            for i in range(0, 7):
                if rec_can[i] <= theta <= rec_can[i + 1]:
                    a_s = (theta * y_can[i] - x_can[i]) / (
                        x_can[i + 1] - x_can[i] - theta * (y_can[i + 1] - y_can[i]))
                    self.sx = x_can[i] + a_s * (x_can[i + 1] - x_can[i])
                    self.sy = y_can[i] + a_s * (y_can[i + 1] - y_can[i])
                if diff_x >= 0:
                    self.sx = min(self.sx, diff_x)
                else:
                    self.sx = max(self.sx, diff_x)
                self.sy = min(abs(diff_y), self.sy)
            if diff_y < 0:
                self.sy = -self.sy

            t = 0
            self.walk(self.sx, self.sy, t)
            return self.running()
