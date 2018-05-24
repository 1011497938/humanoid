from DecisionMaking.BehaviorTree.Task import Action
from utils.mathutil import get_angle, radian_to_degree, angle_normalization


class TurnAround(Action):
    def init(self):
        self.turned = 0
        self.target_angle = angle_normalization(self.bb.robot_pos.z + 180.0)

    def reset_(self):
        self.turned = 0

    def tick(self):
        if self.bb.visionInfo.see_ball:
            self.step()
            return self.success()
        else:
            # print self.bb.robot_pos.z - self.target_angle
            if abs(self.bb.robot_pos.z - self.target_angle) < 20:
                return self.failure()
            else:
                angle = get_angle(self.bb.last_seen_ball_field)

                if angle > 0:
                    self.turn(20)
                else:
                    self.turn(-20)

                self.lookAt(43, 0)
                return self.running()
