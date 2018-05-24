from DecisionMaking.BehaviorTree.Task import Action
from utils.mathutil import get_angle, radian_to_degree
from Timer import Timer

class TurnAroundOnce(Action):
    def init(self):
        self.turned = 0
        self.timer = Timer()

    def reset_(self):
        self.turned = 0

    def tick(self):
        print '----------TurnAroundOnce tick'
        print self.turned
        if self.bb.visionInfo.see_ball:
            self.gazeBall()
            return self.success()

        else:
            # better use fieldangle
            if abs(self.turned) > 90:
                print 'success'
                return self.success()
            else:
                # print self.turned
                self.turned += self.bb.robot_deltaZ
                # print 'angle {} last_seen_field {}'.format(angle, self.bb.last_seen_ball_field)
                self.turn(5)
                return self.running()
