# @Author: Yusu Pan <yuthon>
# @Date:   2017-07-25T10:24:34+09:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: AvoidObstacle.py
# @Last modified by:   yuthon
# @Last modified time: 2017-07-25T10:24:36+09:00
# @Copyright: ZJUDancer

from DecisionMaking.BehaviorTree.Branch import sequence, selector
from DecisionMaking.BehaviorTree.Task import Action
from utils.mathutil import get_magnitude

class AvoidObstacle(Action):
    """Keep away from obstacle to avoid pushing penalty."""
    def init(self):
        pass

    def tick(self):
        print "------------ AvoidObstacle tick"
        if self.bb.visionInfo.see_obstacle:
            obstacles_field = self.bb.visionInfo.obstacles_field
            see_ball = self.bb.visionInfo.see_ball
            ball_field = self.bb.ball_field
            # for obstacle in obstacles_field:
            if get_magnitude(obstacles_field[0]) < 30 and see_ball and get_magnitude(ball_field) < 15:
                self.crouch()
                return self.running()
        return self.failure()
