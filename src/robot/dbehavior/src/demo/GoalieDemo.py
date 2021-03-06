from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import sequence, parallel
from DecisionMaking.BehaviorTree.Decorator import seeBall
from utils.VecPos import VecPos
from Timer import Timer

#                                 scrambleDist    dangerDist
#
#                                           L     L
#                                           |     |
#                                           |     |
#                                           V     V
#
#                                     |------------------------------------------------
#                                     |     |     |
#                                     |
#                                     | ------------
#                                     |            |
#                                     |            |
#    maxKeepUp L----->                |            |
#                                     |            |
#                                     |            |           O(ball)
#                                     |            |     ------
#                                  -  |         ---|-----
#                                     | --------   |
#   self.keepPoint P-->         ------O-           |
#                       --------      |            |
#   keepLineLeft P--> O               |            |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
#   maxKeepDown L----->               |            |
#                                     |            |
#                                     |            |
#                                     |-------------
#                                     |
#                                     |
#                                     |---------------------------------------------------

keepPointLeft = VecPos(-600, 0, 0)
keepPointRight = VecPos(600, 0, 180)
keepLineLeft = VecPos(-440, 0, 0)
keepLineRight = VecPos(440, 0, 180)
scrambleDist = 30          # the ball distance to decide which side to scramble
dangerDist = 100            # the ball distance to take action by ball velocity
maxKeepUp = 80
minKeepDown = -80
dangerVelocity = 70


class GoalieDemo(Action):
    def init(self):
        self.attackRight = True
        self.keepPoint = VecPos(0, 0, 0)
        self.angleDiff = 0
        # self.maxVelocity = 0
        self.attackPointY = 0
        pass

    def tick(self):
        # if self.bb.field_angle - self.keepPoint.z > 180:
        #     self.angleDiff = self.bb.field_angle - (self.keepPoint.z + 360)
        # elif self.bb.field_angle - self.keepPoint.z < -180:
        #     self.angleDiff = (self.bb.field_angle + 360) - self.keepPoint.z
        # else:
        #     self.angleDiff = self.bb.field_angle - self.keepPoint.z

        if self.bb.visionInfo.see_ball:
            self.gazeBall()
            print self.bb.ball_field
            # print 'ballVelocity :',  self.bb.ball_velocity.length()
            # if self.bb.ball_velocity.length() > self.maxVelocity:
            #     self.maxVelocity = self.bb.ball_velocity.length()

            if abs(self.bb.ball_field.x) < scrambleDist:
                if self.bb.ball_field.y > 5:
                    # print 'scramble left --- scrambleDist'
                    self.goalieLeft()
                    return self.running()  # failure
                elif self.bb.ball_field.y < -5:
                    # print 'scramble right --- scrambleDist'
                    self.goalieRight()
                    return self.running()  # failure
                else:
                    # print 'keep mid --- scrambleDist'
                    self.crouch
                    return self.running()
            elif abs(self.bb.ball_field.x) < dangerDist:
                # print 'ball velocity    : ', self.bb.ball_velocity
                if self.calcAttackPointY():
                    if self.bb.ball_velocity.length() > dangerVelocity and self.attackPointY > 5 and self.attackPointY < 200:
                        # print 'scramble left --- dangerDist'
                        self.goalieLeft()
                        return self.running()
                    elif self.bb.ball_velocity.length() > dangerVelocity and self.attackPointY < -5 and self.attackPointY > -200:
                        # print 'scramble right --- dangerDist'
                        self.goalieRight()
                        return self.running()
                    else:
                        # print 'keep mid --- dangerDist'
                        self.crouch
                        return self.running()
                else:
                    # print 'keep mid --- dangerDist'
                    self.crouch
                    return self.running()
            else:
                # print 'ball is too far'
                return self.running()
                # if self.calPos():
                #     print(self.keepPoint)
                #     if self.bb.robot_pos.distance(self.keepPoint) > 10:
                #         self.gotoGlobalOmni(self.keepPoint)
                #     else:
                #         self.crouch
                #     return self.running()
                # else:
                #     return self.running()


            # if self.bb.robot_pos.y - self.keepPoint.y > 20:
            #     self.walk(0, -3, 0)
            # elif self.bb.robot_pos.y - self.keepPoint.y < -20:
            #     self.walk(0, 3, 0)
            # elif self.angleDiff > 5:
            #     self.turn(-5)
            # elif self.angleDiff < -5:
            #     self.turn(5)

        else:
            self.lookAt(0, 0)
            self.crouch
        return self.running()

    def calcAttackPointY(self):
        if abs(self.bb.visionInfo.ball_velocity.x) < 0.01:
            return False
        else:
            self.attackPointY = self.bb.visionInfo.ball_velocity.y / self.bb.visionInfo.ball_velocity.x * self.bb.ball_field.x + self.bb.ball_field.y
            return True

    def calPos(self):
        if abs(self.bb.ball_global.x - keepPointLeft.x) < 10:
            return False
        if self.attackRight:
            self.keepPoint.y = (self.bb.ball_global.y - keepLineLeft.y) * (keepLineLeft.x - keepPointLeft.x) / (self.bb.ball_global.x - keepPointLeft.x) + keepLineLeft.y
            self.keepPoint.x = keepLineLeft.x
            self.keepPoint.z = 0
        else:
            self.keepPoint.y = (self.bb.ball_global.y - keepLineRight.y) * (keepLineRight.x - keepPointRight.x) / (self.bb.ball_global.x - keepPointRight.x) + keepLineRight.y
            self.keepPoint.x = keepLineRight.x
            self.keepPoint.z = -180
        if self.keepPoint.y > maxKeepUp:
            self.keepPoint.y = maxKeepUp
        elif self.keepPoint.y < minKeepDown:
            self.keepPoint.y = minKeepDown
        return True
