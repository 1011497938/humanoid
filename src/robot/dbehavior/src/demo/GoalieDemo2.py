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
#                                     |            |  -----  O(ball)
#                                     |  O(robot)  |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
#                                     |            |
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

scramblePointY = 5
ballMinMoveSpeed = 5
adjustDist = 300           # the ball distance to adjust self position
adjustBallY = 20           # when ball's y is larger than adjustBallY, robot will adjust its position
dangerDist = 100            # the ball distance to take action by ball velocity
scrambleDist = 30          # the ball distance to decide which side to scramble
maxMoveUp = 80
maxMoveDown = -80
dangerVelocity = 30


class GoalieDemo2(Action):
    def init(self):
        self.attackRight = True
        self.moveY = 0
        # self.maxVelocity = 0
        self.attackPointY = 0
        self.robotYPre = 0
        self.robotYCur = 0
        self.inited = False
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
                if self.bb.ball_field.y > scramblePointY:
                    # print 'scramble left --- scrambleDist'
                    self.goalieLeft()
                    return self.success()  # failure
                elif self.bb.ball_field.y < -scramblePointY:
                    # print 'scramble right --- scrambleDist'
                    self.goalieRight()
                    return self.success()  # failure
                else:
                    # print 'keep mid --- scrambleDist'
                    self.crouch
                    return self.running()
            elif abs(self.bb.ball_field.x) < dangerDist:
                # print 'ball velocity    : ', self.bb.ball_velocity
                if self.bb.ball_velocity.length() < ballMinMoveSpeed:
                    print 'lineUp --- dangerDist'
                    self.lineUp()
                    return self.running()
                if self.calcAttackPointY():
                    if self.bb.ball_velocity.length() > dangerVelocity and self.attackPointY > scramblePointY and self.attackPointY < 200:
                        print 'scramble left --- dangerDist'
                        self.goalieLeft()
                        return self.success()
                    elif self.bb.ball_velocity.length() > dangerVelocity and self.attackPointY < -scramblePointY and self.attackPointY > -200:
                        print 'scramble right --- dangerDist'
                        self.goalieRight()
                        return self.success()
                    # else:
                    #     print 'lineUp --- dangerDist'
                    #     self.lineUp()
                    #     return self.running()c
                else:
                    self.lineUp()
                    return self.running()
            elif abs(self.bb.ball_field.x) < adjustDist:
                self.lineUp()
                return self.running()
            else:
                self.crouch()
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

    def lineUp(self):
        if not self.inited:
            self.robotYPre = self.bb.robot_pos.y
            self.robotYCur = self.robotYPre
            self.inited = True
        else:
            self.robotYCur = self.bb.robot_pos.y
            self.moveY += self.robotYCur - self.robotYPre
            self.robotYPre = self.robotYCur



        if self.moveY < maxMoveUp and self.bb.ball_field.y > adjustBallY:
            self.walk(0, 3, 0)
        elif self.moveY > maxMoveDown and self.bb.ball_field.y < -adjustBallY:
            self.walk(0, -3, 0)
        else:
            self.crouch()

    def calcAttackPointY(self):
        if abs(self.bb.visionInfo.ball_velocity.x) < 0.01:
            return False
        else:
            self.attackPointY = self.bb.visionInfo.ball_velocity.y / self.bb.visionInfo.ball_velocity.x * self.bb.ball_field.x + self.bb.ball_field.y
            return True
