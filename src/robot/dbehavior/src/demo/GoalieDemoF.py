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

scramblePointX = 5
ballMinMoveSpeed = 5
adjustDist = 300           # the ball distance to adjust self position
adjustBallX = 20           # when ball's y is larger than adjustBallY, robot will adjust its position
dangerDist = 60            # the ball distance to take action by ball velocity
scrambleDist = 60          # the ball distance to decide which side to scramble
maxMoveUp = 80
maxMoveDown = -80
dangerVelocity = 30


class GoalieDemoF(Action):
    def init(self):
        self.attackRight = True
        self.moveX = 0
        # self.maxVelocity = 0
        self.attackPointX = 0
        self.robotXPre = 0
        self.robotXCur = 0
        self.inited = False
        self.lookAt(0, -80)
        self.leftScramble = False
        self.rightScramble = False
        self.goalie_timer = Timer(3)
        pass

    def tick(self):
        # if self.bb.field_angle - self.keepPoint.z > 180:
        #     self.angleDiff = self.bb.field_angle - (self.keepPoint.z + 360)
        # elif self.bb.field_angle - self.keepPoint.z < -180:
        #     self.angleDiff = (self.bb.field_angle + 360) - self.keepPoint.z
        # else:
        #     self.angleDiff = self.bb.field_angle - self.keepPoint.z
        if self.leftScramble and not self.goalie_timer.finished():
            self.goalieLeft()
            print 'scramble left'
            return self.running()
        elif self.leftScramble and self.goalie_timer.finished():
            return self.success()
        elif self.rightScramble and not self.goalie_timer.finished():
            self.goalieRight()
            print 'scramble right'
            return self.running()
        elif self.rightScramble and self.goalie_timer.finished():
            return self.success()




        if self.bb.visionInfo.see_ball:
            self.gazeBall()
            print self.bb.ball_field
            print 'ballVelocity :',  self.bb.ball_velocity.y
            # if self.bb.ball_velocity.length() > self.maxVelocity:
            #     self.maxVelocity = self.bb.ball_velocity.length()

            if abs(self.bb.ball_field.y) < scrambleDist:
                if self.bb.ball_field.x > scramblePointX:
                    print 'scramble left --- scrambleDist'
                    self.leftScramble = True
                    self.goalie_timer.restart()
                    # time.sleep(3)
                    return self.running()  # failure
                elif self.bb.ball_field.x < -scramblePointX:
                    print 'scramble right --- scrambleDist'
                    self.rightScramble = True
                    self.goalie_timer.restart()
                    # time.sleep(3)
                    return self.running()  # failure
                else:
                    print 'keep mid --- scrambleDist'
                    self.crouch()
                    return self.running()
            elif abs(self.bb.ball_field.y) < dangerDist:
                # print 'ball velocity    : ', self.bb.ball_velocity
                # if abs(self.bb.ball_velocity.y) < ballMinMoveSpeed:
                    # print 'lineUp --- dangerDist'
                    # self.lineUp()
                    # return self.running()
                if self.calcAttackPointX():
                    if abs(self.bb.ball_velocity.y) > dangerVelocity and self.attackPointX > scramblePointX and self.attackPointX < 200:
                        print 'scramble left --- dangerDist'
                        self.leftScramble = True
                        self.goalie_timer.restart()
                        # time.sleep(3)
                        return self.running()
                    elif abs(self.bb.ball_velocity.y) > dangerVelocity and self.attackPointX < -scramblePointX and self.attackPointX > -200:
                        print 'scramble right --- dangerDist'
                        self.rightScramble = True
                        self.goalie_timer.restart()
                        # time.sleep(3)
                        return self.running()
                    # else:
                    #     print 'lineUp --- dangerDist'
                    #     return self.running()c
                    #     self.lineUp()
                else:
                    # self.lineUp()
                    self.crouch()
                    return self.running()
            elif abs(self.bb.ball_field.y) < adjustDist:
                # print 'adjustDist'
                # self.lineUp()
                self.crouch()
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
            self.lookAt(0, -80)
            self.crouch
        return self.running()

    def lineUp(self):
        print 'moveX: ', self.moveX
        if not self.inited:
            self.robotXPre = self.bb.robot_pos.x
            self.robotXCur = self.robotXPre
            self.inited = True
        else:
            self.robotXCur = self.bb.robot_pos.x
            self.moveX += self.robotXCur - self.robotXPre
            self.robotXPre = self.robotXCur

        if self.moveX < maxMoveUp and self.bb.ball_field.x > adjustBallX:
            self.walk(3, 0, 0)
        elif self.moveX > maxMoveDown and self.bb.ball_field.x < -adjustBallX:
            self.walk(-1, 0, 0)
        else:
            self.crouch()

    def calcAttackPointX(self):
        if abs(self.bb.visionInfo.ball_velocity.y) < 0.01:
            return False
        else:
            self.attackPointX = -self.bb.visionInfo.ball_velocity.x / self.bb.visionInfo.ball_velocity.y * self.bb.ball_field.y + self.bb.ball_field.x
            return True
