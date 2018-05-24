from DecisionMaking.BehaviorTree.Task import Action
from utils.mathutil import get_angle, radian_to_degree
from Timer import Timer
from utils.VecPos import VecPos

GazePlats = [
    VecPos(40, 0),
    VecPos(0, 0)
]
SafeDist = 100


class IsBallFarAway(Action):
    def init(self):
        self.cycle = 0
        self.turned = 0
        self.turnCnt = 0
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()
        self.timer = Timer()
        self.fieldAnglePre = self.bb.field_angle
        # print 'init'
        # print self.bb.field_angle

    def reset_(self):
        self.turned = 0
        self.cycle = 0

    def tick(self):
        self.cycle += 1
        # print 'fuck ao', self.cycle, self.turnCnt
        if self.bb.visionInfo.see_ball:
            self.gazeBall()
            if abs(self.bb.ball_global.x) < SafeDist:
                print 'Is ball far away tick SUCCESS'
                return self.success()
            else:
                print 'Is ball far away tick FAILURE'
                return self.failure()
        elif self.turnCnt > 7:
            #print("FindBallAroundOnce: failure")
            # print self.bb.field_angle
            print 'Is ball far away tick FAILURE'
            return self.failure()

        else:
            self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            # print 'self.curPlat: ', self.curPlat
            if self.bb.field_angle - self.fieldAnglePre > 180:
                self.turned += self.bb.field_angle - (self.fieldAnglePre + 360)
            elif self.bb.field_angle - self.fieldAnglePre < -180:
                self.turned += (self.bb.field_angle + 360) - self.fieldAnglePre
            else:
                self.turned += self.bb.field_angle - self.fieldAnglePre
            self.fieldAnglePre = self.bb.field_angle
            # print 'angle {} last_seen_field {}'.format(angle, self.bb.last_seen_ball_field)
            self.turn(10)
            # better use fieldangle
            # if abs(self.turned) > 360:
            #     self.next_plat()
            #     self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            #     return self.success()
            # if abs(self.turned) > 270:
            #     self.next_plat()
            #     self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            #     return self.running()
            # if abs(self.turned) > 180:
            #     self.next_plat()
            #     self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
            #     return self.running()
            if abs(self.turned) > 90:
                self.turned -= 90
                self.next_plat()
                self.turnCnt += 1
                # print "lookAt ({}, {})".format(self.curPlat.x, self.curPlat.y)
                # print self.bb.field_angle
            print 'Is ball far away tick RUNNING'
            return self.running()

    def next_plat(self):
        try:
            self.curPlat = self.iter.next()
            # self.world.scanning = True
            return True
        except StopIteration:
            # GazePlats.reverse()
            # self.world.scanning = False
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()
            return False
