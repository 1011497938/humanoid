from DecisionMaking.BehaviorTree.Task import Action
from Timer import Timer
from utils.VecPos import VecPos

GazePlats = [
    VecPos(0, 0),
    VecPos(0, 40),
    VecPos(0, 0),
    VecPos(0, -40)
]

ATTACK_DIATANCE_X = 200
ATTACK_DIATANCE_Y = 165

class GoalieScanField(Action):
    """
    Head skill: Find ball, when robot is not turning, scan field, else look down
    """

    def init(self):
        self.timer = Timer()
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()

    def tick(self):
        if self.bb.parameters.attackRight:
            print 'fuck right'
            self.bb.reset_particle_point(VecPos(-self.bb.parameters.GoaliePointX, 0))
        else:
            print 'fuck left'
            self.bb.reset_particle_point(VecPos(self.bb.parameters.GoaliePointX, 0))
        self.crouch()
        if self.bb.see_ball:
            self.gazeBall()
            if self.bb.attackRight:
                if self.bb.ball_global.x < -450 + ATTACK_DIATANCE_X and abs(self.bb.ball_global.y) < 200:
                    print 'GoalieScanField Succcess'
                    return self.success()
                else:
                    return self.running()
            else:
                if self.bb.ball_global.x > 450 - ATTACK_DIATANCE_X and abs(self.bb.ball_global.y) < 200:
                    print 'GoalieScanField Succcess'
                    return self.success()
                else:
                    return self.running()
        else:
            if self.timer.elapsed() > 1:
                self.timer.restart()
                self.next_plat()
            self.lookAt(self.curPlat.x, self.curPlat.y, 10, 10)
        # print('LookAround running')
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
