from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from states.GCStates import Initial
from utils.VecPos import VecPos
from headskills.ScanField_fast import scanField_fast
from headskills.TrackBall import trackBall
from dmsgs.msg import GCInfo
from Timer import Timer

# Actually shuold be Init
class Reentry(Action):
    def init(self):
        self.timer = Timer(self.bb.parameters.motionInitTime)
        self.timer2 = Timer(self.bb.parameters.motionInitTime)
        self.initial = Initial()
        self.cur_state = None

    def onEnter(self):
        self.timer.restart()

    def tick(self):
        gc_info = self.bb.GCInfo

        trackBall.tick()
        if not self.bb.motionConnected:
            self.timer.restart()

        if not self.timer.finished():
            self.timer2.restart()
            self.initial.tick()
            self.crouch()
            self.lookAt(45, 0)
            return self.running()

        # if penalised, wait
        elif gc_info.penalised or gc_info.state is GCInfo.SET:
            self.lookAt(0, 0)
            self.crouch()
            return self.running()

        else:
            if gc_info.connected and gc_info.state is GCInfo.INITIAL:
                self.bb.reentry = False
                return self.success()

            if gc_info.connected and gc_info.state is GCInfo.PLAYING and self.bb.see_ball and self.bb.see_ball_cnt > 5:
                self.bb.reentry = False
                return self.success()

            self.dest = self.bb.get_default_dest()
            if not self.got_dest_loose(self.dest):
                self.gotoGlobal(self.dest)
                return self.running()
            else:
                self.crouch()
                self.bb.reentry = False
                return self.success()

