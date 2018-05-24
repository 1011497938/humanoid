from DecisionMaking.BehaviorTree.Task import Action
from DecisionMaking.BehaviorTree.Branch import parallel
from headskills.ScanField_fast import ScanField_fast
from headskills.ScanField import ScanField
from utils.VecPos import VecPos
from Timer import Timer
from dmsgs.msg import BehaviorInfo
from FieldGeometry import *

class _Ready(Action):
    def init(self):
        # self.t1 = Timer(self.bb.parameters.motionInitTime)
        self.dest = None

    def onEnter(self):
        pass

    def tick(self):
        if not self.bb.t1.finished():
            self.crouch()
            return self.running()
        else:
            self.dest = self.bb.get_default_dest()
            if not self.got_dest_loose(self.dest):
                self.gotoGlobal(self.dest)
                return self.running()
            else:
                self.crouch()
                return self.running()

# if see circle, track circle
# todo, set point, role point

Ready = parallel(_Ready, ScanField)
